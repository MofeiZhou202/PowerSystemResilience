#!/usr/bin/env python3.8
"""Benchmark Phase 5 L2O neural branching priorities on SCUC.

Default case:
    IEEE-118, T=24, wind+solar, StrictHiGHS/native branch-and-cut path.

The script trains or loads a small NumPy neural network from branch-derived
pseudocost gain labels, uses the network to score generator-time branching
priorities, and optionally gates the policy against a validation solve before
benchmarking. The production solver remains C++; Python owns data generation,
model training, artifact persistence, and policy evaluation.
"""

import argparse
import copy
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np  # type: ignore[import-not-found]

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "build_mipsolvers"))

import mipsolvers  # type: ignore[import-not-found]  # noqa: E402


ARTIFACT_SCHEMA = "mipsolvers.l2o.phase5.branching_mlp.v1"


def parse_budgets(text):
    return [float(item) for item in text.split(",") if item.strip()]


def make_options(limit, gap, seed, lp_kernel_backend=None):
    opt = mipsolvers.engine.BCOptions()
    opt.time_limit_sec = limit
    opt.gap_tol = gap
    opt.num_threads = 1
    opt.deterministic_parallel = True
    opt.random_seed = seed
    if lp_kernel_backend is not None:
        opt.lp_kernel_backend = lp_kernel_backend
    return opt


def selected_native_lp_backend(args):
    if args.native_lp_kernel == "highs":
        return mipsolvers.engine.LpKernelBackend.HiGHS
    return mipsolvers.engine.LpKernelBackend.ExperimentalNative


def solver_path_name(strict_highs, lp_kernel_backend):
    if strict_highs:
        return "StrictHiGHS(vendored_highs_mip_lifecycle)"
    if lp_kernel_backend == mipsolvers.engine.LpKernelBackend.HiGHS:
        return "NativeBranchAndCut(vendored_highs_lp_kernel)"
    return "NativeBranchAndCut(self_developed_lp_kernel)"


def mean(rows, key):
    return statistics.mean(row[key] for row in rows)


def sigmoid(values):
    clipped = np.clip(values, -40.0, 40.0)
    return 1.0 / (1.0 + np.exp(-clipped))


def binary_cross_entropy(prob, target, pos_weight, sample_weight=None):
    eps = 1e-8
    if sample_weight is None:
        sample_weight = np.ones_like(target)
    weight = (target * pos_weight + (1.0 - target)) * sample_weight
    loss = -weight * (target * np.log(prob + eps) + (1.0 - target) * np.log(1.0 - prob + eps))
    denom = max(float(np.sum(sample_weight)), 1.0)
    return float(np.sum(loss) / denom)


class BranchingScoreNetwork:
    """One-hidden-layer MLP for branch-derived UC score labels."""

    def __init__(self, input_dim, hidden_dim, output_dim=3, seed=0):
        rng = np.random.default_rng(seed)
        self.input_dim = int(input_dim)
        self.hidden_dim = int(hidden_dim)
        self.output_dim = int(output_dim)
        self.mean = np.zeros((self.input_dim,), dtype=float)
        self.std = np.ones((self.input_dim,), dtype=float)
        self.w1 = rng.normal(0.0, math.sqrt(2.0 / max(1, self.input_dim)),
                             size=(self.input_dim, self.hidden_dim))
        self.b1 = np.zeros((self.hidden_dim,), dtype=float)
        self.w2 = rng.normal(0.0, math.sqrt(2.0 / max(1, self.hidden_dim)),
                             size=(self.hidden_dim, self.output_dim))
        self.b2 = np.zeros((self.output_dim,), dtype=float)

    def _standardize(self, features):
        return (features - self.mean) / self.std

    def _forward_standardized(self, standardized):
        hidden_linear = standardized @ self.w1 + self.b1
        hidden = np.tanh(hidden_linear)
        logits = hidden @ self.w2 + self.b2
        prob = sigmoid(logits)
        return hidden, prob

    def fit(self, features, labels, sample_weights, epochs, learning_rate, l2, validation_fraction, seed):
        features = np.asarray(features, dtype=float)
        labels = np.asarray(labels, dtype=float)
        if sample_weights is None:
            sample_weights = np.ones_like(labels)
        sample_weights = np.asarray(sample_weights, dtype=float)
        if features.ndim != 2 or labels.ndim != 2:
            raise ValueError("features and labels must be two-dimensional")
        if features.shape[0] != labels.shape[0]:
            raise ValueError("features and labels must have the same row count")
        if sample_weights.shape != labels.shape:
            raise ValueError("sample_weights must match labels")
        if features.shape[0] < 8:
            raise ValueError("not enough training samples for branching network")
        if float(np.sum(sample_weights)) <= 0.0:
            raise ValueError("branching network has zero label weight")

        self.mean = features.mean(axis=0)
        self.std = features.std(axis=0)
        self.std[self.std < 1e-8] = 1.0
        standardized = self._standardize(features)

        rng = np.random.default_rng(seed)
        order = rng.permutation(features.shape[0])
        n_val = int(round(features.shape[0] * validation_fraction))
        n_val = min(max(n_val, 1), max(1, features.shape[0] // 3))
        val_idx = order[:n_val]
        train_idx = order[n_val:]
        if train_idx.size == 0:
            train_idx = val_idx
        x_train = standardized[train_idx]
        y_train = labels[train_idx]
        w_train = sample_weights[train_idx]
        x_val = standardized[val_idx]
        y_val = labels[val_idx]
        w_val = sample_weights[val_idx]

        total_weight = np.maximum(w_train.sum(axis=0), 1.0)
        positive = np.maximum((w_train * y_train).sum(axis=0), 1.0)
        negative = np.maximum(total_weight - positive, 1.0)
        pos_weight = np.clip(negative / positive, 1.0, 20.0)

        history = []
        best = None
        for epoch in range(int(epochs)):
            hidden, prob = self._forward_standardized(x_train)
            weight = (y_train * pos_weight + (1.0 - y_train)) * w_train
            grad_logits = (prob - y_train) * weight / max(1.0, float(np.sum(w_train)))

            grad_w2 = hidden.T @ grad_logits + l2 * self.w2
            grad_b2 = grad_logits.sum(axis=0)
            grad_hidden = (grad_logits @ self.w2.T) * (1.0 - hidden * hidden)
            grad_w1 = x_train.T @ grad_hidden + l2 * self.w1
            grad_b1 = grad_hidden.sum(axis=0)

            self.w2 -= learning_rate * grad_w2
            self.b2 -= learning_rate * grad_b2
            self.w1 -= learning_rate * grad_w1
            self.b1 -= learning_rate * grad_b1

            if epoch == int(epochs) - 1 or epoch % max(1, int(epochs) // 10) == 0:
                _, train_prob = self._forward_standardized(x_train)
                _, val_prob = self._forward_standardized(x_val)
                train_loss = binary_cross_entropy(train_prob, y_train, pos_weight, w_train)
                val_loss = binary_cross_entropy(val_prob, y_val, pos_weight, w_val)
                record = {"epoch": epoch + 1, "train_loss": train_loss, "validation_loss": val_loss}
                history.append(record)
                if best is None or val_loss < best["validation_loss"]:
                    best = dict(record)

        return {
            "samples": int(features.shape[0]),
            "train_samples": int(x_train.shape[0]),
            "validation_samples": int(x_val.shape[0]),
            "positive_rate": labels.mean(axis=0).tolist(),
            "weighted_positive_rate": (np.sum(sample_weights * labels, axis=0) /
                                       np.maximum(np.sum(sample_weights, axis=0), 1.0)).tolist(),
            "label_weight_sum": float(np.sum(sample_weights)),
            "positive_weight": pos_weight.tolist(),
            "best": best or {},
            "last": history[-1] if history else {},
            "history": history,
        }

    def predict_probabilities(self, features):
        features = np.asarray(features, dtype=float)
        _, prob = self._forward_standardized(self._standardize(features))
        return prob

    def predict_scores(self, case):
        feature_pack = mipsolvers.l2o.scuc_generator_time_features(case)
        features = np.asarray(feature_pack["features"], dtype=float)
        ng, periods, n_features = features.shape
        probabilities = self.predict_probabilities(features.reshape(-1, n_features))
        score = probabilities.max(axis=1).reshape(ng, periods)
        return score, {
            "feature_names": list(feature_pack["feature_names"]),
            "output_names": ["commitment", "startup", "shutdown"],
            "score_aggregation": "max_neural_output_probability",
            "score_min": float(score.min()),
            "score_max": float(score.max()),
            "score_mean": float(score.mean()),
        }

    def to_dict(self):
        return {
            "input_dim": self.input_dim,
            "hidden_dim": self.hidden_dim,
            "output_dim": self.output_dim,
            "mean": self.mean.tolist(),
            "std": self.std.tolist(),
            "w1": self.w1.tolist(),
            "b1": self.b1.tolist(),
            "w2": self.w2.tolist(),
            "b2": self.b2.tolist(),
        }

    @classmethod
    def from_dict(cls, payload):
        model = cls(payload["input_dim"], payload["hidden_dim"], payload.get("output_dim", 3))
        model.mean = np.asarray(payload["mean"], dtype=float)
        model.std = np.asarray(payload["std"], dtype=float)
        model.w1 = np.asarray(payload["w1"], dtype=float)
        model.b1 = np.asarray(payload["b1"], dtype=float)
        model.w2 = np.asarray(payload["w2"], dtype=float)
        model.b2 = np.asarray(payload["b2"], dtype=float)
        return model


def save_model_artifact(path, model, metadata):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": ARTIFACT_SCHEMA,
        "model": model.to_dict(),
        "metadata": metadata,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def load_model_artifact(path):
    payload = json.loads(Path(path).read_text())
    if payload.get("schema") != ARTIFACT_SCHEMA:
        raise ValueError("unsupported Phase 5 branching model artifact schema")
    return BranchingScoreNetwork.from_dict(payload["model"]), payload.get("metadata", {})


def parse_int_list(text):
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(int(item))
    return values


def parse_scenarios(text):
    scenarios = []
    for item in text.split(","):
        name = item.strip().lower()
        if not name:
            continue
        if name in ("base", "none"):
            scenarios.append((name, False, False))
        elif name == "wind":
            scenarios.append((name, True, False))
        elif name == "solar":
            scenarios.append((name, False, True))
        elif name in ("wind+solar", "solar+wind", "renewables"):
            scenarios.append(("wind+solar", True, True))
        else:
            raise ValueError("unknown training scenario: {}".format(item))
    if not scenarios:
        raise ValueError("at least one training scenario is required")
    return scenarios


def build_ieee118_case(periods, wind, solar):
    return mipsolvers.scuc.build_ieee118_case(T=periods, with_wind=wind, with_solar=solar)


def extract_uc_matrix(solution, cols, ng, periods):
    col_array = np.asarray(cols, dtype=int).reshape(ng, periods)
    out = np.zeros((ng, periods), dtype=float)
    valid = (col_array >= 0) & (col_array < solution.size)
    out[valid] = solution[col_array[valid]]
    return np.clip(out, 0.0, 1.0)


def uc_hint_for_case(case):
    summary = mipsolvers.l2o.scuc_mip_summary(case)
    return summary["uc_hint"]


def extract_incumbent_training_labels(case, result):
    hint = uc_hint_for_case(case)
    ng = int(hint["ng"])
    periods = int(hint["T"])
    solution = np.asarray(result["x"], dtype=float)
    commitment = extract_uc_matrix(solution, hint["ig_cols"], ng, periods)
    startup = extract_uc_matrix(solution, hint["su_cols"], ng, periods)
    shutdown = extract_uc_matrix(solution, hint["sd_cols"], ng, periods)
    labels = np.stack([commitment, startup, shutdown], axis=2)
    weights = np.ones_like(labels)
    metadata = {
        "label_source": "incumbent_solution_proxy",
        "observed_entries": int(labels.size),
        "weighted_entries": float(weights.sum()),
        "message": "fallback warm-start style labels; not branch-regret labels",
    }
    return labels, weights, metadata


def vector_from_artifact(pc, key, n_cols):
    values = np.asarray(pc.get(key, []), dtype=float)
    if values.size < n_cols:
        out = np.zeros((n_cols,), dtype=float)
        out[:values.size] = values
        return out
    return values[:n_cols]


def normalize_observed_utilities(labels, weights):
    observed = weights > 0.0
    if not np.any(observed):
        return labels
    transformed = np.zeros_like(labels)
    transformed[observed] = np.log1p(np.maximum(0.0, labels[observed]))
    max_value = float(np.max(transformed[observed]))
    if max_value > 1e-12:
        transformed[observed] /= max_value
    return transformed


def extract_pseudocost_training_labels(case, result, beta, min_observed):
    hint = uc_hint_for_case(case)
    ng = int(hint["ng"])
    periods = int(hint["T"])
    n_cols = int(result.get("artifacts", {}).get("pseudocost_init", {}).get("n_orig_cols", 0))
    pc = result.get("artifacts", {}).get("pseudocost_init", {})
    if not pc or n_cols <= 0:
        raise ValueError("pseudocost artifact is missing; increase label budget or use --label-source incumbent")

    up = vector_from_artifact(pc, "pseudocostup", n_cols)
    down = vector_from_artifact(pc, "pseudocostdown", n_cols)
    samples_up = vector_from_artifact(pc, "nsamplesup", n_cols)
    samples_down = vector_from_artifact(pc, "nsamplesdown", n_cols)
    inferences_up = vector_from_artifact(pc, "ninferencesup", n_cols)
    inferences_down = vector_from_artifact(pc, "ninferencesdown", n_cols)
    conflict_up = vector_from_artifact(pc, "conflictscoreup", n_cols)
    conflict_down = vector_from_artifact(pc, "conflictscoredown", n_cols)

    labels = np.zeros((ng, periods, 3), dtype=float)
    weights = np.zeros_like(labels)
    role_columns = [hint["ig_cols"], hint["su_cols"], hint["sd_cols"]]
    observed_entries = 0
    for role, cols in enumerate(role_columns):
        col_array = np.asarray(cols, dtype=int).reshape(ng, periods)
        valid = (col_array >= 0) & (col_array < n_cols)
        role_up = np.zeros((ng, periods), dtype=float)
        role_down = np.zeros((ng, periods), dtype=float)
        role_samples = np.zeros((ng, periods), dtype=float)
        role_inferences = np.zeros((ng, periods), dtype=float)
        role_conflict = np.zeros((ng, periods), dtype=float)
        role_up[valid] = up[col_array[valid]]
        role_down[valid] = down[col_array[valid]]
        role_samples[valid] = samples_up[col_array[valid]] + samples_down[col_array[valid]]
        role_inferences[valid] = inferences_up[col_array[valid]] + inferences_down[col_array[valid]]
        role_conflict[valid] = conflict_up[col_array[valid]] + conflict_down[col_array[valid]]
        observed = valid & ((role_samples > 0.0) | (role_inferences > 0.0) |
                            (role_up > 0.0) | (role_down > 0.0) | (role_conflict > 0.0))
        utility = np.minimum(np.maximum(0.0, role_up), np.maximum(0.0, role_down))
        utility += beta * np.maximum(np.maximum(0.0, role_up), np.maximum(0.0, role_down))
        utility += 0.05 * np.maximum(0.0, role_conflict)
        labels[:, :, role] = utility
        weights[:, :, role] = np.where(observed, np.minimum(1.0, 0.25 + 0.25 * role_samples +
                                                            0.05 * role_inferences), 0.0)
        observed_entries += int(np.count_nonzero(observed))

    if observed_entries < min_observed:
        raise ValueError("only {} pseudocost label entries observed; need at least {}".format(
            observed_entries, min_observed))
    labels = normalize_observed_utilities(labels, weights)
    metadata = {
        "label_source": "pseudocost_gain_artifact",
        "utility": "log1p_normalized_min_up_down_plus_beta_max_up_down_plus_conflict",
        "beta": float(beta),
        "observed_entries": int(observed_entries),
        "total_entries": int(labels.size),
        "weighted_entries": float(weights.sum()),
        "pseudocost_samples_total": float(pc.get("nsamplestotal", 0.0)),
        "pseudocost_inferences_total": float(pc.get("ninferencestotal", 0.0)),
        "pseudocost_cost_total": float(pc.get("cost_total", 0.0)),
        "message": "branch-derived labels from observed pseudocost gains in original column space",
    }
    return labels, weights, metadata


def solve_label_case(case, budget, gap, seed, strict_highs, include_artifact_vectors,
                     lp_kernel_backend):
    return mipsolvers.l2o.solve_scuc_mip(
        case,
        options=make_options(budget, gap, seed, lp_kernel_backend),
        strict_highs=strict_highs,
        include_artifact_vectors=include_artifact_vectors,
    )


def collect_training_data(periods_list, scenarios, label_budget, gap, seed, strict_highs, args):
    features_rows = []
    label_rows = []
    weight_rows = []
    runs = []
    feature_names = None
    for periods in periods_list:
        for scenario_name, wind, solar in scenarios:
            case = build_ieee118_case(periods, wind, solar)
            feature_pack = mipsolvers.l2o.scuc_generator_time_features(case)
            features = np.asarray(feature_pack["features"], dtype=float)
            if feature_names is None:
                feature_names = list(feature_pack["feature_names"])
            need_artifacts = args.label_source == "pseudocost"
            lp_kernel_backend = selected_native_lp_backend(args)
            result = solve_label_case(
                case, label_budget, gap, seed, strict_highs, need_artifacts,
                lp_kernel_backend)
            try:
                if args.label_source == "pseudocost":
                    labels, weights, label_metadata = extract_pseudocost_training_labels(
                        case, result, args.pseudocost_beta, args.min_branch_label_entries)
                else:
                    labels, weights, label_metadata = extract_incumbent_training_labels(case, result)
            except ValueError:
                if not args.allow_incumbent_fallback:
                    raise
                labels, weights, label_metadata = extract_incumbent_training_labels(case, result)
                label_metadata["fallback_from"] = args.label_source
            features_rows.append(features.reshape(-1, features.shape[2]))
            label_rows.append(labels.reshape(-1, labels.shape[2]))
            weight_rows.append(weights.reshape(-1, weights.shape[2]))
            runs.append({
                "periods": int(periods),
                "scenario": scenario_name,
                "objective": float(result["objective"]),
                "gap": float(result["mip_gap"]),
                "status": str(result.get("status", "")),
                "solver_sec": float(result.get("runtime_sec", label_budget)),
                "samples": int(features.shape[0] * features.shape[1]),
                "nodes": int(result.get("bc_stats", {}).get("nodes_explored", 0)),
                "solver_path": solver_path_name(strict_highs, lp_kernel_backend),
                "native_lp_kernel": args.native_lp_kernel if not strict_highs else "strict_highs",
                "label_metadata": label_metadata,
            })
    return np.vstack(features_rows), np.vstack(label_rows), np.vstack(weight_rows), feature_names or [], runs


def train_branching_model(args, strict_highs):
    periods_list = parse_int_list(args.train_periods)
    scenarios = parse_scenarios(args.train_scenarios)
    features, labels, sample_weights, feature_names, label_runs = collect_training_data(
        periods_list, scenarios, args.label_budget, args.gap, args.seed, strict_highs, args)
    model = BranchingScoreNetwork(
        input_dim=features.shape[1], hidden_dim=args.hidden_dim, output_dim=labels.shape[1], seed=args.seed)
    training = model.fit(
        features, labels, sample_weights, epochs=args.epochs, learning_rate=args.learning_rate,
        l2=args.l2, validation_fraction=args.validation_fraction, seed=args.seed)
    metadata = {
        "trained_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "model_type": "numpy_one_hidden_layer_mlp",
        "loss_name": "weighted_soft_binary_cross_entropy",
        "label_source": args.label_source,
        "phase5_status": "branch_derived_pseudocost_training" if args.label_source == "pseudocost"
                 else "bootstrap_only_not_final_branch_regret_loss",
        "feature_names": feature_names,
        "output_names": ["commitment", "startup", "shutdown"],
        "train_periods": periods_list,
        "train_scenarios": [name for name, _, _ in scenarios],
        "label_budget_sec": float(args.label_budget),
        "strict_highs_labels": bool(strict_highs),
        "solver_path": solver_path_name(strict_highs, selected_native_lp_backend(args)),
        "native_lp_kernel": args.native_lp_kernel if not strict_highs else "strict_highs",
        "label_runs": label_runs,
        "training": training,
    }
    return model, metadata


def compact_mip_summary(summary):
    keep = {}
    for key, value in summary.items():
        if isinstance(value, (int, float, str, bool)) or value is None:
            keep[key] = value
    for key in ("uc_hint", "fingerprint"):
        value = summary.get(key)
        if isinstance(value, dict):
            keep[key] = {
                subkey: subvalue
                for subkey, subvalue in value.items()
                if isinstance(subvalue, (int, float, str, bool)) or subvalue is None
            }
    for key in ("binary_idx", "integer_idx"):
        value = summary.get(key)
        if isinstance(value, list):
            keep[f"num_{key}"] = len(value)
    return keep


def solve_baseline(case, options, strict_highs):
    start = time.perf_counter()
    result = mipsolvers.l2o.solve_scuc_mip(
        case,
        options=options,
        strict_highs=strict_highs,
        include_artifact_vectors=False,
    )
    wall = time.perf_counter() - start
    return {
        "objective": float(result["objective"]),
        "gap": float(result["mip_gap"]),
        "solver_sec": float(result.get("runtime_sec", wall)),
        "wall_sec": wall,
        "nodes": int(result.get("bc_stats", {}).get("nodes_explored", 0)),
        "status": str(result.get("status", "")),
    }


def solve_phase5(case, scores, options, policy, strict_highs):
    start = time.perf_counter()
    result = mipsolvers.l2o.solve_scuc_mip_with_branching_policy(
        case,
        generator_time_scores=scores,
        options=options,
        policy_options=policy,
        strict_highs=strict_highs,
        include_artifact_vectors=False,
    )
    wall = time.perf_counter() - start
    report = result["branching_policy"]["report"]
    return {
        "objective": float(result["objective"]),
        "gap": float(result["mip_gap"]),
        "solver_sec": float(result.get("runtime_sec", wall)),
        "wall_sec": wall,
        "nodes": int(result.get("bc_stats", {}).get("nodes_explored", 0)),
        "status": str(result.get("status", "")),
        "priority_entries": int(report["num_priority_entries"]),
        "dynamic_prior": bool(report["installed_dynamic_prior"]),
        "neural_policy_used": True,
    }


def gated_baseline_result(baseline_result):
    result = copy.deepcopy(baseline_result)
    result["priority_entries"] = 0
    result["dynamic_prior"] = False
    result["neural_policy_used"] = False
    return result


def gate_neural_policy(case, scores, policy, budget, gap, seed, strict_highs, tolerance,
                       min_nodes, force_neural, lp_kernel_backend):
    if force_neural:
        return {"mode": "forced", "use_neural_policy": True, "message": "forced by --force-neural"}
    if budget <= 0.0:
        return {"mode": "disabled", "use_neural_policy": True, "message": "gate budget is zero"}

    options = make_options(budget, gap, seed, lp_kernel_backend)
    baseline = solve_baseline(case, options, strict_highs)
    neural = solve_phase5(
        case, scores, make_options(budget, gap, seed, lp_kernel_backend),
        policy, strict_highs)
    if max(baseline["nodes"], neural["nodes"]) < min_nodes:
        return {
            "mode": "validation_solve",
            "use_neural_policy": False,
            "gate_budget_sec": float(budget),
            "gap_tolerance": float(tolerance),
            "min_nodes": int(min_nodes),
            "message": "rejected: gate solve did not reach branch-tree evidence",
            "baseline": baseline,
            "neural": neural,
        }

    baseline_gap_finite = math.isfinite(baseline["gap"])
    neural_gap_finite = math.isfinite(neural["gap"])
    if baseline_gap_finite and neural_gap_finite:
        use_neural = neural["gap"] <= baseline["gap"] + tolerance
    elif not baseline_gap_finite and neural_gap_finite:
        use_neural = True
    elif not baseline_gap_finite and not neural_gap_finite:
        use_neural = neural["objective"] <= baseline["objective"]
    else:
        use_neural = False
    message = "accepted" if use_neural else "rejected: neural gate gap exceeded baseline tolerance"
    return {
        "mode": "validation_solve",
        "use_neural_policy": use_neural,
        "gate_budget_sec": float(budget),
        "gap_tolerance": float(tolerance),
        "min_nodes": int(min_nodes),
        "message": message,
        "baseline": baseline,
        "neural": neural,
    }


def print_table(rows):
    print()
    print("| Budget | Baseline obj | Baseline gap | Neural/gated obj | Neural/gated gap | Solver s baseline/policy | Nodes baseline/policy | Neural used | Gap reduction |")
    print("|---:|---:|---:|---:|---:|---:|---:|:---:|---:|")
    for row in rows:
        print(
            "| {budget:.2f}s | {baseline_objective:.6f} | {baseline_gap:.6f} | "
            "{phase5_objective:.6f} | {phase5_gap:.6f} | "
            "{baseline_solver_sec:.6f}/{phase5_solver_sec:.6f} | "
            "{baseline_nodes:.1f}/{phase5_nodes:.1f} | {phase5_neural_policy_used} | "
            "{gap_reduction:.6f} |".format(**row)
        )
    print()


def main():
    parser = argparse.ArgumentParser(description="Phase 5 L2O SCUC neural branching-priority benchmark")
    parser.add_argument("--periods", type=int, default=24, help="SCUC horizon length T")
    parser.add_argument("--budgets", default="60.0")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--gap", type=float, default=1e-4)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument("--native", action="store_true", help="Use the non-StrictHiGHS native selector path")
    parser.add_argument("--native-lp-kernel", choices=("self", "highs"), default="self",
                        help="LP kernel used when --native is set: self-developed native kernel or vendored HiGHS")
    parser.add_argument("--model-path", default=str(ROOT / "build_mipsolvers" / "l2o_phase5_branching_mlp.json"))
    parser.add_argument("--reuse-model", action="store_true", help="Load --model-path instead of retraining when it exists")
    parser.add_argument("--no-save-model", action="store_true", help="Do not write the trained model artifact")
    parser.add_argument("--train-periods", default="24", help="Comma-separated IEEE-118 horizons used for label solves")
    parser.add_argument("--train-scenarios", default="wind+solar",
                        help="Comma-separated training scenarios: base, wind, solar, wind+solar")
    parser.add_argument("--label-budget", type=float, default=120.0, help="Seconds per supervised label solve")
    parser.add_argument("--label-source", choices=("pseudocost", "incumbent"), default="pseudocost",
                        help="Training labels: branch-derived pseudocost gains or incumbent proxy labels")
    parser.add_argument("--allow-incumbent-fallback", action="store_true",
                        help="Fall back to incumbent proxy labels if branch-derived labels are unavailable")
    parser.add_argument("--pseudocost-beta", type=float, default=0.1,
                        help="Weight on the larger branch direction in pseudocost utility labels")
    parser.add_argument("--min-branch-label-entries", type=int, default=1,
                        help="Minimum observed pseudocost label entries required per label solve")
    parser.add_argument("--epochs", type=int, default=250)
    parser.add_argument("--hidden-dim", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=0.03)
    parser.add_argument("--l2", type=float, default=1e-4)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--gate-budget", type=float, default=60.0,
                        help="Validation-solve budget for accepting the neural policy; 0 disables gating")
    parser.add_argument("--gate-gap-tolerance", type=float, default=1e-5,
                        help="Allowed neural-policy gap regression during the gate solve")
    parser.add_argument("--gate-min-nodes", type=int, default=1,
                        help="Reject the policy if the gate solve explores fewer branch nodes than this")
    parser.add_argument("--force-neural", action="store_true", help="Evaluate neural scores even if the gate would reject them")
    args = parser.parse_args()

    budgets = parse_budgets(args.budgets)
    strict_highs = not args.native
    lp_kernel_backend = selected_native_lp_backend(args)
    case = mipsolvers.scuc.build_ieee118_case(T=args.periods, with_wind=True, with_solar=True)
    summary = compact_mip_summary(mipsolvers.l2o.scuc_mip_summary(case))

    model_path = Path(args.model_path)
    if args.reuse_model and model_path.exists():
        model, model_metadata = load_model_artifact(model_path)
        model_source = "loaded"
    else:
        model, model_metadata = train_branching_model(args, strict_highs)
        model_source = "trained"
        if not args.no_save_model:
            save_model_artifact(model_path, model, model_metadata)

    scores, score_metadata = model.predict_scores(case)

    policy = mipsolvers.l2o.SCUCBranchingPolicyOptions()
    policy.enable_static_priorities = True
    policy.enable_dynamic_priors = True
    policy.clear_existing_priorities = True
    policy.include_startup_shutdown = True
    policy.prefer_earlier_periods = True
    policy.learned_priority_scale = 1500.0
    policy.dynamic_prior_weight = 0.80

    priority_probe = mipsolvers.l2o.make_scuc_branching_priorities(case, scores, policy)
    print("MODEL", json.dumps({
        "source": model_source,
        "artifact_path": str(model_path),
        "metadata": model_metadata,
        "score_metadata": score_metadata,
    }, sort_keys=True))
    print("POLICY", json.dumps({
        "strict_highs": strict_highs,
        "solver_path": solver_path_name(strict_highs, lp_kernel_backend),
        "native_lp_kernel": args.native_lp_kernel if not strict_highs else "strict_highs",
        "periods": args.periods,
        "mip_summary": summary,
        "score_source": "neural_network",
        "score_min": float(scores.min()),
        "score_max": float(scores.max()),
        "priority_report": priority_probe["report"],
    }, sort_keys=True))
    gate = gate_neural_policy(
        case, scores, policy, args.gate_budget, args.gap, args.seed,
        strict_highs, args.gate_gap_tolerance, args.gate_min_nodes, args.force_neural,
        lp_kernel_backend)
    use_neural_policy = bool(gate["use_neural_policy"])
    print("GATE", json.dumps(gate, sort_keys=True))

    rows = []
    for budget in budgets:
        baseline_runs = []
        phase5_runs = []
        for _ in range(args.repeats):
            baseline = solve_baseline(
                case, make_options(budget, args.gap, args.seed, lp_kernel_backend),
                strict_highs)
            baseline_runs.append(baseline)
            if use_neural_policy:
                phase5_runs.append(solve_phase5(
                    case, scores,
                    make_options(budget, args.gap, args.seed, lp_kernel_backend),
                    policy, strict_highs))
            else:
                phase5_runs.append(gated_baseline_result(baseline))

        row = {
            "budget": budget,
            "baseline_objective": mean(baseline_runs, "objective"),
            "baseline_gap": mean(baseline_runs, "gap"),
            "baseline_solver_sec": mean(baseline_runs, "solver_sec"),
            "baseline_wall_sec": mean(baseline_runs, "wall_sec"),
            "baseline_nodes": mean(baseline_runs, "nodes"),
            "baseline_status": baseline_runs[-1]["status"],
            "phase5_objective": mean(phase5_runs, "objective"),
            "phase5_gap": mean(phase5_runs, "gap"),
            "phase5_solver_sec": mean(phase5_runs, "solver_sec"),
            "phase5_wall_sec": mean(phase5_runs, "wall_sec"),
            "phase5_nodes": mean(phase5_runs, "nodes"),
            "phase5_status": phase5_runs[-1]["status"],
            "phase5_priority_entries": phase5_runs[-1]["priority_entries"],
            "phase5_dynamic_prior": phase5_runs[-1]["dynamic_prior"],
            "phase5_neural_policy_used": bool(phase5_runs[-1]["neural_policy_used"]),
            "gate_mode": gate["mode"],
            "gate_message": gate["message"],
            "solver_path": solver_path_name(strict_highs, lp_kernel_backend),
            "native_lp_kernel": args.native_lp_kernel if not strict_highs else "strict_highs",
        }
        row["gap_reduction"] = row["baseline_gap"] - row["phase5_gap"]
        row["objective_delta"] = row["baseline_objective"] - row["phase5_objective"]
        rows.append(row)
        print("RESULT", json.dumps(row, sort_keys=True))

    print_table(rows)
    print("SUMMARY_JSON", json.dumps(rows, sort_keys=True))


if __name__ == "__main__":
    main()
