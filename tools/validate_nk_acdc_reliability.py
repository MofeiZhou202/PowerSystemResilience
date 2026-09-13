#!/usr/bin/env python3
"""Independent identity and scope oracle for the N-k AC/DC study."""

from __future__ import annotations

import json
import math
import pathlib
import subprocess
import sys
import tempfile


ABS_TOL = 1e-12


def assert_close(actual: float, expected: float, message: str) -> None:
    if not math.isclose(actual, expected, rel_tol=1e-11, abs_tol=ABS_TOL):
        raise AssertionError(f"{message}: {actual} != {expected}")


def main() -> int:
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "usage: validate_nk_acdc_reliability.py "
            "<study-executable> [existing-report.json]"
        )
    executable = pathlib.Path(sys.argv[1]).resolve()
    if len(sys.argv) == 3:
        report_path = pathlib.Path(sys.argv[2]).resolve()
        report = json.loads(report_path.read_text(encoding="utf-8"))
    else:
        with tempfile.TemporaryDirectory(prefix="nk-acdc-oracle-") as directory:
            report_path = pathlib.Path(directory) / "report.json"
            completed = subprocess.run(
                [
                    str(executable),
                    "--smoke",
                    "--audit-only",
                    "--output",
                    str(report_path),
                ],
                check=False,
                text=True,
                capture_output=True,
            )
            if completed.returncode != 0:
                raise RuntimeError(completed.stderr or completed.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))

    if not report["cases"]:
        raise AssertionError("oracle expected at least one study case")
    for case in report["cases"]:
        catalog = case["catalog"]
        events = {row["key"]: row for row in case["event_catalog"]}
        if len(events) != len(case["event_catalog"]):
            raise AssertionError("canonical N-k event keys are not unique")
        initiating_assets = catalog.get(
            "initiating_assets", case["network"]["initiating_assets"]
        )
        for order_row in catalog["events_by_order"]:
            order = order_row["order"]
            expected_full_count = math.comb(initiating_assets, order)
            if order_row["joint_events"] != expected_full_count:
                raise AssertionError(
                    "current complete authored catalog does not match the "
                    f"canonical order-{order} subset count"
                )
        event_asset_sets = []
        for event in case["event_catalog"]:
            assets = tuple(
                sorted((asset["kind"], asset["component_index"])
                       for asset in event["assets"])
            )
            if len(assets) != event["order"] or len(set(assets)) != len(assets):
                raise AssertionError("event order or component uniqueness mismatch")
            event_asset_sets.append(assets)
        if len(set(event_asset_sets)) != len(event_asset_sets):
            raise AssertionError("N-k subset recursion emitted a duplicate event")
        states = {row["label"]: row for row in case["operating_state_design"]}
        rows = case["conditional_results"]
        expected = sum(row["joint_events"] for row in catalog["events_by_order"])
        expected *= catalog["operating_states"]
        if expected != catalog["conditional_states"]:
            raise AssertionError("N-k combinatorial catalog dimension mismatch")

        resolved_frequency = 0.0
        unresolved_frequency = 0.0
        static_eens = 0.0
        dynamic_eens = 0.0
        groups: dict[tuple[str, str], list[float]] = {}
        resolved_rows = []
        attempted_dae_time = 0.0
        loo_estimate = 0.0
        loo_reference = 0.0
        loo_absolute_error = 0.0
        loo_reference_absolute = 0.0
        loo_unresolved_frequency = 0.0
        loo_misclassified_frequency = 0.0
        loo_audited_frequency = 0.0
        for row in rows:
            event = events[row["event_key"]]
            state = states[row["operating_state"]]
            assert_close(
                row["event_frequency_per_year"],
                event["frequency_per_year"],
                "conditional event frequency mismatch",
            )
            assert_close(
                row["operating_probability"],
                state["probability"],
                "conditional operating probability mismatch",
            )
            exposure = event["frequency_per_year"] * state["probability"]
            assert_close(
                row["exposure_frequency_per_year"],
                exposure,
                "conditional exposure mismatch",
            )
            attempted_dae_time += row["trajectory_runtime_s"]
            if not row["resolved"]:
                unresolved_frequency += exposure
                continue
            resolved_rows.append(row)
            resolved_frequency += exposure
            static_eens += exposure * row["static_consequence_mwh"]
            dynamic_eens += exposure * row["dynamic_consequence_mwh"]
            group = groups.setdefault((row["event_key"], row["class_label"]), [0.0, 0.0])
            group[0] += state["probability"]
            group[1] += state["probability"] * row["dynamic_consequence_mwh"]

            if not row["loo_has_neighbor"]:
                loo_unresolved_frequency += exposure
                continue
            loo_audited_frequency += exposure
            if row["loo_predicted_class_label"] != row["class_label"]:
                loo_misclassified_frequency += exposure
            if not row["loo_restoration_resolved"]:
                loo_unresolved_frequency += exposure
                continue
            predicted = row["loo_predicted_dynamic_consequence_mwh"]
            reference = row["dynamic_consequence_mwh"]
            loo_estimate += exposure * predicted
            loo_reference += exposure * reference
            loo_absolute_error += exposure * abs(predicted - reference)
            loo_reference_absolute += exposure * abs(reference)

        assert_close(
            resolved_frequency,
            catalog["resolved_frequency_per_year"],
            "resolved frequency mismatch",
        )
        assert_close(
            unresolved_frequency,
            catalog["unresolved_frequency_per_year"],
            "unresolved frequency mismatch",
        )
        necessity = case["transient_necessity"]
        assert_close(
            static_eens,
            necessity["static_eens_mwh_per_year"],
            "static EENS mismatch",
        )
        assert_close(
            dynamic_eens,
            necessity["transition_aware_eens_mwh_per_year"],
            "transition-aware EENS mismatch",
        )

        class_result = case["class_accuracy"]
        class_eens = sum(events[key[0]]["frequency_per_year"] * values[1]
                         for key, values in groups.items())
        assert_close(dynamic_eens, class_result["direct_eens_mwh_per_year"],
                     "class direct EENS mismatch")
        assert_close(class_eens, class_result["class_eens_mwh_per_year"],
                     "class aggregation mismatch")
        independently_recomputed_error = abs(dynamic_eens - class_eens)
        assert_close(independently_recomputed_error,
                     class_result["aggregation_error_mwh_per_year"],
                     "reported class aggregation error is inconsistent")
        if independently_recomputed_error > 1e-10:
            raise AssertionError("class aggregation identity failed")
        if len(groups) != class_result["class_records"]:
            raise AssertionError("class-record count mismatch")
        # A class is the inverse image of the restoration-entry status for one
        # fixed initiating event. Equal VSC labels across two events remain two
        # interface classes because their initiating outage vectors differ.
        labels_to_events: dict[str, set[str]] = {}
        for event_key, label in groups:
            labels_to_events.setdefault(label, set()).add(event_key)
        cross_event_label = any(len(event_keys) > 1
                                for event_keys in labels_to_events.values())
        if cross_event_label:
            event_conditioned_count = sum(
                len(event_keys) for event_keys in labels_to_events.values()
            )
            if event_conditioned_count != len(groups):
                raise AssertionError("equal VSC labels were merged across events")

        assert_close(loo_estimate,
                     class_result["leave_one_out_estimate_mwh_per_year"],
                     "leave-one-out estimate mismatch")
        assert_close(loo_reference,
                     class_result["leave_one_out_reference_mwh_per_year"],
                     "leave-one-out reference mismatch")
        assert_close(loo_unresolved_frequency,
                     class_result["leave_one_out_unresolved_frequency_per_year"],
                     "leave-one-out unresolved frequency mismatch")
        assert_close(loo_misclassified_frequency,
                     class_result["leave_one_out_misclassified_frequency_per_year"],
                     "leave-one-out misclassification frequency mismatch")
        # reliability_assessment_models.md, "leave-one-out operating-state
        # validation": ratios are defined only when their weighted reference
        # has coverage. The C++ producer uses +inf internally, serialized as
        # JSON null, so the oracle must validate the availability flag instead
        # of inventing a finite zero for an undefined relative error.
        loo_has_coverage = loo_reference > ABS_TOL
        if bool(class_result["leave_one_out_has_coverage"]) != loo_has_coverage:
            raise AssertionError("leave-one-out coverage flag mismatch")
        reported_relative = class_result["leave_one_out_relative_error"]
        if loo_has_coverage:
            loo_relative = abs(loo_estimate - loo_reference) / abs(loo_reference)
            assert_close(loo_relative, reported_relative,
                         "leave-one-out relative error mismatch")
        elif reported_relative is not None:
            raise AssertionError("uncovered leave-one-out relative error must be null")

        loo_absolute_has_coverage = loo_reference_absolute > ABS_TOL
        reported_absolute_relative = class_result[
            "leave_one_out_weighted_absolute_relative_error"
        ]
        if loo_absolute_has_coverage:
            loo_absolute_relative = loo_absolute_error / loo_reference_absolute
            assert_close(
                loo_absolute_relative,
                reported_absolute_relative,
                "leave-one-out weighted absolute error mismatch",
            )
        elif reported_absolute_relative is not None:
            raise AssertionError(
                "uncovered leave-one-out weighted absolute error must be null"
            )

        loo_class_accuracy = (
            1.0 - loo_misclassified_frequency / loo_audited_frequency
            if loo_audited_frequency > ABS_TOL
            else 0.0
        )
        assert_close(
            loo_class_accuracy,
            class_result["leave_one_out_entry_state_accuracy"],
            "leave-one-out entry-state accuracy mismatch",
        )

        compression = case["compression"]
        if len(groups) != compression["class_records"]:
            raise AssertionError("compression class-record count mismatch")
        if len(resolved_rows) != compression["resolved_conditional_states"]:
            raise AssertionError("compression resolved-state count mismatch")
        assert_close(attempted_dae_time,
                     compression["dae_library_attempt_wall_time_s"],
                     "DAE-library timing mismatch")
        projected_dae_library = attempted_dae_time + compression["lookup_wall_time_s"]
        assert_close(projected_dae_library,
                     compression["projected_dae_library_plus_lookup_time_s"],
                     "projected DAE-library cost mismatch")

        # The producer represents undefined zero-sample means and their derived
        # projections as +inf/NaN internally, which nlohmann_json emits as null.
        # A zero here is not a valid timing observation.
        if not resolved_rows:
            nullable_fields = (
                "class_to_state_ratio",
                "measured_mean_dae_time_s",
                "measured_mean_direct_condition_time_s",
                "measured_mean_static_restoration_time_s",
                "measured_mean_dynamic_restoration_time_s",
                "projected_direct_mc_wall_time_s",
                "projected_class_mc_wall_time_s",
                "projected_end_to_end_speedup",
            )
            for field in nullable_fields:
                if compression[field] is not None:
                    raise AssertionError(
                        f"zero-sample compression field {field} must be null"
                    )
            expected_saving = False
        else:
            resolved_count = len(resolved_rows)
            expected_ratio = len(groups) / resolved_count
            assert_close(expected_ratio, compression["class_to_state_ratio"],
                         "class-to-state ratio mismatch")
            mean_dae = sum(
                row["trajectory_runtime_s"] for row in resolved_rows
            ) / resolved_count
            mean_direct = sum(
                row["direct_condition_runtime_s"] for row in resolved_rows
            ) / resolved_count
            mean_static_restoration = sum(
                row["static_restoration_runtime_s"] for row in resolved_rows
            ) / resolved_count
            mean_dynamic_restoration = sum(
                row["dynamic_restoration_runtime_s"] for row in resolved_rows
            ) / resolved_count
            assert_close(mean_dae, compression["measured_mean_dae_time_s"],
                         "mean DAE time mismatch")
            assert_close(mean_direct,
                         compression["measured_mean_direct_condition_time_s"],
                         "mean direct time mismatch")
            assert_close(mean_static_restoration,
                         compression["measured_mean_static_restoration_time_s"],
                         "mean static restoration time mismatch")
            assert_close(mean_dynamic_restoration,
                         compression["measured_mean_dynamic_restoration_time_s"],
                         "mean dynamic restoration time mismatch")
            projected_direct = compression["mc_samples"] * mean_direct
            projected_class = (
                projected_dae_library
                + compression["mc_samples"] * mean_dynamic_restoration
            )
            assert_close(projected_direct,
                         compression["projected_direct_mc_wall_time_s"],
                         "projected direct cost mismatch")
            assert_close(projected_class,
                         compression["projected_class_mc_wall_time_s"],
                         "projected class cost mismatch")
            assert_close(projected_direct / max(ABS_TOL, projected_class),
                         compression["projected_end_to_end_speedup"],
                         "projected speedup mismatch")
            expected_saving = projected_direct > projected_class

        timing_admissible = unresolved_frequency <= 1e-12
        if compression["timing_projection_admissible"] != timing_admissible:
            raise AssertionError("timing-projection admissibility mismatch")
        expected_saving = timing_admissible and expected_saving
        if compression["net_computational_saving_demonstrated"] != expected_saving:
            raise AssertionError("net-saving admission mismatch")
        expected_reduction = 1.0 - (
            catalog["conditional_states"] / compression["mc_samples"]
        )
        assert_close(
            expected_reduction,
            compression["dae_solve_reduction_fraction_at_mc_samples"],
            "DAE solve-reduction calculation mismatch",
        )
        if catalog["entry_model"] != "explicit simultaneous joint-entry frequency":
            raise AssertionError("oracle rejects an ambiguous N-k frequency model")
        if catalog["marginal_frequency_products_used"]:
            raise AssertionError("oracle rejects products of marginal outage rates")
        if not any("positive-sequence" in item for item in case["model_limitations"]):
            raise AssertionError("IEEE 123 phase-domain limitation is not disclosed")

    print("N-k AC/DC oracle passed")
    for case in report["cases"]:
        print(
            case["case"],
            "states=", case["catalog"]["conditional_states"],
            "classes=", case["class_accuracy"]["class_records"],
            "aggregation_error=", case["class_accuracy"]["aggregation_error_mwh_per_year"],
            "loo_entry_accuracy=", case["class_accuracy"]["leave_one_out_entry_state_accuracy"],
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
