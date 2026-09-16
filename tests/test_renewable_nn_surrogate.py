"""Fast contract tests for the optional CAS neural surrogate."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

NUMERICS = Path(__file__).parents[1] / "docs/research/market_related/renewable-sample-factory/numerics"
sys.path.insert(0, str(NUMERICS))
import nn_surrogate as nn  # noqa: E402


def _toy(n: int = 24):
    rng = np.random.default_rng(4)
    x = rng.normal(size=(n, 24)).astype(np.float32)
    u = (x[:, :16] > 0).astype(np.float32)
    price = np.repeat((20.0 + x[:, :8]), 4, axis=1).astype(np.float32)
    cost = (100.0 + x[:, :1].ravel() * 4.0).astype(np.float32)
    return {"x": x, "u": u, "price": price, "cost": cost,
            "epsilon": np.zeros(n, dtype=np.float32)}


def test_multitask_training_and_checkpoint(tmp_path):
    data = _toy()
    cfg = nn.SurrogateConfig(horizon=8, n_generators=2, n_buses=4,
                             hidden=12, latent=16, epochs=3, patience=3,
                             batch_size=8)
    model, report = nn.train_model({k: v[:18] for k, v in data.items()},
                                   {k: v[18:] for k, v in data.items()}, cfg, seed=5)
    assert report["epochs_ran"] >= 1
    pred = nn.predict(model, data["x"][18:])
    assert pred["commitment"].shape == (6, 16)
    assert pred["price"].shape == (6, 32)
    assert np.isfinite(pred["cost"]).all()
    path = tmp_path / "model.pt"
    nn.save_checkpoint(str(path), model)
    restored = nn.load_checkpoint(str(path))
    np.testing.assert_allclose(pred["cost"], nn.predict(restored, data["x"][18:])["cost"], atol=1e-5)

