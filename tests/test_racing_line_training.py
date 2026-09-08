from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest


def load_trainer() -> ModuleType:
    path = Path(__file__).parents[1] / "scripts" / "train_racing_line_cmaes.py"
    spec = importlib.util.spec_from_file_location("train_racing_line_cmaes", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_resume_configuration_rejects_any_changed_setting() -> None:
    trainer = load_trainer()
    expected = {"stage": "joint", "generations": 20, "training_seeds": [19, 53]}

    trainer._assert_resume_configuration(expected, expected)
    with pytest.raises(ValueError, match="training_seeds"):
        trainer._assert_resume_configuration(
            {**expected, "training_seeds": [19, 54]},
            expected,
        )


def test_completed_generations_ignores_partial_candidate_records(tmp_path: Path) -> None:
    trainer = load_trainer()
    metrics = tmp_path / "metrics.jsonl"
    records = (
        {"record_type": "initial_mean"},
        {"record_type": "individual", "generation": 2},
        {"record_type": "generation", "generation": 1},
        {"record_type": "individual", "generation": 2},
    )
    metrics.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")

    assert trainer._completed_generations(metrics) == [{"record_type": "generation", "generation": 1}]


def test_atomic_json_writer_leaves_no_temporary_file(tmp_path: Path) -> None:
    trainer = load_trainer()
    destination = tmp_path / "checkpoint.json"

    trainer._write_json(destination, {"generation": 11})

    assert json.loads(destination.read_text(encoding="utf-8")) == {"generation": 11}
    assert not destination.with_suffix(".json.tmp").exists()
