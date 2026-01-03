import subprocess
from pathlib import Path
from typing import Dict, Optional

from mf_automated_perception.procedure.core.procedure_base import (
  GrainBase,
  GrainKey,
)
from mf_automated_perception.procedure.defs.test_basic_procedures.definition import (
  TestBasicProcedures,
)


class TestBasicProceduresImpl(TestBasicProcedures):
  """
  Implementation of basic pytest runner.
  """

  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    if input_grains:
      raise RuntimeError(
        f"{self.key}: input_grains should be empty, "
        f"got: {list(input_grains.keys())}"
      )

    repo_root = Path("/workspace")
    tests_dir = repo_root / "tests"

    test_targets = [
      tests_dir / "grain",
      tests_dir / "procedure",
      tests_dir / "realtime",
    ]

    for target in test_targets:
      logger.info(f"Running pytest: {target}")

      cmd = [
        "python3",
        "-m",
        "pytest",
        "-s",
        str(target),
      ]

      subprocess.run(
        cmd,
        cwd=repo_root,
        check=True,
      )

    logger.info("All tests passed")
