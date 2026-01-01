from typing import ClassVar, Optional, Type, Tuple, Dict
from pathlib import Path
import subprocess

from pydantic import BaseModel

from mf_automated_perception.procedure.core.procedure_base import (
  ProcedureBase,
  GrainBase,
  GrainKey,
)




class TestBasicProcedures(ProcedureBase):
  key: ClassVar[str] = "test_basic_procedures"
  version: ClassVar[str] = "0.0.1"

  docker_image: ClassVar[str] = "mf-mantis-eye"
  docker_image_tag: ClassVar[str] = "latest"

  ParamModel: ClassVar[Optional[Type]] = None
  description: ClassVar[str] = "Run all basic pytest suites (grain, procedure, realtime)"

  input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey] = ()

  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase | None,
    config: None,
    logger,
  ) -> None:
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
        '-s',
        str(target),
      ]

      subprocess.run(cmd, cwd=repo_root, check=True)

    logger.info("All tests passed")
