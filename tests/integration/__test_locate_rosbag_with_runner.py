import pytest
from mflib.perception.automated_perception.runner.gp_runner import GPRunner


# test
from mflib.perception.automated_perception.tests.integration_phase import (
  IntegrationPhase,
)
pytestmark = [
  pytest.mark.integration,
  pytest.mark.order(IntegrationPhase.LOCATE_ROSBAG_WITH_RUNNER),
]


def test_locate_rosbags_with_runner():
  """
  Integration test:
  - Execute ProcedureLocateRosbags via GPRunner
  - Verify output grain is created and usable
  """

  # ------------------------------------------------------------
  # arrange
  # ------------------------------------------------------------

  procedure_key = "locate_rosbags"
  config = {
    "search_root": "/workspace/data/mantis-eye/dongtan_automated_perception",
  }

  runner = GPRunner()

  # ------------------------------------------------------------
  # act
  # ------------------------------------------------------------
  output_grain = runner.run(
    procedure_key=procedure_key,
    grain_select_rule="latest",
    config=config,
    creator="test_user",
  )

  # ------------------------------------------------------------
  # assert
  # ------------------------------------------------------------
  from mflib.perception.automated_perception.grain.defs.raw_rosbag_path import (
    RawRosbagPath,
  )
  assert isinstance(output_grain, RawRosbagPath)
  assert output_grain.is_created

  # optional sanity check: DB exists
  db_path = output_grain.db_path
  assert db_path.exists(), f"Expected DB not found: {db_path}"
