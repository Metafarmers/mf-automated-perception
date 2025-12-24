import logging
import pytest
from mflib.perception.automated_perception.procedure.defs.locate_rosbags import LocateRosbags
from mflib.perception.automated_perception.grain.defs.raw_rosbag_path import RawRosbagPath
from mflib.perception.automated_perception.utils.gp_logger import get_logger

# test order
from mflib.perception.automated_perception.tests.integration_phase import IntegrationPhase
pytestmark = [
  pytest.mark.integration,
  pytest.mark.order(IntegrationPhase.LOCATE_ROSBAG_2_PROCEDURE),
]

def test_main():
  procedure = LocateRosbags()
  config = {
    'search_root': '/workspace/data/mantis-eye/dongtan_automated_perception',
  }

  output_grain = RawRosbagPath()

  log_root = output_grain.log_root

  logger = get_logger(
    name="ProcedureLocateRosbags",
    file_dir=log_root,
    level=logging.DEBUG,)
  output_grain.set_provenance(
    source_procedure=procedure.key,
    source_grain_keys=[],
    creator="test_user",
  )
  output_grain.create()

  procedure.run(
    input_grains={},
    output_grain=output_grain,
    config=config,
    logger=logger,
  )

  output_grain.close()
