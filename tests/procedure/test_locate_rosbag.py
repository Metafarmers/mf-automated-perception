import logging
from pathlib import Path

import yaml

from mf_automated_perception.env import EXTERNAL_DATA_ROOT, LOG_DIR_ROOT
from mf_automated_perception.grain.defs.raw_rosbag_path import RawRosbagPath
from mf_automated_perception.procedure.defs.locate_rosbags.definition import (
  LocateRosbags,
)
from mf_automated_perception.utils.gp_logger import get_logger


def test_main():
  procedure = LocateRosbags()

  # running a procedure requires config file
  param_type = procedure.get_param_type()
  parameter = param_type(
    search_root=str(EXTERNAL_DATA_ROOT),
  )
  config_file = Path('/tmp/locate_rosbags_config.yaml')
  config_file.write_text(
    yaml.safe_dump(parameter.model_dump(), sort_keys=False)
  )

  output_grain = RawRosbagPath()

  log_root = LOG_DIR_ROOT

  logger = get_logger(
    name="ProcedureLocateRosbags",
    file_dir=log_root,
    level=logging.DEBUG,)

  procedure.run(
    input_grains={},
    output_grain=output_grain,
    config_file=config_file,
    context={
      "creator": "test_user",
    },
    logger=logger,
  )

  output_grain.close()
