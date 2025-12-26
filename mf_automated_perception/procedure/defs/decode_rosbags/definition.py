from pathlib import Path
from typing import ClassVar, Dict, List, Optional, Tuple, Type

import yaml
from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase

from mf_automated_perception.utils.analyze_rosbag import (
  map_topics_and_intrinsics,
)


class DecodeRosbagsConfig(BaseModel):
  decode_images: bool = True
  decode_pointclouds: bool = True
  # decode_imu: bool = True
  # decode_tf: bool = True
  # tf_from_to_frames: Optional[List[Tuple[str, str]]] = None

class DecodeRosbags(ProcedureBase):
  key: ClassVar[str] = "decode_rosbags"
  version: ClassVar[str] = "1.0.0"
  ParamModel: ClassVar[Optional[Type]] = DecodeRosbagsConfig
  description: ClassVar[str] = "Locate ROS2 rosbags and store their paths as raw grains."
  input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (("raw", "rosbag", "path"),)
  output_grain_key: ClassVar[GrainKey] = ("raw", "rosbag", "decoded")


  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase,
    config,
    logger,
  ) -> None:
    # show rosbag path...
    # iterate...
    # get images, lidar scans
    rosbag_path_grain: GrainBase = input_grains[('raw', 'rosbag', 'path')]
    paths = rosbag_path_grain.dict_view(table='path_to_raw')
    for p in paths:
      topic_names_dict = map_topics_and_intrinsics(bag_dir=p['bag_dir'], logger=logger)

      print(topic_names_dict)
      # bag_dir = Path(p['bag_dir'])
      # storage_identifier = p['storage_identifier']
      # bag_data_root = bag_dir / storage_identifier
      # image_dir = bag_data_root / 'images'
      # pointcloud_dir = bag_data_root / 'pointclouds'
      break

