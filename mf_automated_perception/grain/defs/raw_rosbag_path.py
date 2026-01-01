from typing import ClassVar, Tuple

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class RawRosbagPath(GrainBase):
  """
  Raw rosbag grain.

  Includes:
    - init schema
    - images
    - pointclouds
  """

  key: ClassVar[GrainKey] = ("raw", "rosbag", "path")

  SCHEMA_FILES: Tuple[str, ...] = (
    "001_init_v1.sql",
    "901_path_to_raw_rosbag_v1.sql",
  )