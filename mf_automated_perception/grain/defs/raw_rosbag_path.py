from typing import ClassVar

from mflib.perception.automated_perception.grain.grain_base import GrainBase, GrainKey


class RawRosbagPath(GrainBase):
  """
  Raw rosbag grain.

  Includes:
    - init schema
    - images
    - pointclouds
  """

  key: ClassVar[GrainKey] = ("raw", "rosbag", "path")

  SCHEMA_FILES = [
    "001_init.sql",
    "901_path_to_raw_rosbags.sql",
  ]
