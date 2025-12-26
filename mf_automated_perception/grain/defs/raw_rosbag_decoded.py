from typing import ClassVar

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class RawRosbagDecoded(GrainBase):
  """
  Raw rosbag grain.
  """

  key: ClassVar[GrainKey] = ("raw", "rosbag", 'decoded')

  SCHEMA_FILES = [
    "001_init_v1.sql",
    "101_images_v1.sql",
    "102_pointclouds_v1.sql",
  ]
