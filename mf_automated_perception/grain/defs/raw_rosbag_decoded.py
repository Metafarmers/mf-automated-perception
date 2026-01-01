from typing import ClassVar, Tuple

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class RawRosbagDecoded(GrainBase):
  """
  Raw rosbag grain.
  """

  key: ClassVar[GrainKey] = ("raw", "rosbag", 'decoded')

  SCHEMA_FILES: Tuple[str, ...] = (
    "001_init_v1.sql",
    "101_image_v1.sql",
    "102_pointcloud_v1.sql",
  )