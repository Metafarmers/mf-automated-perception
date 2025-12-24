from mflib.perception.automated_perception.grain.grain_base import GrainBase, GrainKey


class RawRosbag(GrainBase):
  """
  Raw rosbag grain.

  Includes:
    - init schema
    - images
    - pointclouds
  """

  key: GrainKey = ("raw", "rosbag")

  SCHEMA_FILES = [
    "001_init.sql",
    "101_images.sql",
    "102_pointclouds.sql",
  ]
