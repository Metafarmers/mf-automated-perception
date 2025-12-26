from typing import ClassVar

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class Dummy(GrainBase):
  """
  Dummy grain for testing and sanity checks.
  Not intended for production pipelines.
  """
  key: ClassVar[GrainKey] = ("dummy",)

  SCHEMA_FILES = [
    "001_init_v1.sql",
    "301_odometry_v1.sql",
  ]
