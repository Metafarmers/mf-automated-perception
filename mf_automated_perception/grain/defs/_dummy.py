from typing import ClassVar, Tuple

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class Dummy(GrainBase):
  """
  Dummy grain for testing and sanity checks.
  Not intended for production pipelines.
  """
  key: ClassVar[GrainKey] = ("dummy",)

  SCHEMA_FILES: Tuple[str, ...] = (
    "001_init_v1.sql",
    "301_odometry_v1.sql",
  )