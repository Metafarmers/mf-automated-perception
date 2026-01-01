from typing import ClassVar, Tuple

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class Slam3dGlim(GrainBase):
  key: ClassVar[GrainKey] = ('slam3d', 'glim')

  SCHEMA_FILES: Tuple[str, ...] = (
    "001_init_v1.sql",
    "130_ply_v1.sql",
    "301_odometry_v1.sql"
  )