from typing import ClassVar, Tuple

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class Snapshot(GrainBase):
  """
  Snapshot grain to store sensory data that are temporally aligned.
  """
  key: ClassVar[GrainKey] = ("snapshot",)

  SCHEMA_FILES: Tuple[str, ...] = (
    "001_init_v1.sql",
    "150_snapshot_item.sql",
    "160_snapshot_header.sql",
  )