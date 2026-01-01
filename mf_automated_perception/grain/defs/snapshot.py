from typing import ClassVar, Tuple

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


class Snapshot(GrainBase):
  """
  Snapshot grain to store sensory data that are temporally aligned.
  """
  key: ClassVar[GrainKey] = ("snapshot",)

  SCHEMA_FILES: Tuple[str, ...] = (
    "001_init_v1.sql",
    "150_snapshot_item_v1.sql",
    "160_snapshot_v1.sql",
    "101_image_v1.sql",
    "102_pointcloud_v1.sql",
    "201_yolo_bbox_v1.sql",
    "202_yolo_keypoint_v1.sql",
    "203_yolo_seg_mask_v1.sql",
    "301_odometry_v1.sql"
  )