import json
import sqlite3
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import numpy as np


@dataclass
class MfBBox:
  sec: int
  nsec: int
  label: int
  score: float
  tlwh: Tuple[int, int, int, int]  # x, y, w, h

@dataclass
class MfKeypoint(MfBBox):
  keypoint_xy: List[int] = field(default_factory=list)     # [x0,y0,x1,y1,...]
  keypoint_score: List[float] = field(default_factory=list)

@dataclass
class MfSegMaskMemory(MfBBox):
  mask_xy: np.ndarray  # (N,2) polygon in image coordinates


@dataclass
class MfSegMask(MfBBox):
  mask_npz_path: Path | None = None

def add_bbox_to_sql(
  conn: sqlite3.Connection,
  *,
  image_id: int,
  mfbbox: MfBBox,
) -> int:
  """
  Insert one bounding box into yolo_bbox table.
  Returns bbox_id.
  """
  x, y, w, h = mfbbox.tlwh

  cur = conn.execute(
    """
    INSERT INTO yolo_bbox (
      image_id,
      class_id,
      score,
      x,
      y,
      w,
      h
    )
    VALUES (?, ?, ?, ?, ?, ?, ?)
    """,
    (
      image_id,
      mfbbox.label,
      mfbbox.score,
      float(x),
      float(y),
      float(w),
      float(h),
    ),
  )
  return cur.lastrowid

def add_seg_mask_to_sql(
  conn: sqlite3.Connection,
  *,
  bbox_id: int,
  mfmask: MfSegMask,
) -> int:
  """
  Insert one segmentation mask entry.
  Assumes mask is stored as an npz file.
  Returns seg_mask_id.
  """
  if mfmask.mask_npz_path is None:
    raise ValueError("mask_npz_path is None")

  path = Path(mfmask.mask_npz_path)
  if not path.exists():
    raise FileNotFoundError(path)

  cur = conn.execute(
    """
    INSERT INTO yolo_seg_mask (
      bbox_id,
      path_to_npz,
      num_points
    )
    VALUES (?, ?, ?)
    """,
    (
      bbox_id,
      str(path),
      None,
    ),
  )
  return cur.lastrowid


def add_bbox_with_optional_seg_mask(
  conn: sqlite3.Connection,
  *,
  image_id: int,
  mfbbox: MfBBox,
  mfmask: MfSegMask | None = None,
) -> int:
  """
  Insert bbox, and optionally its segmentation mask.
  Returns bbox_id.
  """
  bbox_id = add_bbox_to_sql(
    conn,
    image_id=image_id,
    mfbbox=mfbbox,
  )

  if mfmask is not None:
    add_seg_mask_to_sql(
      conn,
      bbox_id=bbox_id,
      mfmask=mfmask,
    )

  return bbox_id

def add_keypoint_to_sql(
  conn: sqlite3.Connection,
  *,
  bbox_id: int,
  mfkp: MfKeypoint,
  keypoint_format: str,
) -> None:
  """
  Insert keypoints associated with one bbox.
  One-to-one relation: bbox_id is PRIMARY KEY.
  """
  if len(mfkp.keypoint_xy) % 2 != 0:
    raise ValueError("keypoint_xy length must be even")

  xs = mfkp.keypoint_xy[0::2]
  ys = mfkp.keypoint_xy[1::2]

  if mfkp.keypoint_score:
    if len(mfkp.keypoint_score) != len(xs):
      raise ValueError("keypoint_score length mismatch")
    scores = json.dumps(mfkp.keypoint_score)
  else:
    scores = None

  conn.execute(
    """
    INSERT INTO yolo_keypoint (
      bbox_id,
      xs,
      ys,
      scores,
      keypoint_format
    )
    VALUES (?, ?, ?, ?, ?)
    """,
    (
      bbox_id,
      json.dumps(xs),
      json.dumps(ys),
      scores,
      keypoint_format,
    ),
  )
