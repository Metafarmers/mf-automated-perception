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
