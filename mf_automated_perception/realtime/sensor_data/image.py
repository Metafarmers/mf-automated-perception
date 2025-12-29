from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field, field_validator
import json


class MfImage(BaseModel):
  # ---------- identity ----------
  image_id: Optional[int] = None

  # ---------- timestamp ----------
  timestamp_sec: int
  timestamp_nsec: int
  sensor_name: str

  # ---------- image metadata ----------
  path_to_raw: str
  width: int
  height: int
  encoding: str

  # ---------- camera model ----------
  K: Optional[List[float]] = None   # length 9 expected
  D: Optional[List[float]] = None   # variable length

  # ---------- validators ----------
  @field_validator("K")
  @classmethod
  def validate_K(cls, v):
    if v is None:
      return v
    if len(v) != 9:
      raise ValueError("K must be a flattened 3x3 matrix (length 9)")
    return v

  # ---------- helpers ----------
  @classmethod
  def from_dict(cls, d: Dict[str, Any]) -> "MfImage":
    return cls(**d)

  def to_db_dict(self) -> Dict[str, Any]:
    """
    Convert to dict suitable for SQLite insertion.
    Lists are JSON-encoded.
    """
    return {
      "image_id": self.image_id,
      "timestamp_sec": self.timestamp_sec,
      "timestamp_nsec": self.timestamp_nsec,
      "sensor_name": self.sensor_name,
      "path_to_raw": self.path_to_raw,
      "width": self.width,
      "height": self.height,
      "encoding": self.encoding,
      "K": json.dumps(self.K) if self.K is not None else None,
      "D": json.dumps(self.D) if self.D is not None else None,
    }
