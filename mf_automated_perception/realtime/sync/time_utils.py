# time_utils.py
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple
import bisect


# ==================================================
# time utilities
# ==================================================

SEC_IN_NSEC       = 1_000_000_000
MILLISEC_IN_NSEC  = 1_000_000
MICROSEC_IN_NSEC  = 1_000


def to_ns(sec: int, nsec: int) -> int:
  """
  Convert (sec, nsec) to nanoseconds.
  No normalization is done here.
  Caller is responsible for valid input.
  """
  return sec * SEC_IN_NSEC + nsec


# ==================================================
# internal time-ordered entry
# ==================================================

@dataclass(order=True, slots=True)
class SensorEntry:
  """
  Internal time-ordered sensor entry.

  - t_ns is the ONLY time representation
  - ordering is defined solely by t_ns
  """
  t_ns: int
  payload: Any = field(compare=False)


# ==================================================
# sensor buffer
# ==================================================

class SensorBuffer:
  """
  Time-ordered buffer for a single sensor stream.

  - Stores SensorEntry in sorted order
  - Uses bisect on entry.t_ns
  - O(log N) nearest lookup
  - Supports memory purge by slicing
  """

  __slots__ = ("sensor_name", "sensor_type", "_entries")

  def __init__(self, sensor_name: str, sensor_type: str):
    self.sensor_name = sensor_name
    self.sensor_type = sensor_type
    self._entries: List[SensorEntry] = []

  # ----------------------------------------------
  # insertion
  # ----------------------------------------------
  def add(self, *, sec: int = 0, nsec: int, payload: Any) -> None:
    """
    Insert payload with nanosecond timestamp, keeping time order.
    """
    t_ns = to_ns(sec, nsec)
    entry = SensorEntry(t_ns, payload)
    idx = bisect.bisect_right(self._entries, entry)
    self._entries.insert(idx, entry)

  # ----------------------------------------------
  # query
  # ----------------------------------------------
  def find_neighbors(
    self,
    *,
    anchor_sec: int = 0,
    anchor_nsec: int,
  ):
    """
    Find left/right neighboring entries around anchor time.

    Returns:
      (left_entry, right_entry)

    - left_entry  : largest t_ns < anchor_ns (or None)
    - right_entry : smallest t_ns >= anchor_ns (or None)
    """
    if not self._entries:
      return None, None

    anchor_ns = to_ns(anchor_sec, anchor_nsec)
    dummy = SensorEntry(anchor_ns, None)
    idx = bisect.bisect_left(self._entries, dummy)

    # ------------------------
    # boundary cases
    # ------------------------

    if idx == 0 or idx == len(self._entries):
      return None, None

    # ------------------------
    # normal case
    # ------------------------

    left = self._entries[idx - 1]
    right = self._entries[idx]
    return left, right


  # ----------------------------------------------
  # memory management
  # ----------------------------------------------

  def erase_before(self, ts_ns: int) -> None:
    """
    Remove all entries strictly before ts_ns.
    """
    dummy = SensorEntry(ts_ns, None)
    idx = bisect.bisect_left(self._entries, dummy)
    if idx > 0:
      del self._entries[:idx]

  def erase_until(self, ts_ns: int) -> None:
    """
    Remove all entries up to and including ts_ns.
    """
    dummy = SensorEntry(ts_ns, None)
    idx = bisect.bisect_right(self._entries, dummy)
    if idx > 0:
      del self._entries[:idx]

  # ----------------------------------------------
  # diagnostics
  # ----------------------------------------------

  def __len__(self) -> int:
    return len(self._entries)
