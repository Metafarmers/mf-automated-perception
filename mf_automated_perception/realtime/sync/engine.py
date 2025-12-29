# sync_engine.py

from typing import Any, Dict, Optional
from dataclasses import dataclass

from mf_automated_perception.realtime.sync.time_utils import (
  SensorBuffer,
  SensorEntry,
  to_ns,
)

# ==================================================
# interpolation
# ==================================================

def try_interpolation(
  left: SensorEntry,
  right: SensorEntry,
  anchor_ns: int,
) -> SensorEntry:
  """
  Delegate interpolation to payload class.

  Payload class must implement:
    interpolate(left, right, anchor_ns)
  """
  if left is None or right is None:
    raise ValueError("both left and right entries are required")

  if not (left.t_ns <= anchor_ns <= right.t_ns):
    raise ValueError("anchor_ns must be between left and right")

  if left.t_ns == right.t_ns:
    return left

  ratio = (anchor_ns - left.t_ns) / (right.t_ns - left.t_ns)
  payload = left.payload.__class__.interpolate(
    left=left.payload,
    right=right.payload,
    anchor_ratio_from_left=ratio,
  )
  return SensorEntry(anchor_ns, payload)


# ==================================================
# selection policy + erase boundary
# ==================================================

@dataclass(slots=True)
class SyncDecision:
  chosen: SensorEntry
  erase_before_ns: int


def select_entry_with_left_boundary(
  *,
  anchor_ns: int,
  left: Optional[SensorEntry],
  right: Optional[SensorEntry],
  sensor_type: str,
) -> Optional[SyncDecision]:
  """
  Decide which entry to use and how far buffer can be safely advanced.
  """

  if left is None or right is None: # prevent calling with any None
    return None

  sensor_type = sensor_type.lower()

  # -----------------------------
  # non-interpolatable sensors
  # -----------------------------
  if sensor_type in {"camera", "lidar"}:
    dist_to_left = abs(left.t_ns - anchor_ns)
    dist_to_right = abs(right.t_ns - anchor_ns)
    chosen = left if dist_to_left <= dist_to_right else right

    return SyncDecision(
      chosen=chosen,
      erase_before_ns=chosen.t_ns,
    )

  # -----------------------------
  # interpolatable sensors
  # -----------------------------
  if sensor_type in {"odometry"}:
    # interpolation case
    chosen = try_interpolation(left, right, anchor_ns)

    # IMPORTANT:
    # left is still needed for future interpolations
    return SyncDecision(
      chosen=chosen,
      erase_before_ns=left.t_ns,
    )
  raise ValueError(f"unimplemented sensor_type: {sensor_type}")


# ==================================================
# sync engine
# ==================================================

class SyncEngine:
  """
  Pivot-sensor-based synchronization engine.
  """

  def __init__(self):
    self._buffers: Dict[str, SensorBuffer] = {}
    self._sensor_types: Dict[str, str] = {}
    self._max_dt_ns: Dict[str, int] = {}

  # ----------------------------------------------
  # registration
  # ----------------------------------------------

  def register_sensor(
    self,
    sensor_name: str,
    sensor_type: str,
    max_dt_ns: int,
  ) -> None:
    if sensor_name in self._buffers:
      raise ValueError(f"sensor already registered: {sensor_name}")

    self._buffers[sensor_name] = SensorBuffer(sensor_name, sensor_type)
    self._sensor_types[sensor_name] = sensor_type
    self._max_dt_ns[sensor_name] = max_dt_ns

  # ----------------------------------------------
  # ingestion
  # ----------------------------------------------

  def add_sample(
    self,
    *,
    sensor_name: str,
    sec: int,
    nsec: int,
    payload: Any,
  ) -> None:
    if sensor_name not in self._buffers:
      raise KeyError(f"unknown sensor: {sensor_name}")

    self._buffers[sensor_name].add(sec=sec, nsec=nsec, payload=payload)

  # ----------------------------------------------
  # sync at a single anchor (NO mutation)
  # ----------------------------------------------

  def sync_at(
    self,
    *,
    anchor_ns: int,
  ) -> tuple[Optional[Dict[str, SyncDecision]], Optional[str]]:
    """
    Sync all sensors at anchor time (NO mutation).
    """
    decisions: Dict[str, SyncDecision] = {}

    for sensor_name, buffer in self._buffers.items():
      left, right = buffer.find_neighbors(anchor_nsec=anchor_ns)

      decision = select_entry_with_left_boundary(
        anchor_ns=anchor_ns,
        left=left,
        right=right,
        sensor_type=self._sensor_types[sensor_name],
      )

      if decision is None:
        return None, f"no data around anchor for sensor: {sensor_name}"

      if abs(decision.chosen.t_ns - anchor_ns) > self._max_dt_ns[sensor_name]:
        return None, f"dt exceeds tolerance for sensor: {sensor_name}"

      decisions[sensor_name] = decision

    return decisions, None


  # ----------------------------------------------
  # pivot-sensor-driven sync + advance
  # ----------------------------------------------

  def sync_over_pivot_sensor(
    self,
    pivot_sensor_name: str,
  ) -> tuple[Dict[int, Dict[str, SensorEntry]], Optional[str]]:
    """
    Sync using anchor times derived from a pivot sensor,
    constrained to the temporal intersection of all sensors.
    """
    def _compute_intersection_ns(self) -> tuple[int, int]:
      overlap_start: Optional[int] = None
      overlap_end: Optional[int] = None

      for buffer in self._buffers.values():
        if len(buffer) == 0:
          raise RuntimeError(f'no data for sensor: {buffer.sensor_name}')

        first_t = buffer._entries[0].t_ns
        last_t = buffer._entries[-1].t_ns

        if overlap_start is None or first_t > overlap_start:
          overlap_start = first_t

        if overlap_end is None or last_t < overlap_end:
          overlap_end = last_t

      if overlap_start is None or overlap_end is None or overlap_start > overlap_end:
        raise RuntimeError('no temporal overlap across sensors')

      return overlap_start, overlap_end

    if pivot_sensor_name not in self._buffers:
      raise KeyError(f"unknown pivot sensor: {pivot_sensor_name}")

    if len(self._buffers) == 0:
      return {}, 'no sensors registered'

    try:
      overlap_start, overlap_end = _compute_intersection_ns(self)
    except RuntimeError as e:
      return {}, str(e)

    pivot_buffer = self._buffers[pivot_sensor_name]

    # -------------------------------------------------
    # build anchor time queue (offline style)
    # -------------------------------------------------
    anchor_time_queue = [
      entry.t_ns
      for entry in pivot_buffer._entries
      if overlap_start <= entry.t_ns <= overlap_end
    ]

    results: Dict[int, Dict[str, SensorEntry]] = {}
    latest_error: Optional[str] = None

    for anchor_ns in anchor_time_queue:
      decisions, error = self.sync_at(anchor_ns=anchor_ns)
      print(f"anchor_ns={anchor_ns}, decisions={decisions}, error={error}")

      if decisions is None:
        latest_error = error
        continue

      # advance buffers
      for sensor_name, decision in decisions.items():
        self._buffers[sensor_name].erase_before(decision.erase_before_ns)

      results[anchor_ns] = {
        name: decision.chosen
        for name, decision in decisions.items()
      }

    if not results:
      return {}, (
        f'failed sync with pivot sensor {pivot_sensor_name}. '
        f'Latest error: {latest_error}'
      )

    return results, None
