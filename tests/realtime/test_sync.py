# tests/test_sync_engine_basic.py

from dataclasses import dataclass
from typing import List

import matplotlib.pyplot as plt
import numpy as np

from mf_automated_perception.realtime.sync.engine import SyncEngine
from mf_automated_perception.realtime.sync.time_utils import SensorEntry


@dataclass(slots=True)
class DummyPayload:
  value: float

  @classmethod
  def interpolate(cls, left, right, anchor_ratio_from_left: float):
    v = left.value * (1.0 - anchor_ratio_from_left) + right.value * anchor_ratio_from_left
    return cls(v)


def make_entries(times: List[int]) -> List[SensorEntry]:
  return [SensorEntry(t, DummyPayload(float(t))) for t in times]


def test_sync_engine1():
  s1_times = [100, 120, 150, 200]
  s2_times = [90, 150, 180, 210]

  engine = SyncEngine()
  engine.register_sensor("s1", 'camera', max_dt_ns=20)
  engine.register_sensor("s2", 'camera', max_dt_ns=20) # nouse

  for e in make_entries(s1_times):
    engine.add_sample(sensor_name="s1", sec=0, nsec=e.t_ns, payload=e.payload)
  for e in make_entries(s2_times):
    engine.add_sample(sensor_name="s2", sec=0, nsec=e.t_ns, payload=e.payload)

  results, error = engine.sync_over_pivot_sensor(pivot_sensor_name="s2")
  if results is None:
    print("Sync error:", error)
  else:
    print("Sync results:")
    for t_ns, synced in results.items():
      print(f"  t_ns={t_ns}, ({synced['s1'].t_ns}, {synced['s2'].t_ns})")
  assert set(results.keys()) == {150, 180}


def test_sync_engine2():
  s1_times = [100, 120, 150, 200]
  s2_times = [90, 150, 180, 210]

  engine = SyncEngine()
  engine.register_sensor("s1", 'odometry', max_dt_ns=20)
  engine.register_sensor("s2", 'odometry', max_dt_ns=20) # nouse

  for e in make_entries(s1_times):
    engine.add_sample(sensor_name="s1", sec=0, nsec=e.t_ns, payload=e.payload)
  for e in make_entries(s2_times):
    engine.add_sample(sensor_name="s2", sec=0, nsec=e.t_ns, payload=e.payload)

  results, error = engine.sync_over_pivot_sensor(pivot_sensor_name="s2")
  if results is None:
    print("Sync error:", error)
  else:
    print("Sync results:")
    for t_ns, synced in results.items():
      print(f"  t_ns={t_ns}, ({synced['s1'].t_ns}, {synced['s2'].t_ns})")
  assert set(results.keys()) == {150, 180}



def test_sync_engine_20hz_30hz_50hz_plot():
  # --------------------------------------------
  # 1. generate perfect timestamps
  # --------------------------------------------
  n = 50

  hz20 = int(1e9 / 20)  # 50_000_000 ns
  hz30 = int(1e9 / 30)  # 33_333_333 ns
  hz50 = int(1e9 / 50)  # 20_000_000 ns

  s20_times = np.arange(n//5) * hz20
  s30_times = np.arange(n//3) * hz30
  s50_times = np.arange(n//2) * hz50

  # --------------------------------------------
  # 2. build engine
  # --------------------------------------------
  engine = SyncEngine()
  engine.register_sensor("s20", "camera", max_dt_ns=hz20 // 5)
  engine.register_sensor("s30", "camera", max_dt_ns=hz30 // 2)
  engine.register_sensor("s50", "camera", max_dt_ns=hz50 // 2)

  for t in s20_times:
    engine.add_sample(sensor_name="s20", sec=0, nsec=int(t), payload=t)
  for t in s30_times:
    engine.add_sample(sensor_name="s30", sec=0, nsec=int(t), payload=t)
  for t in s50_times:
    engine.add_sample(sensor_name="s50", sec=0, nsec=int(t), payload=t)

  # pivot = fastest sensor
  results, error = engine.sync_over_pivot_sensor("s50")
  assert error is None
  assert len(results) > 0

  # --------------------------------------------
  # 3. extract sync results
  # --------------------------------------------
  anchor_times = np.array(sorted(results.keys()))

  s20_synced = np.array([results[t]["s20"].t_ns for t in anchor_times])
  s30_synced = np.array([results[t]["s30"].t_ns for t in anchor_times])
  s50_synced = np.array([results[t]["s50"].t_ns for t in anchor_times])

  # --------------------------------------------
  # 4. plot: intuitive sync visualization
  # --------------------------------------------
  plt.figure(figsize=(12, 5))

  # raw streams
  plt.scatter(s20_times, np.zeros_like(s20_times), alpha=0.25, label="s20 raw (20Hz)")
  plt.scatter(s30_times, np.ones_like(s30_times), alpha=0.25, label="s30 raw (30Hz)")
  plt.scatter(s50_times, np.ones_like(s50_times) * 2, alpha=0.25, label="s50 raw (50Hz)")

  # sync connections
  for t in anchor_times:
    plt.plot(
      [results[t]["s20"].t_ns, results[t]["s50"].t_ns],
      [0, 2],
      "k-",
      alpha=0.2,
    )
    plt.plot(
      [results[t]["s30"].t_ns, results[t]["s50"].t_ns],
      [1, 2],
      "k-",
      alpha=0.2,
    )

  # synced points
  plt.scatter(s20_synced, np.zeros_like(s20_synced), s=80, marker="x", label="s20 synced")
  plt.scatter(s30_synced, np.ones_like(s30_synced), s=80, marker="x", label="s30 synced")
  plt.scatter(s50_synced, np.ones_like(s50_synced) * 2, s=80, marker="x", label="s50 synced")

  plt.yticks([0, 1, 2], ["s20 (20Hz)", "s30 (30Hz)", "s50 (50Hz)"])
  plt.xlabel("time [ns]")
  plt.title("SyncEngine test: 20Hz / 30Hz / 50Hz (pivot = 50Hz)")
  plt.legend()
  plt.grid(True, axis="x", alpha=0.2)
  plt.tight_layout()
  plt.savefig("/workspace/tests/realtime/test_sync_engine_20hz_30hz_50hz_plot.png")

def test_sync_engine_20hz_30hz_50hz_plot_with_interpolation():
  # --------------------------------------------
  # 1. generate perfect timestamps
  # --------------------------------------------
  n = 50

  hz20 = int(1e9 / 20)  # 50_000_000 ns
  hz30 = int(1e9 / 30)  # 33_333_333 ns
  hz50 = int(1e9 / 50)  # 20_000_000 ns

  s20_times = np.arange(n // 5) * hz20   # sparse
  s30_times = np.arange(n // 3) * hz30
  s50_times = np.arange(n // 2) * hz50

  # --------------------------------------------
  # 2. build engine
  # --------------------------------------------
  engine = SyncEngine()
  engine.register_sensor("s20", "odometry", max_dt_ns=hz20 // 2)  # interpolation ON
  engine.register_sensor("s30", "camera",   max_dt_ns=hz30 // 2)
  engine.register_sensor("s50", "camera",   max_dt_ns=hz50 // 2)

  for t in s20_times:
    engine.add_sample(sensor_name="s20", sec=0, nsec=int(t), payload=DummyPayload(float(t)))
  for t in s30_times:
    engine.add_sample(sensor_name="s30", sec=0, nsec=int(t), payload=DummyPayload(float(t)))
  for t in s50_times:
    engine.add_sample(sensor_name="s50", sec=0, nsec=int(t), payload=DummyPayload(float(t)))

  # pivot = fastest sensor
  results, error = engine.sync_over_pivot_sensor("s50")
  assert error is None
  assert len(results) > 0

  # --------------------------------------------
  # 3. extract sync results
  # --------------------------------------------
  anchor_times = np.array(sorted(results.keys()))

  s20_synced = np.array([results[t]["s20"].t_ns for t in anchor_times])
  s30_synced = np.array([results[t]["s30"].t_ns for t in anchor_times])
  s50_synced = np.array([results[t]["s50"].t_ns for t in anchor_times])

  # --------------------------------------------
  # 4. plot: interpolation-aware visualization
  # --------------------------------------------
  plt.figure(figsize=(13, 5))

  # raw streams
  plt.scatter(
    s20_times,
    np.zeros_like(s20_times),
    alpha=0.25,
    label="s20 raw (20Hz, odometry)",
  )
  plt.scatter(
    s30_times,
    np.ones_like(s30_times),
    alpha=0.25,
    label="s30 raw (30Hz)",
  )
  plt.scatter(
    s50_times,
    np.ones_like(s50_times) * 2,
    alpha=0.25,
    label="s50 raw (50Hz, pivot)",
  )

  # sync connections
  for t in anchor_times:
    # s20 is interpolated → dashed line
    plt.plot(
      [results[t]["s20"].t_ns, results[t]["s50"].t_ns],
      [0, 2],
      linestyle="--",
      color="black",
      alpha=0.25,
    )
    # s30 is nearest → solid line
    plt.plot(
      [results[t]["s30"].t_ns, results[t]["s50"].t_ns],
      [1, 2],
      linestyle="-",
      color="black",
      alpha=0.25,
    )

  # synced points
  plt.scatter(
    s20_synced,
    np.zeros_like(s20_synced),
    s=90,
    marker="D",
    label="s20 synced (interpolated)",
  )
  plt.scatter(
    s30_synced,
    np.ones_like(s30_synced),
    s=80,
    marker="x",
    label="s30 synced",
  )
  plt.scatter(
    s50_synced,
    np.ones_like(s50_synced) * 2,
    s=80,
    marker="x",
    label="s50 synced (anchor)",
  )

  plt.yticks([0, 1, 2], ["s20 (20Hz, interp)", "s30 (30Hz)", "s50 (50Hz)"])
  plt.xlabel("time [ns]")
  plt.title("SyncEngine test: 20Hz interpolated (odometry) / 30Hz / 50Hz")
  plt.legend()
  plt.grid(True, axis="x", alpha=0.2)
  plt.tight_layout()
  plt.savefig(
    "/workspace/tests/realtime/test_sync_engine_20hz_30hz_50hz_plot_interp.png"
  )
