# tests/host/test_load_grain.py
import pytest
from pathlib import Path
from mf_automated_perception.grain.grain_factory import GrainFactory
from mf_automated_perception.env import GRAIN_DATA_ROOT


def test_load_latest_raw_rosbag_path():
  """Test loading latest raw/rosbag/path grain."""
  grain = GrainFactory.load_grain(
    key=("raw", "rosbag", "path"),
    grain_data_root=GRAIN_DATA_ROOT,
    load_rule="latest"
  )

  assert grain is not None
  assert grain.key == ("raw", "rosbag", "path")
  assert grain.uuid is not None

  # Verify grain has data
  rows = list(grain.dict_view(table="path_to_raw", limit=5))
  assert len(rows) > 0
  assert "bag_dir" in rows[0]


def test_load_raw_rosbag_path_by_uuid():
  """Test loading specific raw/rosbag/path grain by UUID."""
  # First get latest to find a valid UUID
  latest_grain = GrainFactory.load_grain(
    key=("raw", "rosbag", "path"),
    grain_data_root=GRAIN_DATA_ROOT,
    load_rule="latest"
  )

  assert latest_grain is not None
  target_uuid = latest_grain.uuid

  # Load by UUID
  grain = GrainFactory.load_grain(
    key=("raw", "rosbag", "path"),
    grain_data_root=GRAIN_DATA_ROOT,
    load_rule="uuid",
    uuid=target_uuid
  )

  assert grain is not None
  assert grain.uuid == target_uuid
  assert grain.grain_data_dir_abs == latest_grain.grain_data_dir_abs


def test_load_nonexistent_uuid_returns_none():
  """Test loading non-existent UUID returns None."""
  grain = GrainFactory.load_grain(
    key=("raw", "rosbag", "path"),
    grain_data_root=GRAIN_DATA_ROOT,
    load_rule="uuid",
    uuid="nonexistent-uuid-12345"
  )

  assert grain is None
