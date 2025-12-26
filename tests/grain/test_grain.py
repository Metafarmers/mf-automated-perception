import json
import sqlite3
from typing import ClassVar

from mf_automated_perception.grain.defs._dummy import Dummy
from mf_automated_perception.grain.grain_base import GrainBase, GrainKey


def test_grain_base_lifecycle_real_environment():
  """
  Real-run lifecycle test.

  Assumes:
  - grain package is already installed
  - grain/schema directory exists
  - Dummy grain is available under grain/defs/_dummy.py
  """

  # ===============================================================
  # create grain (official lifecycle)
  # ===============================================================

  grain = Dummy()
  grain.set_provenance(
    source_procedure="test_procedure",
    source_grain_keys=[grain.key],
    creator="test_user",
  )
  grain.create()

  # ===============================================================
  # check filesystem artifacts (real paths)
  # ===============================================================

  assert grain.grain_data_root.exists()
  assert grain.db_path.exists()
  assert grain.manifest_path.exists()

  # ===============================================================
  # check manifest content
  # ===============================================================

  manifest = json.loads(grain.manifest_path.read_text())

  assert manifest["key"] == list(grain.key)
  assert manifest["source_procedure"] == "test_procedure"
  assert manifest["creator"] == "test_user"

  # ===============================================================
  # check schema application (real schema)
  # ===============================================================

  conn = sqlite3.connect(grain.db_path)

  table = conn.execute(
    "SELECT name FROM sqlite_master "
    "WHERE type='table' AND name='odometry';"
  ).fetchone()

  index = conn.execute(
    "SELECT name FROM sqlite_master "
    "WHERE type='index' AND name='idx_odometry_timestamp';"
  ).fetchone()

  conn.close()

  assert table is not None
  assert index is not None

  # ===============================================================
  # check open / close semantics
  # ===============================================================

  conn = grain.open()
  assert conn is not None
  grain.close()
  assert grain._conn is None

def test_simple():
  # simple test
  class TestGrain(GrainBase):
    key: ClassVar[GrainKey] = ("test", "grain") # type: ignore
    SCHEMA_FILES = ["001_init_v1.sql"]

  grain = TestGrain()
  grain.set_provenance(
    source_procedure="simple_test",
    source_grain_keys=[],
    creator="simple_tester",
  )
  grain.create()
  grain.open()
  grain.close()
  grain.print_grain_summary()
  assert grain.grain_data_dir.exists() # it can not be called as it is deleted
  grain.delete()

def test_dummy_grain():
  grain = Dummy()
  grain.set_provenance(
    source_procedure="dummy_test",
    source_grain_keys=[],
    creator="dummy_tester",
  )
  grain.create()
  grain.open()
  grain.close()
  grain.print_grain_summary()
  assert grain.grain_data_dir.exists()
  grain.delete()

def test_dummy_grain_with_odometry_db():
  grain = Dummy()

  grain.set_provenance(
    source_procedure="dummy_test",
    source_grain_keys=[],
    creator="dummy_tester",
  )
  grain.create()

  # schema는 이미 적용됨
  conn = grain.open()

  for i in range(3):
    conn.execute(
      """
      INSERT INTO odometry (
        timestamp_sec, timestamp_nsec,
        px, py, pz,
        qx, qy, qz, qw,
        vx, vy, vz,
        wx, wy, wz
      )
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
      """,
      (
        100 + i,
        i * 100_000_000,
        float(i),
        float(i + 1),
        float(i + 2),
        0.0, 0.0, 0.0, 1.0,
        0.1 * i, 0.2 * i, 0.3 * i,
        0.01 * i, 0.02 * i, 0.03 * i,
      ),
    )

  conn.commit()
  grain.close()

  # inspection
  assert "odometry" in grain.list_tables()

  rows = grain.count_rows('odometry')
  assert rows == 3

  grain.summarize_db(max_rows=2)
