import json
import sqlite3
import pytest

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.grain.defs._dummy import Dummy

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
  assert grain.grain_lib_root.exists()
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

@pytest.mark.integration
@pytest.mark.order(1)
def test_simple():
  # simple test
  class TestGrain(GrainBase):
    key: GrainKey = ("test", "grain")
    SCHEMA_FILES = ["001_init.sql"]

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

@pytest.mark.integration
@pytest.mark.order(1)
def test_dummy_grain():
  grain = Dummy()
  grain.set_provenance(
    source_procedure="dummy_test",
    source_grain_keys=[grain.key],
    creator="dummy_tester",
  )
  grain.create()
  grain.open()
  grain.close()
  grain.print_grain_summary()
