from pathlib import Path

from mf_automated_perception.grain.defs._dummy import Dummy
from mf_automated_perception.grain.grain_base import GrainBase, GrainKey

def test_dummy_grain_create_load_and_schema_roundtrip(tmp_path):
  """
  End-to-end test for:
    - grain creation
    - schema application & persistence
    - DB insert
    - reload from disk
    - schema inspection
  """

  # --------------------------------------------------
  # 1. create grain
  # --------------------------------------------------
  grain = Dummy()

  grain.set_provenance(
    source_procedure="dummy_test",
    source_grain_keys=[],
    creator="dummy_tester",
  )

  grain.create()

  assert grain.is_created
  assert grain.grain_data_dir_abs.exists()
  assert grain.db_path_abs.exists()
  assert grain.manifest_path_abs.exists()

  # --------------------------------------------------
  # 2. schema should already be applied
  # --------------------------------------------------
  tables = grain.list_tables()
  assert "odometry" in tables

  # --------------------------------------------------
  # 3. insert dummy odometry data
  # --------------------------------------------------
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

  assert grain.count_rows("odometry") == 3


  # --------------------------------------------------
  # 6. reload grain from disk
  # --------------------------------------------------
  loaded_grain = Dummy.load_from_dir(
    grain_data_dir=grain.grain_data_dir_abs
  )

  # --------------------------------------------------
  # 4. schema inspection (stored schema, not global)
  # --------------------------------------------------
  schema_sql = loaded_grain.get_schema_sql()

  assert isinstance(schema_sql, str)
  assert "CREATE TABLE" in schema_sql
  assert "odometry" in schema_sql

  print("\n[Stored Schema after load]")
  print(schema_sql)

  # --------------------------------------------------
  # 5. print DB summary (human inspection)
  # --------------------------------------------------
  loaded_grain.summarize_db(max_rows=2)

  assert loaded_grain.is_created
  assert loaded_grain.grain_id == grain.grain_id
  assert loaded_grain.list_tables() == grain.list_tables()
  assert loaded_grain.count_rows("odometry") == 3
  # schema should still be accessible after load
  loaded_schema_sql =   loaded_grain.get_schema_sql()
  assert loaded_schema_sql == schema_sql

  print("\n[Loaded Grain Summary]")
  loaded_grain.print_grain_summary()
  # --------------------------------------------------
  # 7. cleanup
  # --------------------------------------------------
  loaded_grain.delete()
  assert not grain.grain_data_dir_abs.exists()
