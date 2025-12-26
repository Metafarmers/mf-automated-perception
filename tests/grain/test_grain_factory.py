import pytest

from mf_automated_perception.grain.grain_factory import GrainFactory

# ===============================================================
# fixture: isolated grain data root
# ===============================================================

@pytest.fixture
def temp_grain_root(tmp_path, monkeypatch):
  root = tmp_path / "grains"
  root.mkdir()
  monkeypatch.setenv("GRAIN_DATA_ROOT", str(root))
  return root


# ===============================================================
# GrainFactory integration test
# ===============================================================

def test_grain_factory_full_cycle(temp_grain_root):
  """
  Full integration test for GrainFactory.

  Covers:
    1. list grains
    2. show / inspect grain
    3. create + delete for all grains
  """

  # --------------------------------------------------
  # 1. list grains
  # --------------------------------------------------
  keys = GrainFactory.list_keys()

  assert isinstance(keys, tuple)
  assert len(keys) > 0

  for key in keys:
    assert isinstance(key, tuple)
    assert all(isinstance(k, str) for k in key)

  # --------------------------------------------------
  # 2. show / inspect one grain
  # --------------------------------------------------
  key0 = keys[0]
  grain_cls = GrainFactory.resolve_class(key0)

  grain = grain_cls()
  grain.set_provenance(
    source_procedure="grain_factory_test",
    source_grain_keys=[],
    creator="pytest",
  )
  grain.create()
  grain.close()

  # show-like inspection APIs should not crash
  grain.print_grain_summary()

  tables = grain.list_tables()
  assert isinstance(tables, list)

  grain.summarize_db(max_rows=1)

  assert grain.grain_data_dir.exists()
  grain.delete()

  # --------------------------------------------------
  # 3. create + delete for all grains
  # --------------------------------------------------
  for key in keys:
    grain_cls = GrainFactory.resolve_class(key)

    grain = grain_cls()
    grain.set_provenance(
      source_procedure="grain_factory_test_all",
      source_grain_keys=[],
      creator="pytest",
    )

    grain.create()
    assert grain.is_created
    assert grain.grain_data_dir.exists()

    grain.close()
    assert grain.grain_data_dir.exists()
    grain.delete()

