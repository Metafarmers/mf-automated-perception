from pathlib import Path

from mf_automated_perception.grain.grain_factory import GrainFactory
from mf_automated_perception.env import GRAIN_DATA_ROOT

def test_grain_preview():
  GrainFactory.build_registry()
  loaded_grain = GrainFactory.load_grain(
    grain_data_root=GRAIN_DATA_ROOT,
    key=('raw', 'rosbag', 'decoded'),
    load_rule="latest")

  print('[Loaded Grain]')
  loaded_grain.print_grain_summary()

  print('\n[Schema SQL]')
  schema_sql = loaded_grain.get_schema_sql()
  print(schema_sql)

  print('\n[DB Summary]')
  loaded_grain.summarize_db(max_rows=10)

