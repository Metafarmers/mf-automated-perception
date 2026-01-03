from mf_automated_perception.env import LOG_DIR_ROOT
from mf_automated_perception.grain.defs._dummy import Dummy as DummyGrain
from mf_automated_perception.procedure.defs.dummy.implementation import (
  DummyImpl,
)
from mf_automated_perception.utils.gp_logger import get_logger


def test_dummy_procedure_end_to_end(tmp_path):
  """
  End-to-end test for ProcedureBase using Dummy procedure.
  """

  # --------------------------------------------------
  # prepare output grain
  # --------------------------------------------------
  output_grain = DummyGrain()
  output_grain.set_provenance(
    source_procedure="_dummy:1.0.0",
    source_grain_keys=[],
    creator="test_suite",
  )
  output_grain.create()
  # print('# of rows in DummyGrain():', output_grain.count_rows('odometry'))

  # --------------------------------------------------
  # run procedure
  # --------------------------------------------------
  proc = DummyImpl()
  logger = get_logger(
    name="ProcedureDummyTest",
  )
  proc._run(
    input_grains=None,
    output_grain=output_grain,
    config={},  # DummyConfig empty or default
    logger=logger,
  )
  output_grain.close()

  # --------------------------------------------------
  # inspect output grain
  # --------------------------------------------------
  assert output_grain.is_created

  # provenance check
  manifest = output_grain.manifest
  assert manifest is not None
  assert manifest.source_procedure == "_dummy:1.0.0"
  assert manifest.source_grain_keys == []

  # DB content check
  assert "odometry" in output_grain.list_tables()

  rows = output_grain.dict_view(table='odometry')
  print(rows)
  assert len(rows) == 3

  # quick sanity check on first row
  row0 = rows[0]
  assert row0["timestamp_sec"] == 100
  assert row0["px"] == 0.0
  assert row0["py"] == 1.0

  # summary should not raise
  output_grain.summarize_db(max_rows=2)
