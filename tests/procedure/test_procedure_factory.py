from mf_automated_perception.procedure.core.procedure_base import ProcedureBase
from mf_automated_perception.procedure.core.procedure_factory import ProcedureFactory
from mf_automated_perception.procedure.defs._dummy.definition import (
  Dummy as DummyProcedure,
)


def test_procedure_factory_dummy():
  proc_cls = ProcedureFactory.get("_dummy")
  assert proc_cls is DummyProcedure
  assert issubclass(proc_cls, ProcedureBase)

  proc = proc_cls()
  assert isinstance(proc, ProcedureBase)
