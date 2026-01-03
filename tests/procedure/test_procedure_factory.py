from mf_automated_perception.procedure.core.procedure_base import ProcedureBase
from mf_automated_perception.procedure.core.procedure_factory import ProcedureFactory
from mf_automated_perception.procedure.defs.dummy.implementation import (
  DummyImpl,
)


def test_procedure_factory_dummy():
  ProcedureFactory.build_registry()
  proc_cls = ProcedureFactory.resolve_class("dummy")
  assert proc_cls is DummyImpl
  assert issubclass(proc_cls, ProcedureBase)

  proc = proc_cls()
  assert isinstance(proc, ProcedureBase)
