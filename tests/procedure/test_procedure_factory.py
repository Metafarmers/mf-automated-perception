from mflib.perception.automated_perception.procedure.procedure_factory import (
  ProcedureFactory,
)
from mflib.perception.automated_perception.procedure.procedure_base import (
  ProcedureBase,
)

def test_build_registry():
  ProcedureFactory.build_registry()

  registry = ProcedureFactory.list_procedures()
  print(registry)
  assert isinstance(registry, dict)
  assert len(registry) > 0

  for key, proc_cls in registry.items():
    assert issubclass(proc_cls, ProcedureBase)
    assert proc_cls.key == key


def test_get_procedure():
  ProcedureFactory.build_registry()
  registry = ProcedureFactory.list_procedures()

  key = next(iter(registry.keys()))
  proc_cls = ProcedureFactory.get(key)

  assert proc_cls is registry[key]
