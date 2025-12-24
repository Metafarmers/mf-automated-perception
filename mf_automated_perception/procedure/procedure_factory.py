# mflib/perception/automated_perception/procedure/procedure_factory.py

import importlib
import pkgutil
from typing import Dict, Type

from mflib.perception.automated_perception.procedure.procedure_base import (
  ProcedureBase,
)

def key_to_class_name(key: str) -> str:
  return "".join(part.capitalize() for part in key.split("_"))

class ProcedureFactory:
  _REGISTRY: Dict[str, Type[ProcedureBase]] = {}
  _BUILT: bool = False

  @classmethod
  def build_registry(cls) -> None:
    if cls._BUILT:
      return

    base_pkg = "mflib.perception.automated_perception.procedure.defs"
    pkg = importlib.import_module(base_pkg)

    for modinfo in pkgutil.iter_modules(pkg.__path__):
      key = modinfo.name
      module_path = f"{base_pkg}.{key}"
      class_name = key_to_class_name(key)

      try:
        module = importlib.import_module(module_path)
        proc_cls = getattr(module, class_name)
        cls._validate(proc_cls, key)
      except Exception:
        continue

      cls._REGISTRY[key] = proc_cls

    cls._BUILT = True

  @classmethod
  def get(cls, key: str) -> Type[ProcedureBase]:
    cls.build_registry()   # ← 핵심
    return cls._REGISTRY[key]

  @classmethod
  def list_procedures(cls) -> Dict[str, Type[ProcedureBase]]:
    cls.build_registry()   # ← 핵심
    return dict(cls._REGISTRY)


  @staticmethod
  def _validate(proc_cls: Type[ProcedureBase], key: str) -> None:
    if not issubclass(proc_cls, ProcedureBase):
      raise TypeError(
        f"{proc_cls.__name__} is not a subclass of ProcedureBase"
      )

    if proc_cls.key != key:
      raise ValueError(
        f"Procedure key mismatch: "
        f"class defines key='{proc_cls.key}', "
        f"but requested key='{key}'"
      )
