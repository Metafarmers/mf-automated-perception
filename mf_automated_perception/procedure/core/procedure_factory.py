# mflib/perception/automated_perception/procedure/procedure_factory.py

import importlib
import pkgutil
from typing import Dict, Type

from mf_automated_perception.procedure.core.procedure_base import ProcedureBase


def key_to_class_name(key: str) -> str:
  # "locate_rosbags" -> "LocateRosbags"
  return "".join(part.capitalize() for part in key.split("_"))


class ProcedureFactory:
  _REGISTRY: Dict[str, Type[ProcedureBase]] = {}
  _BUILT: bool = False

  @classmethod
  def build_registry(cls) -> None:
    if cls._BUILT:
      return

    base_pkg = "mf_automated_perception.procedure.defs"
    pkg = importlib.import_module(base_pkg)

    errors: list[str] = []

    for modinfo in pkgutil.iter_modules(pkg.__path__):
      key = modinfo.name

      # ------------------------------------------------------------
      # allow __pycache__
      # ------------------------------------------------------------
      if key == "__pycache__":
        continue

      module_path = f"{base_pkg}.{key}.definition"
      class_name = key_to_class_name(key)

      # ------------------------------------------------------------
      # import definition
      # ------------------------------------------------------------
      try:
        module = importlib.import_module(module_path)
      except ModuleNotFoundError as e:
        # definition.py 자체가 없는 경우 vs 내부 import 에러 구분
        if e.name == module_path:
          errors.append(
            f"[{key}] missing definition.py "
            f"(expected module '{module_path}')"
          )
        else:
          errors.append(
            f"[{key}] import failed due to missing dependency: "
            f"{e.name} (while importing '{module_path}')"
          )
        continue
      except Exception as e:
        errors.append(
          f"[{key}] failed to import '{module_path}': "
          f"{type(e).__name__}: {e}"
        )
        continue

      # ------------------------------------------------------------
      # class existence
      # ------------------------------------------------------------
      if not hasattr(module, class_name):
        errors.append(
          f"[{key}] missing class '{class_name}' "
          f"in '{module_path}'"
        )
        continue

      proc_cls = getattr(module, class_name)

      # ------------------------------------------------------------
      # validate Procedure class
      # ------------------------------------------------------------
      try:
        cls._validate(proc_cls, key)
      except Exception as e:
        errors.append(
          f"[{key}] validation failed for "
          f"{proc_cls.__module__}.{proc_cls.__name__}: {e}"
        )
        continue

      # ------------------------------------------------------------
      # duplicate key
      # ------------------------------------------------------------
      if key in cls._REGISTRY:
        errors.append(
          f"[{key}] duplicate procedure key detected: "
          f"{cls._REGISTRY[key]} vs {proc_cls}"
        )
        continue

      cls._REGISTRY[key] = proc_cls

    # --------------------------------------------------------------
    # final decision
    # --------------------------------------------------------------
    if errors:
      msg = (
        "Procedure registry build failed with the following errors:\n"
        + "\n".join(f"  - {e}" for e in errors)
      )
      raise RuntimeError(msg)

    cls._BUILT = True



  @classmethod
  def resolve_class(cls, key: str) -> Type[ProcedureBase]:
    cls.build_registry()

    if key not in cls._REGISTRY:
      raise KeyError(
        f"Procedure '{key}' not found. "
        f"Available: {sorted(cls._REGISTRY.keys())}"
      )

    return cls._REGISTRY[key]

  @classmethod
  def list_procedures(cls) -> Dict[str, Type[ProcedureBase]]:
    cls.build_registry()
    return dict(cls._REGISTRY)

  @staticmethod
  def _validate(proc_cls: Type[ProcedureBase], key: str) -> None:
    if not issubclass(proc_cls, ProcedureBase):
      raise TypeError(
        f"{proc_cls.__module__}.{proc_cls.__name__} "
        f"is not a subclass of ProcedureBase"
      )

    if proc_cls.key != key:
      raise ValueError(
        f"Procedure key mismatch: "
        f"class defines key='{proc_cls.key}', "
        f"but directory name is '{key}'"
      )
