# mf_automated_perception/procedure/procedure_factory.py

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

  # ============================================================
  # registry build (definition only)
  # ============================================================
  @classmethod
  def build_registry(cls) -> None:
    if cls._BUILT:
      return

    base_pkg = "mf_automated_perception.procedure.defs"
    pkg = importlib.import_module(base_pkg)

    errors: list[str] = []

    for modinfo in pkgutil.iter_modules(pkg.__path__):
      key = modinfo.name

      if key.startswith("_") or key == "__pycache__":
        continue

      module_path = f"{base_pkg}.{key}.definition"
      class_name = key_to_class_name(key)

      # -----------------------------
      # import definition
      # -----------------------------
      try:
        module = importlib.import_module(module_path)
      except ModuleNotFoundError as e:
        if e.name == module_path:
          errors.append(
            f"[{key}] missing definition.py "
            f"(expected '{module_path}')"
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

      # -----------------------------
      # class existence
      # -----------------------------
      if not hasattr(module, class_name):
        errors.append(
          f"[{key}] missing class '{class_name}' "
          f"in '{module_path}'"
        )
        continue

      proc_def_cls = getattr(module, class_name)

      # -----------------------------
      # validate definition
      # -----------------------------
      try:
        cls._validate_definition(proc_def_cls, key)
      except Exception as e:
        errors.append(
          f"[{key}] validation failed for "
          f"{proc_def_cls.__module__}.{proc_def_cls.__name__}: {e}"
        )
        continue

      if key in cls._REGISTRY:
        errors.append(
          f"[{key}] duplicate procedure key detected"
        )
        continue

      cls._REGISTRY[key] = proc_def_cls

    if errors:
      msg = (
        "Procedure registry build failed with the following errors:\n"
        + "\n".join(f"  - {e}" for e in errors)
      )
      raise RuntimeError(msg)

    cls._BUILT = True

  # ============================================================
  # resolve implementation (folder convention)
  # ============================================================
  @classmethod
  def resolve_class(cls, key: str) -> Type[ProcedureBase]:
    cls.build_registry()

    if key not in cls._REGISTRY:
      raise KeyError(
        f"Procedure '{key}' not found. "
        f"Available: {sorted(cls._REGISTRY.keys())}"
      )

    proc_def_cls = cls._REGISTRY[key]

    impl_module_path = (
      f"mf_automated_perception.procedure.defs.{key}.implementation"
    )
    impl_class_name = f"{proc_def_cls.__name__}Impl"

    try:
      impl_module = importlib.import_module(impl_module_path)
    except ModuleNotFoundError as e:
      if e.name == impl_module_path:
        raise RuntimeError(
          f"Implementation module missing for procedure '{key}'. "
          f"Expected '{impl_module_path}.py'"
        )
      raise
    except Exception as e:
      raise RuntimeError(
        f"Failed to import implementation module '{impl_module_path}' "
        f"for procedure '{key}': {e}"
      )

    if not hasattr(impl_module, impl_class_name):
      raise RuntimeError(
        f"Implementation class '{impl_class_name}' not found in "
        f"module '{impl_module_path}'"
      )

    impl_cls = getattr(impl_module, impl_class_name)

    if not issubclass(impl_cls, proc_def_cls):
      raise TypeError(
        f"{impl_cls.__module__}.{impl_cls.__name__} "
        f"is not a subclass of {proc_def_cls.__name__}"
      )

    return impl_cls

  # ============================================================
  # utilities
  # ============================================================
  @classmethod
  def list_procedures(cls) -> Dict[str, Type[ProcedureBase]]:
    cls.build_registry()
    return dict(cls._REGISTRY)

  @staticmethod
  def _validate_definition(
    proc_cls: Type[ProcedureBase],
    key: str,
  ) -> None:
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

    # definition은 절대 실행 로직을 가지면 안 됨
    if "_run" in proc_cls.__dict__:
      raise ValueError(
        f"{proc_cls.__name__} defines _run(). "
        f"Definition classes must not implement runtime logic."
      )
