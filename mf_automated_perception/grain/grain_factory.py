import importlib
import pkgutil
from typing import Dict, Tuple, Type
from datetime import datetime
from pathlib import Path

from mflib.perception.automated_perception.grain.grain_base import (
  GrainBase,
  GrainKey,
)


class GrainFactory:
  """
  Global factory and registry for Grain classes.
  """

  _BASE_PKG = "mflib.perception.automated_perception.grain.defs"

  _REGISTRY: Dict[GrainKey, Type[GrainBase]] = {}
  _BUILT: bool = False

  # --------------------------------------------------
  # registry
  # --------------------------------------------------
  @classmethod
  def build_registry(cls) -> None:
    if cls._BUILT:
      return

    try:
      pkg = importlib.import_module(cls._BASE_PKG)
    except ImportError as e:
      raise ImportError(
        f"Failed to import grain defs package: {cls._BASE_PKG}"
      ) from e

    if not hasattr(pkg, "__path__"):
      raise RuntimeError(f"{cls._BASE_PKG} is not a package")

    for modinfo in pkgutil.iter_modules(pkg.__path__):
      module_name = modinfo.name

      if module_name.startswith("_"):
        continue

      key: GrainKey = tuple(module_name.split("_"))
      class_name = "".join(k.capitalize() for k in key)
      full_module = f"{cls._BASE_PKG}.{module_name}"

      module = importlib.import_module(full_module)

      if not hasattr(module, class_name):
        raise RuntimeError(
          f"Expected grain class '{class_name}' not found in {full_module}"
        )

      grain_cls = getattr(module, class_name)

      if not issubclass(grain_cls, GrainBase):
        raise TypeError(
          f"{class_name} is not a subclass of GrainBase"
        )

      if key in cls._REGISTRY:
        raise RuntimeError(f"Duplicate grain key detected: {key}")

      cls._REGISTRY[key] = grain_cls

    cls._BUILT = True

  # --------------------------------------------------
  # public API
  # --------------------------------------------------
  @classmethod
  def resolve_class(cls, key: GrainKey) -> Type[GrainBase]:
    cls.build_registry()
    try:
      return cls._REGISTRY[key]
    except KeyError:
      raise KeyError(f"Grain key not registered: {key}") from None

  @classmethod
  def list_keys(cls) -> Tuple[GrainKey, ...]:
    cls.build_registry()
    return tuple(cls._REGISTRY.keys())

  @classmethod
  def load_latest_grain(
    cls,
    *,
    key: GrainKey,
    grain_data_root: Path,
  ) -> GrainBase:
    cls.build_registry()

    grain_cls = cls.resolve_class(key)

    key_dir = grain_data_root
    for k in key:
      key_dir = key_dir / k

    if not key_dir.exists():
      raise FileNotFoundError(f"No grain directory for key {key}: {key_dir}")

    candidates = []
    for p in key_dir.iterdir():
      if not p.is_dir():
        continue

      try:
        ts = datetime.strptime(p.name, "%Y-%m-%d_%H-%M-%S")
      except ValueError:
        continue

      candidates.append((ts, p))

    if not candidates:
      raise FileNotFoundError(f"No grain instances found for key {key}: {key_dir}")

    candidates.sort(key=lambda x: x[0], reverse=True)
    latest_dir = candidates[0][1]

    return grain_cls.load_from_dir(grain_data_dir=latest_dir)
