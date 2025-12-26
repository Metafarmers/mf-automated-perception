import importlib
import pkgutil
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Type

from mf_automated_perception.grain.grain_base import (
  GrainBase,
  GrainKey,
)


class GrainFactory:
  """
  Global factory and registry for Grain classes.

  Registry rule example:
    - module name: raw_rosbag_path
    - class name: RawRosbagPath
    - class.key: ("raw", "rosbag", "path")
  """

  _BASE_PKG = "mf_automated_perception.grain.defs"

  _REGISTRY: Dict[GrainKey, Type[GrainBase]] = {}
  _BUILT: bool = False

  # --------------------------------------------------
  # registry
  # --------------------------------------------------
  @classmethod
  def build_registry(cls) -> None:
    if cls._BUILT:
      return

    # build into a temporary dict to keep atomicity
    registry: Dict[GrainKey, Type[GrainBase]] = {}

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

      full_module = f"{cls._BASE_PKG}.{module_name}"
      module = importlib.import_module(full_module)

      expected_key: GrainKey = tuple(module_name.split("_"))
      expected_class_name = "".join(k.capitalize() for k in expected_key)

      if not hasattr(module, expected_class_name):
        raise RuntimeError(
          f"Grain module '{full_module}' must define class "
          f"'{expected_class_name}' "
          f"(derived from module name '{module_name}')"
        )

      grain_cls = getattr(module, expected_class_name)

      if not isinstance(grain_cls, type) or not issubclass(grain_cls, GrainBase):
        raise TypeError(
          f"{expected_class_name} in {full_module} "
          f"is not a subclass of GrainBase"
        )

      class_key = getattr(grain_cls, "key", None)
      if class_key is None:
        raise RuntimeError(
          f"{expected_class_name} must define class-level 'key'"
        )

      if tuple(class_key) != expected_key:
        raise RuntimeError(
          f"Grain key mismatch in {expected_class_name}: "
          f"expected {expected_key} from module name, "
          f"but class defines {class_key}"
        )

      if expected_key in registry:
        raise RuntimeError(f"Duplicate grain key detected: {expected_key}")

      registry[expected_key] = grain_cls

    # commit atomically
    cls._REGISTRY = registry
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
  def _parse_timestamp_dirs(
    cls,
    base_dir: Path,
  ) -> List[tuple[datetime, Path]]:
    candidates: List[tuple[datetime, Path]] = []

    for p in base_dir.iterdir():
      if not p.is_dir():
        continue

      # split on first underscore after timestamp
      name = p.name
      ts_part = name[:19]  # "YYYY-MM-DD_HH-MM-SS"

      try:
        ts = datetime.strptime(ts_part, "%Y-%m-%d_%H-%M-%S")
      except ValueError:
        continue

      candidates.append((ts, p))

    return candidates


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
      raise FileNotFoundError(
        f"No grain directory for key {key}: {key_dir}"
      )

    candidates = cls._parse_timestamp_dirs(key_dir)

    if not candidates:
      raise FileNotFoundError(
        f"No grain instances found for key {key}: {key_dir}"
      )

    candidates.sort(key=lambda x: x[0], reverse=True)
    latest_dir = candidates[0][1]

    return grain_cls.load_from_dir(grain_data_dir=latest_dir)
