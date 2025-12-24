from abc import ABC, abstractmethod
from typing import Any, Dict, List, Type, ClassVar, Optional

from pydantic import BaseModel

from mflib.perception.automated_perception.grain.grain_base import GrainBase, GrainKey


class ProcedureBase(BaseModel, ABC):
  """
  Grain -> Grain transformation unit.
  Procedure metadata is declarative and validated by pydantic.
  """

  # ---------- procedure metadata (class-level) ----------
  key: ClassVar[str]
  version: ClassVar[str]
  description: ClassVar[str]
  ParamModel: ClassVar[Optional[Type]] = None
  input_grain_keys: ClassVar[List[GrainKey]] = []
  output_grain_key: ClassVar[GrainKey]

  @property
  def procedure_id(self) -> str:
    return f"{self.key}:{self.version}"

  # ---------- pydantic config ----------
  model_config = {
    "arbitrary_types_allowed": True,
    "extra": "forbid",
  }

  # ===============================================================
  # execution
  # ===============================================================

  def run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase,
    config: Dict[str, Any],
    logger,
  ) -> None:
    cls = self.__class__

    # ---- input grain key validation ----
    expected = set(cls.input_grain_keys)
    received = set(input_grains.keys())
    if expected != received:
      raise ValueError(
        f"{cls.key}: input grain keys mismatch "
        f"(expected={expected}, received={received})"
      )

    # ---- config validation ----
    params = config
    if cls.ParamModel is not None:
      params = cls.ParamModel.model_validate(config)

    # ---- output grain key validation ----
    if output_grain.key != cls.output_grain_key:
      raise ValueError(
        f"{cls.key}: output grain key mismatch "
        f"(expected={cls.output_grain_key}, received={output_grain.key})"
      )

    # ---- output grain initialization check ----
    if not output_grain.is_created:
      raise RuntimeError(
        f"{cls.key}: call create() on output grain before running procedure: {output_grain.key}"
      )

    # ---- execute procedure-specific logic ----
    self._run(
      input_grains=input_grains,
      output_grain=output_grain,
      config=params,
      logger=logger,
    )

  # ===============================================================
  # subclass contract
  # ===============================================================

  @abstractmethod
  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase,
    config,
    logger,
  ) -> None:
    ...
