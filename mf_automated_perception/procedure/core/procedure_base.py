from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, ClassVar, Dict, Optional, Tuple, Type

import yaml
from pydantic import BaseModel

from mf_automated_perception.env import PROJECT_ROOT
from mf_automated_perception.grain.grain_base import (
  GrainBase,
  GrainKey,
)


class ProcedureBase(BaseModel, ABC):
  """
  Grain -> Grain transformation unit.

  Execution model:
    - File-system based
    - Procedure loads grains by itself
    - Runner only prepares environment and executes
  """

  # ===============================================================
  # procedure metadata (class-level)
  # ===============================================================
  key: ClassVar[str]
  version: ClassVar[str]
  docker_image: ClassVar[str]
  docker_image_tag: ClassVar[str]
  description: ClassVar[str] = ""

  ParamModel: ClassVar[Optional[Type[BaseModel]]]

  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey]

  # default loading rule
  # supported: "latest", None
  load_input_rule: ClassVar[Optional[str]] = None

  @property
  def procedure_id(self) -> str:
    return f"{self.key}:{self.version}"

  model_config = {
    "arbitrary_types_allowed": True,
    "extra": "forbid",
  }

  # ===============================================================
  # execution
  # ===============================================================

  def _check_input_grains(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
  ) -> None:
    cls = self.__class__

    required = set(cls.required_input_grain_keys)
    optional = set(cls.optional_input_grain_keys)
    allowed = required | optional

    # -------------------------------------------------
    # normalize input
    # -------------------------------------------------
    if input_grains is None:
      if required:
        raise RuntimeError(
          f"{cls.key}: input_grains must be provided "
          f"(required_input_grain_keys={cls.required_input_grain_keys})"
        )
      return None

    # -------------------------------------------------
    # validate keys
    # -------------------------------------------------
    for key in input_grains.keys():
      if key not in allowed:
        raise RuntimeError(
          f"{cls.key}: unexpected input grain key {key}, "
          f"allowed_keys={tuple(allowed)}"
        )

    # -------------------------------------------------
    # validate required presence
    # -------------------------------------------------
    for key in required:
      if key not in input_grains:
        raise RuntimeError(
          f"{cls.key}: missing required input grain key {key}"
        )

  def run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config_file: Path,
    context: Dict[str, Any],
    logger,
  ) -> None:
    cls = self.__class__

    # check input grains
    self._check_input_grains(
      input_grains=input_grains,
    )
    if context.get('creator') is None:
      raise ValueError(f"{cls.key}: context['creator'] is required")

    # config validation
    params = None
    if cls.ParamModel is not None:
      if not config_file:
        raise ValueError(f"{cls.key}: config file is required")
      if not config_file.is_absolute():
        config_file = PROJECT_ROOT / config_file
      data = yaml.safe_load(config_file.read_text())
      params = self.ParamModel.model_validate(data)

    # --------------------------------------------------
    # output grain validation
    # --------------------------------------------------
    if output_grain is not None:
      if output_grain.key != cls.output_grain_key:
        raise ValueError(
          f"{cls.key}: output grain key mismatch "
          f"(expected={cls.output_grain_key}, received={output_grain.key})"
        )

      # --------------------------------------------------
      # output grain provenance
      # --------------------------------------------------
      input_grain_keys = [] if not input_grains else list(input_grains.keys())
      output_grain.set_provenance(
        source_procedure=self.procedure_id,
        source_grain_keys=input_grain_keys,
        creator=context['creator'],
      )
      output_grain.create()

    # --------------------------------------------------
    # execute procedure logic
    # --------------------------------------------------
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
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    ...

  def get_param_type(self) -> Optional[Type]:
    return self.__class__.ParamModel
