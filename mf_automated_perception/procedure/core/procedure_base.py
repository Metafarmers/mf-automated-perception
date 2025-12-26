from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, ClassVar, Dict, Optional, Tuple, Type

import yaml
from pydantic import BaseModel

from mf_automated_perception.env import GRAIN_DATA_ROOT
from mf_automated_perception.grain.grain_base import (
  GrainBase,
  GrainKey,
)
from mf_automated_perception.grain.grain_factory import (
  GrainFactory,
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
  description: ClassVar[str] = ""

  ParamModel: ClassVar[Optional[Type]] = None

  input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
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

  def _resolve_inputs(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    load_input_rule: Optional[str],
  ) -> Dict[GrainKey, GrainBase]:
    cls = self.__class__

    rule = load_input_rule if load_input_rule is not None else cls.load_input_rule
    requires_inputs = bool(cls.input_grain_keys)

    resolved: Dict[GrainKey, GrainBase] = {}

    if rule == "latest":
      if requires_inputs:
        if input_grains is not None:
          raise RuntimeError(
            f"{cls.key}: input_grains must not be provided "
            f"when load_input_rule='latest' "
            f"(input_grain_keys={cls.input_grain_keys})"
          )

        grain_root = Path(GRAIN_DATA_ROOT)

        for key in cls.input_grain_keys:
          resolved[key] = GrainFactory.load_latest_grain(
            key=key,
            grain_data_root=grain_root,
          )

      return resolved

    # ------------------------------------------
    # explicit input mode
    # ------------------------------------------
    if requires_inputs:
      print('hi!')
      if input_grains is None:
        raise RuntimeError(
          f"{cls.key}: input_grains must be provided explicitly "
          f"when load_input_rule is None "
          f"(input_grain_keys={cls.input_grain_keys})"
        )

      return input_grains

    # no inputs required
    print()
    return {}


  def run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]] = None,
    output_grain: GrainBase,
    config_file: Path,
    context: Dict[str, Any],
    logger,
    load_input_rule: Optional[str] = None,
  ) -> None:
    cls = self.__class__

    # --------------------------------------------------
    # resolve loading rule
    # --------------------------------------------------
    rule = load_input_rule if load_input_rule is not None else cls.load_input_rule

    if input_grains is not None and len(input_grains) == 0: 
      input_grains = None

    if rule not in (None, "latest"):
      raise ValueError(f"{cls.key}: unsupported load_input_rule={rule}")

    resolved_inputs = self._resolve_inputs(
      input_grains=input_grains,
      load_input_rule=load_input_rule,
    )

    # --------------------------------------------------
    # input grain contract validation
    # --------------------------------------------------
    expected = set(cls.input_grain_keys)
    received = set(resolved_inputs.keys())

    if expected != received:
      raise ValueError(
        f"{cls.key}: input grain keys mismatch "
        f"(expected={expected}, received={received})"
      )

    # --------------------------------------------------
    # config validation
    # --------------------------------------------------
    #config_file
    params = None
    if cls.ParamModel is not None:
      if not config_file:
        raise ValueError(f"{cls.key}: config file is required")
      data = yaml.safe_load(config_file.read_text())
      params = self.ParamModel.model_validate(data)

    # --------------------------------------------------
    # output grain validation
    # --------------------------------------------------
    if output_grain.key != cls.output_grain_key:
      raise ValueError(
        f"{cls.key}: output grain key mismatch "
        f"(expected={cls.output_grain_key}, received={output_grain.key})"
      )

    # --------------------------------------------------
    # output grain provenance
    # --------------------------------------------------
    if context.get('creator') is None:
      raise ValueError(f"{cls.key}: context['creator'] is required")
    output_grain.set_provenance(
      source_procedure=self.procedure_id,
      source_grain_keys=list(resolved_inputs.keys()),
      creator=context['creator'],
    )
    output_grain.create()

    # --------------------------------------------------
    # execute procedure logic
    # --------------------------------------------------
    self._run(
      input_grains=resolved_inputs,
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

  def get_param_type(self) -> Optional[Type]:
    return self.__class__.ParamModel
