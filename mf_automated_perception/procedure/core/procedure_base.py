from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from typing import Any, ClassVar, Dict, Literal, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import (
  GrainBase,
  GrainKey,
)
from mf_automated_perception.grain.grain_factory import GrainFactory


class ProcedureBase(BaseModel, ABC):
  """
  Grain -> Grain transformation unit.

  Execution model:
    - Procedure loads its own input grains
    - Entrypoint only orchestrates (parse args, call methods)
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

  @property
  def procedure_id(self) -> str:
    return f"{self.key}:{self.version}"

  model_config = {
    "arbitrary_types_allowed": True,
    "extra": "forbid",
  }

  # ===============================================================
  # grain loading
  # ===============================================================

  def load_inputs(
    self,
    *,
    grain_data_root: Path,
    load_rule: str,
    input_grain_uuids: Optional[Dict[GrainKey, str]],
    logger,
  ) -> Optional[Dict[GrainKey, GrainBase]]:
    """
    Load input grains using a single load rule.

    Args:
      grain_data_root: Root directory for grains
      load_rule: "latest" or "uuid" (applies to ALL inputs)
      input_grain_uuids: Dict of {grain_key: "uuid-string"} (required if load_rule="uuid")
      logger: Logger instance

    Returns:
      Dict of loaded grains or None if no inputs required
    """
    cls = self.__class__

    # No inputs required
    if not cls.required_input_grain_keys and not cls.optional_input_grain_keys:
      return None

    if load_rule == "uuid" and input_grain_uuids is None:
      raise ValueError(f"{cls.key}: input_grain_uuids required when load_rule='uuid'")

    loaded_grains = {}

    # Load required grains
    for key in cls.required_input_grain_keys:
      logger.info(f"Loading required grain {key} with rule={load_rule}")

      if load_rule == "uuid":
        if key not in input_grain_uuids:
          raise ValueError(f"{cls.key}: missing UUID for required grain {key}")
        uuid = input_grain_uuids[key]
      else:
        uuid = None

      grain = GrainFactory.load_grain(
        key=key,
        grain_data_root=grain_data_root,
        load_rule=load_rule,
        uuid=uuid,
      )

      if grain is None:
        raise RuntimeError(f"{cls.key}: failed to load required grain {key}")

      loaded_grains[key] = grain

    # Load optional grains
    for key in cls.optional_input_grain_keys:
      try:
        logger.info(f"Loading optional grain {key} with rule={load_rule}")

        if load_rule == "uuid":
          if key not in input_grain_uuids:
            logger.info(f"Skipping optional grain {key} (no UUID provided)")
            continue
          uuid = input_grain_uuids[key]
        else:
          uuid = None

        grain = GrainFactory.load_grain(
          key=key,
          grain_data_root=grain_data_root,
          load_rule=load_rule,
          uuid=uuid,
        )
        if grain is not None:
          loaded_grains[key] = grain
      except Exception as e:
        logger.warning(f"Failed to load optional grain {key}: {e}")

    return loaded_grains if loaded_grains else None

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
    config: Optional[BaseModel],
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
    if cls.ParamModel is not None and config is None:
      raise ValueError(f"{cls.key}: config is required (ParamModel defined)")

    if config is not None and not isinstance(config, (cls.ParamModel, type(None))):
      raise ValueError(
        f"{cls.key}: config must be instance of {cls.ParamModel}, "
        f"got {type(config)}"
      )

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
      workflow_uuid = context.get("workflow_uuid")
      output_grain.set_provenance(
        source_procedure=self.procedure_id,
        source_grain_keys=input_grain_keys,
        creator=context["creator"],
        workflow_uuid=workflow_uuid,
      )

      output_grain.create()

    # --------------------------------------------------
    # execute procedure logic
    # --------------------------------------------------
    self._run(
      input_grains=input_grains,
      output_grain=output_grain,
      config=config,
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

class ProcedureRunResult(BaseModel):
  """
  Canonical result object for a single procedure execution.
  Serialized to last_run.json and consumed by WorkflowRunner.
  """

  # identity
  procedure_key: str
  procedure_version: Optional[str] = None
  creator: str

  # workflow context
  workflow_uuid: Optional[str] = None
  input_grain_uuid: Optional[str] = None
  output_grain_uuid: Optional[str] = None

  # execution status
  status: Literal["success", "failed"]
  exit_code: int

  # timing
  started_at: datetime
  finished_at: datetime
  duration_sec: Optional[float] = None


  # diagnostics
  log_dir: Optional[Path] = None
  error_message: Optional[str] = None

  class Config:
    arbitrary_types_allowed = True
