from typing import ClassVar, Optional, Tuple, Type, Dict

from pydantic import BaseModel

from mf_automated_perception.procedure.core.procedure_base import (
  GrainBase,
  GrainKey,
  ProcedureBase,
)


class DummyConfig(BaseModel):
  dummy_param: str = "default_value"

class Dummy(ProcedureBase):
  """
  Dummy procedure definition (import-safe).
  """
  key: ClassVar[str] = "_dummy"
  version: ClassVar[str] = "1.0.0"
  docker_image: ClassVar[str] = "mf-mantis-eye"
  docker_image_tag: ClassVar[str] = "latest"
  ParamModel: ClassVar[Optional[Type[BaseModel]]] = DummyConfig
  description: ClassVar[str] = "Dummy procedure for testing purposes."
  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey] = ("dummy",)
