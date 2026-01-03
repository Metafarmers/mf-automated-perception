# mf_automated_perception/procedure/defs/offline_glim/definition.py

from typing import ClassVar, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase


class OfflineGlimConfig(BaseModel):
  livox_topic: str
  livox_imu_topic: str


class OfflineGlim(ProcedureBase):
  """
  Offline GLIM procedure definition (import-safe).
  """

  key: ClassVar[str] = "offline_glim"
  version: ClassVar[str] = "0.0.1"

  docker_image: ClassVar[str] = "glim"
  docker_image_tag: ClassVar[str] = "sync_engine"

  ParamModel: ClassVar[Optional[Type[BaseModel]]] = OfflineGlimConfig
  description: ClassVar[str] = "Offline GLIM procedure."

  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
    ("raw", "rosbag", "path"),
  )
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey] = ("slam3d", "glim")