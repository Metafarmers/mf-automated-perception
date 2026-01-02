from typing import ClassVar, Dict, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase


class DecodeRosbagsConfig(BaseModel):
  decode_images: bool = True
  decode_pointclouds: bool = True
  topic_to_sensor_name_map: Optional[Dict[str, str]] = None


class DecodeRosbags(ProcedureBase):
  """
  Decode rosbag procedure definition (import-safe).
  """

  key: ClassVar[str] = "decode_rosbags"
  version: ClassVar[str] = "0.0.1"

  docker_image: ClassVar[str] = "mf-mantis-eye"
  docker_image_tag: ClassVar[str] = "latest"

  ParamModel: ClassVar[Optional[Type[BaseModel]]] = DecodeRosbagsConfig
  description: ClassVar[str] = "Decompose rosbag into smaller grains."

  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
    ("raw", "rosbag", "path"),
  )
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey] = ("raw", "rosbag", "decoded")

  # implementation pointer
  implementation: ClassVar[str] = (
    "mf_automated_perception.procedure.defs.decode_rosbags.implementation"
  )
