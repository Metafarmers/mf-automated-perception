# mf_automated_perception/procedure/defs/locate_rosbags/definition.py

from typing import ClassVar, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase


class LocateRosbagsConfig(BaseModel):
  search_root: str
  only_most_recent_one: bool = False


class LocateRosbags(ProcedureBase):
  """
  Locate ROS2 rosbags and store their paths as raw grains.
  (definition only, import-safe)
  """

  key: ClassVar[str] = "locate_rosbags"
  version: ClassVar[str] = "0.0.1"

  docker_image: ClassVar[str] = "mf-mantis-eye"
  docker_image_tag: ClassVar[str] = "latest"

  ParamModel: ClassVar[Optional[Type[BaseModel]]] = LocateRosbagsConfig
  description: ClassVar[str] = (
    "Locate ROS2 rosbags and store their paths as raw grains."
  )

  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey] = ("raw", "rosbag", "path")
