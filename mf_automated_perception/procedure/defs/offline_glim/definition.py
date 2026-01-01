from pathlib import Path
import subprocess
from typing import Any, ClassVar, Dict, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.procedure.core.procedure_base import (
  GrainBase,
  GrainKey,
  ProcedureBase,
)

class OfflineGlimConfig(BaseModel):
  livox_topic: str
  livox_imu_topic: str

class OfflineGlim(ProcedureBase):
  key: ClassVar[str] = "offline_glim"
  version: ClassVar[str] = "1.0.0"
  docker_image: ClassVar[str] = 'glim'
  docker_image_tag: ClassVar[str] = 'sync_engine'
  ParamModel: ClassVar[Optional[Type[BaseModel]]] = OfflineGlimConfig
  description: ClassVar[str] = "Offline GLIM procedure."
  required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (('raw', 'rosbag', 'path'),)
  optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = () #pcd, 6d pose with loop closure
  output_grain_key: ClassVar[GrainKey] = ('slam3d', 'glim',)

  def _run(
    self,
    *,
    input_grains,
    output_grain,
    config,
    logger,
  ) -> None:
    logger.info("Running Offline GLIM procedure")

    if not input_grains:
      raise RuntimeError(f"{self.key}: input_grains must be provided.")
    if output_grain is None:
      raise RuntimeError(f"{self.key}: output_grain must be provided.")

    path_grain = input_grains[('raw', 'rosbag', 'path')]

    output_root = Path(output_grain.grain_data_dir_abs)
    output_root.mkdir(parents=True, exist_ok=True)

    glim_dump_dir = output_root / "glim_dump"
    glim_dump_dir.mkdir(parents=True, exist_ok=True)

    ply_path = output_root / "out.ply"

    for item in path_grain.dict_view(table='path_to_raw', limit=-1):
      bag_dir = Path(item['bag_dir'])

      logger.info(f"Running GLIM on rosbag: {bag_dir}")

      cmd_glim = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 run glim_ros glim_rosbag {bag_dir} "
        f"--ros-args "
        f"-p config_path:=/workspace/docker/glim/config "
        f"-p dump_path:={glim_dump_dir} "
        f"-p auto_quit:=true"
      )

      logger.info(f"Executing via bash: {cmd_glim}")
      subprocess.run(
        ["bash", "-lc", cmd_glim],
        check=True,
      )

      cmd_export = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 run glim_ros export_ply_from_dump "
        f"{glim_dump_dir} {ply_path} "
        f"--config_path /workspace/docker/glim/config"
      )

      logger.info(f"Executing via bash: {cmd_export}")
      subprocess.run(
        ["bash", "-lc", cmd_export],
        check=True,
      )

      logger.info(f"PLY exported to: {ply_path}")
