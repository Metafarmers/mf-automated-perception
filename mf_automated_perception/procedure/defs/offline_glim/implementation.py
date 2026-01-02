# mf_automated_perception/procedure/defs/offline_glim/implementation.py

import subprocess
from pathlib import Path
from typing import Dict, Optional

from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from mf_automated_perception.procedure.defs.offline_glim.definition import (
  OfflineGlim,
)


class OfflineGlimImpl(OfflineGlim):
  """
  Offline GLIM procedure implementation.
  """

  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    logger.info("Running Offline GLIM procedure")

    if not input_grains:
      raise RuntimeError(
        f"{self.key}: input_grains must be provided."
      )
    if output_grain is None:
      raise RuntimeError(
        f"{self.key}: output_grain must be provided."
      )

    path_grain = input_grains[("raw", "rosbag", "path")]

    output_root = Path(output_grain.grain_data_dir_abs)
    output_root.mkdir(parents=True, exist_ok=True)

    ply_path = output_root / "out.ply"

    for item in path_grain.dict_view(table="path_to_raw", limit=-1):
      bag_dir = Path(item["bag_dir"])

      glim_dump_dir = output_root / "dump" / bag_dir.name
      glim_dump_dir.mkdir(parents=True, exist_ok=True)
      logger.info(f'glim dump dir: {glim_dump_dir}')

      # logger.info(f"Running GLIM on rosbag: {bag_dir}")

      cmd_glim = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 run glim_ros glim_rosbag "
        f"{bag_dir} "
        "--ros-args "
        "-p config_path:=/workspace/docker/glim/config "
        f"-p dump_path:={glim_dump_dir} "
        "-p auto_quit:=true"
      )

      logger.info(f"Executing via bash: {cmd_glim}")
      subprocess.run(
        ["bash", "-lc", cmd_glim],
        check=True,
      )

      cmd_export = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 run glim_ros export_ply_from_dump "
        f"{glim_dump_dir} {ply_path} "
        "--config_path /workspace/docker/glim/config"
      )

      logger.info(f"Executing via bash: {cmd_export}")
      subprocess.run(
        ["bash", "-lc", cmd_export],
        check=True,
      )

      logger.info(f"PLY exported to: {ply_path}")

      # to odometry topic 
      filename = glim_dump_dir / "traj_imu.txt"

      conn = output_grain.open()

      try:
        with filename.open("r") as f:
          for line in f:
            line = line.strip()
            if not line:
              continue

            parts = line.split()
            if len(parts) != 8:
              logger.warning(f"Invalid traj line (skip): {line}")
              continue

            # ----------------------------------------
            # parse timestamp: sec.nsec
            # ----------------------------------------
            ts_str = parts[0]
            if "." not in ts_str:
              logger.warning(f"Invalid timestamp format (skip): {ts_str}")
              continue

            sec_str, nsec_str = ts_str.split(".", 1)
            timestamp_sec = int(sec_str)

            # nsec는 최대 9자리로 정규화
            timestamp_nsec = int(nsec_str.ljust(9, "0")[:9])

            # ----------------------------------------
            # pose
            # ----------------------------------------
            px, py, pz = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])

            # ----------------------------------------
            # insert
            # ----------------------------------------
            conn.execute(
              """
              INSERT INTO odometry (
                timestamp_sec,
                timestamp_nsec,
                px, py, pz,
                qx, qy, qz, qw,
                vx, vy, vz,
                wx, wy, wz
              )
              VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
              """,
              (
                timestamp_sec,
                timestamp_nsec,
                px, py, pz,
                qx, qy, qz, qw,
                None, None, None,   # linear velocity
                None, None, None,   # angular velocity
              ),
            )

        conn.commit()
        logger.info(f"Inserted odometry from {filename}")

      finally:
        output_grain.close()
