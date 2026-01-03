from typing import Dict, Optional

from mf_automated_perception.procedure.core.procedure_base import (
  GrainBase,
  GrainKey,
)
from mf_automated_perception.procedure.defs.dummy.definition import Dummy


class DummyImpl(Dummy):
  """
  Dummy procedure implementation.
  """

  def _run(
    self,
    *,
    input_grains: Optional[Dict[GrainKey, GrainBase]],
    output_grain: Optional[GrainBase],
    config,
    logger,
  ) -> None:
    logger.info("Running Dummy procedure")

    if input_grains:
      raise RuntimeError(
        f"{self.key}: input_grains should be empty, "
        f"got: {list(input_grains.keys())}"
      )

    if output_grain is None:
      raise RuntimeError(
        f"{self.key}: output_grain must be provided."
      )

    conn = output_grain.open()

    try:
      for i in range(3):
        conn.execute(
          """
          INSERT INTO odometry (
            timestamp_sec, timestamp_nsec,
            px, py, pz,
            qx, qy, qz, qw,
            vx, vy, vz,
            wx, wy, wz
          )
          VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
          """,
          (
            100 + i,
            i * 100_000_000,
            float(i),
            float(i + 1),
            float(i + 2),
            0.0, 0.0, 0.0, 1.0,
            0.1 * i, 0.2 * i, 0.3 * i,
            0.01 * i, 0.02 * i, 0.03 * i,
          ),
        )

      conn.commit()
      logger.info("Dummy odometry rows inserted")

    finally:
      output_grain.close()
