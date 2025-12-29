from typing import Any, ClassVar, Dict, Optional, Tuple, Type

from pydantic import BaseModel

from mf_automated_perception.procedure.core.procedure_base import (
  GrainBase,
  GrainKey,
  ProcedureBase,
)

class DummyConfig(BaseModel):
  dummy_param: str = "default_value"

class Dummy(ProcedureBase):
  key: ClassVar[str] = "_dummy"
  version: ClassVar[str] = "1.0.0"
  docker_image: ClassVar[str] = 'mf-mantis-eye'
  docker_image_tag: ClassVar[str] = 'latest'
  ParamModel: ClassVar[Optional[Type]] = DummyConfig
  description: ClassVar[str] = "Dummy procedure for testing purposes."
  input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
  output_grain_key: ClassVar[GrainKey] = ("dummy", )

  def _run(
    self,
    *,
    input_grains: Dict[GrainKey, GrainBase],
    output_grain: GrainBase,
    config,
    logger,
  ) -> None:
    logger.info("Running Dummy procedure")

    # output grain DB open
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
