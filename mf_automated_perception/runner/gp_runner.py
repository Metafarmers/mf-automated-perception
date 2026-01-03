# mflib/perception/automated_perception/runner/gp_runner.py

from typing import Dict
import logging

from mflib.perception.automated_perception.grain.grain_base import (
  GrainBase,
  GrainKey,
)
from mflib.perception.automated_perception.grain.grain_factory import (
  GrainFactory,
)
from mflib.perception.automated_perception.procedure.procedure_factory import (
  ProcedureFactory,
)
from mflib.perception.automated_perception.procedure.procedure_base import (
  ProcedureBase,
)
from mflib.perception.automated_perception.utils.gp_logger import get_logger


class GPRunner:
  """
  GP Runner v0
  - Execute a single procedure
  - Select latest input grains
  - Create output grain
  """

  def run(
    self,
    *,
    procedure_key: str,
    grain_select_rule: str = "latest",
    config: Dict,
    creator: str = "gp_runner",
  ) -> GrainBase:
    """
    Execute a single procedure.

    Parameters
    ----------
    procedure_key : str
      Procedure key registered in ProcedureFactory.
    grain_select_rule : str
      Currently only "latest" is supported.
    config : dict
      Procedure config (validated inside procedure).
    creator : str
      Provenance creator.
    """

    # ------------------------------------------------------------
    # 0. resolve procedure
    # ------------------------------------------------------------
    proc_cls: type[ProcedureBase] = ProcedureFactory.get(procedure_key)
    proc: ProcedureBase = proc_cls()

    # ------------------------------------------------------------
    # 1. select input grains
    # ------------------------------------------------------------
    input_grains: Dict[GrainKey, GrainBase] = {}

    if proc_cls.input_grain_keys:
      if grain_select_rule != "latest":
        raise ValueError(f"Unsupported grain_select_rule: {grain_select_rule}")

      for gkey in proc_cls.input_grain_keys:
        grain = GrainFactory.load_latest(gkey)
        if grain is None:
          raise RuntimeError(f"No grain found for key: {gkey}")
        input_grains[gkey] = grain

    # ------------------------------------------------------------
    # 2. create output grain
    # ------------------------------------------------------------
    output_gkey = proc_cls.output_grain_key
    output_grain = GrainFactory.resolve_class(key=output_gkey)()
    output_grain.set_provenance(
      source_procedure=proc_cls.key,
      source_grain_keys=list(input_grains.keys()),
      creator=creator,
    )

    output_grain.create()

    # create logger
    logger = get_logger(
      name=proc_cls.key,
      level=logging.DEBUG,
    )


    # ------------------------------------------------------------
    # 3. execute procedure
    # ------------------------------------------------------------
    logger.info(
      f"Running procedure {proc.procedure_id} "
      f"with {len(input_grains)} input grains"
    )

    proc.run(
      input_grains=input_grains,
      output_grain=output_grain,
      config=config,
      logger=logger,
    )

    # ------------------------------------------------------------
    # 4. finalize
    # ------------------------------------------------------------
    output_grain.close()
    logger.info("Procedure finished successfully")

    return output_grain
