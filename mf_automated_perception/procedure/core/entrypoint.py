import argparse
import logging
from pathlib import Path

from mf_automated_perception.env import LOG_DIR_ROOT
from mf_automated_perception.grain.grain_factory import GrainFactory
from mf_automated_perception.procedure.core.procedure_factory import ProcedureFactory
from mf_automated_perception.utils.gp_logger import get_logger


def main() -> None:
  parser = argparse.ArgumentParser(
    description="Run an automated perception procedure"
  )

  parser.add_argument(
    "--procedure",
    required=True,
    help="Procedure key (e.g. locate_rosbags)",
  )

  parser.add_argument(
    "--config",
    type=Path,
    required=False,  # it can be None
    help="Path to procedure config YAML file",
  )

  parser.add_argument(
    "--creator",
    required=True,
    help="Creator name recorded in provenance",
  )

  parser.add_argument(
    "--log-name",
    default="procedure",
    help="Logger name",
  )

  parser.add_argument(
    "--log-level",
    default="INFO",
    choices=["DEBUG", "INFO", "WARNING", "ERROR"],
  )

  args = parser.parse_args()

  logger = get_logger(
    name=args.log_name,
    file_dir=LOG_DIR_ROOT,
    level=getattr(logging, args.log_level),
  )

  ProcCls = ProcedureFactory.resolve_class(args.procedure)
  procedure = ProcCls()

  logger.info(f"Starting procedure: {args.procedure}")
  logger.info(f"Config file: {args.config}")
  logger.info(f"Creator: {args.creator}")

  GrainFactory.build_registry()
  # output grain
  output_grain = GrainFactory.resolve_class(procedure.output_grain_key)()
  try:
    procedure.run(
      input_grains=None,
      load_input_rule='latest',
      output_grain=output_grain,
      config_file=args.config,
      context={
        "creator": args.creator,
      },
      logger=logger,
    )

  except Exception:
    logger.exception("Procedure execution failed")
    raise

  finally:
    try:
      if "output_grain" in locals():
        output_grain.close()
    except Exception:
      logger.exception("Failed to close output grain")

  logger.info("Procedure finished successfully")


if __name__ == "__main__":
  main()
