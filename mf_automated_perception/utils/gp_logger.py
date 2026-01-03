import logging
from pathlib import Path
from mf_automated_perception.env import LOG_DIR_ROOT

def get_logger(
  name: str,
  level: int = logging.INFO,
) -> logging.Logger:
  logger = logging.getLogger(name)
  logger.setLevel(level)

  # 중복 handler 방지
  if logger.handlers:
    return logger

  formatter = logging.Formatter(
    "[%(asctime)s][%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
  )

  # ---- console handler ----
  console_handler = logging.StreamHandler()
  console_handler.setFormatter(formatter)
  logger.addHandler(console_handler)

  # ---- file handler ----
  file_dir = LOG_DIR_ROOT / name
  file_dir.mkdir(parents=True, exist_ok=True)

  log_path = file_dir / f"{name}.log"
  file_handler = logging.FileHandler(log_path, mode="a")
  file_handler.setFormatter(formatter)
  logger.addHandler(file_handler)

  return logger
