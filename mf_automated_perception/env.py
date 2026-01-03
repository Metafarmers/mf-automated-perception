import os
import re
import subprocess
from pathlib import Path
from typing import Optional


# ==================================================
# helpers
# ==================================================

def _require_env(name: str) -> str:
  val = os.environ.get(name)
  if val is None or val.strip() == "":
    raise RuntimeError(f"Required environment variable not set: {name}")
  return val


def _require_path_env(name: str) -> Path:
  return Path(_require_env(name)).expanduser().resolve()


def _env_flag_optional(name: str) -> Optional[bool]:
  val = os.environ.get(name)
  if val is None:
    return None
  return val.lower() in ("1", "true", "yes", "on")


# ==================================================
# environment flags (optional)
# ==================================================

# 단순 metadata 용도, 로직 분기에는 사용 안 해도 됨
IS_HOST_ENV: Optional[bool] = _env_flag_optional("MF_IS_HOST_ENV")


# ==================================================
# paths (env-only, MF_ prefix)
# ==================================================

GRAIN_DATA_ROOT: Path = _require_path_env("MF_GRAIN_DATA_ROOT")
BASE_SCHEMA_ROOT: Path = _require_path_env("MF_BASE_SCHEMA_ROOT")
EXTERNAL_DATA_ROOT: Path = _require_path_env("MF_EXTERNAL_DATA_ROOT")
PROJECT_ROOT: Path = _require_path_env("MF_PROJECT_ROOT")
LOG_DIR_ROOT: Path = GRAIN_DATA_ROOT / "logs"


# ==================================================
# git commit hash (best effort)
# ==================================================

_HEX_RE = re.compile(r"^[0-9a-f]{40}$")


def _find_git_repo_root(start: Path) -> Optional[Path]:
  cur = start
  while cur != cur.parent:
    if (cur / ".git").exists():
      return cur
    cur = cur.parent
  return None


def get_git_commit_hash(start: Path) -> Optional[str]:
  repo_root = _find_git_repo_root(start)
  if repo_root is None:
    return None

  try:
    commit = subprocess.check_output(
      ["git", "rev-parse", "HEAD"],
      cwd=repo_root,
      text=True,
      stderr=subprocess.DEVNULL,
    ).strip().lower()

    if not _HEX_RE.match(commit):
      return None

    return commit

  except (subprocess.SubprocessError, FileNotFoundError):
    return None


GIT_COMMIT_HASH: Optional[str] = get_git_commit_hash(PROJECT_ROOT)
