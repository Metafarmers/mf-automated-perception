import os
import re
import subprocess
from pathlib import Path
from typing import Optional

_HEX_RE = re.compile(r"^[0-9a-f]{40}$")

def _find_git_repo_root(start: Path) -> Path | None:
  cur = start
  while cur != cur.parent:
    if (cur / ".git").exists():
      return cur
    cur = cur.parent
  return None

def get_git_commit_hash(path: Path) -> Optional[str]:
  repo_root = _find_git_repo_root(path)
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


# --------------------------------------------------
# roots (lazy, env-first)
# --------------------------------------------------

def _get_path_from_env(name: str, default: str) -> Path:
  return Path(os.environ.get(name, default)).resolve()


EXTERNAL_DATA_ROOT = _get_path_from_env(
  "EXTERNAL_DATA_ROOT",
  "/external_data",
)

GRAIN_DATA_ROOT = _get_path_from_env(
  "GRAIN_DATA_ROOT",
  "/workspace/data",
)

SCHEMA_ROOT = _get_path_from_env(
  "SCHEMA_ROOT",
  "/workspace/mf_automated_perception/grain/schema",
)

LOG_DIR_ROOT = _get_path_from_env(
  "LOG_DIR_ROOT",
  "/workspace/data/logs",
)

# --------------------------------------------------
# git commit (best-effort, no hard failure)
# --------------------------------------------------

GIT_COMMIT_HASH = get_git_commit_hash(
  Path(os.environ.get("MF_PROJECT_ROOT", "/workspace")).resolve()
)
