from pathlib import Path
import subprocess
import os


def run_procedure(
  *,
  procedure: str,
  docker_image: str,
  creator: str,
  params_file: Path | None = None,
  bash: bool = False,
) -> subprocess.CompletedProcess:
  # --------------------------------------------------
  # host-side required envs
  # --------------------------------------------------
  host_grain_data_root = os.environ.get("MF_GRAIN_DATA_ROOT")
  host_external_data_root = os.environ.get("MF_EXTERNAL_DATA_ROOT")
  host_project_root = os.environ.get("MF_PROJECT_ROOT")

  if host_grain_data_root is None:
    raise RuntimeError("MF_GRAIN_DATA_ROOT is not set on host")
  if host_external_data_root is None:
    raise RuntimeError("MF_EXTERNAL_DATA_ROOT is not set on host")
  if host_project_root is None:
    raise RuntimeError("MF_PROJECT_ROOT is not set on host")

  host_grain_data_root = Path(host_grain_data_root).resolve()
  host_external_data_root = Path(host_external_data_root).resolve()
  host_project_root = Path(host_project_root).resolve()

  # docker run command
  cmd: list[str] = [
    "docker", "run", "--rm", "--gpus", "all",
  ]

  # interactive 옵션은 bash일 때만
  if bash:
    cmd += ["-it"]

  # memory
  cmd += [
    "--ipc", "host",
    "-e", "QT_X11_NO_MITSHM=1",
  ]

  display = os.environ.get("DISPLAY", ":0")
  xauth = os.environ.get("XAUTHORITY")

  if xauth is None:
    xauth = str(Path.home() / ".Xauthority")

  cmd += [
    "-e", f"DISPLAY={display}",
    "-e", f"XAUTHORITY={xauth}",
    "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
    "-v", f"{xauth}:{xauth}",
  ]

  cmd += [
    "-v", f"{host_project_root}:/workspace",
    "-v", f"{host_grain_data_root}:/data",
    "-v", f"{host_external_data_root}:/external_data",
  ]

  # --------------------------------------------------
  # env vars inside docker (env.py contract)
  # --------------------------------------------------
  cmd += [
    "-e", "MF_GRAIN_DATA_ROOT=/data",
    "-e", "MF_BASE_SCHEMA_ROOT=/workspace/mf_automated_perception/grain/schema",
    "-e", "MF_LOG_DIR_ROOT=/data/logs",
    "-e", "MF_EXTERNAL_DATA_ROOT=/external_data",
    "-e", "MF_PROJECT_ROOT=/workspace",
  ]

  # container command
  if bash:
    cmd += [
      docker_image,
      "/bin/bash",
    ]
  else:
    cmd += [
      docker_image,
      "python3", "-m",
      "mf_automated_perception.procedure.core.entrypoint",
      "--procedure", procedure,
      "--creator", creator,
      "--config", str(
        params_file
        if params_file is not None
        else f"/workspace/params/{procedure}.yaml"
      ),
      "--load-input-rule", "latest",
    ]

  out = subprocess.run(cmd, check=True)
  print(f"Procedure '{procedure}' completed with return code: {out.returncode}")
  return out
