#!/bin/bash
set -e
PYTHON_VERSION_REQUIRED="3.10"

# utils
log() { echo "[install] $1"; }

# check project root
PROJECT_ROOT="$(pwd)"
PROJECT_NAME="$(basename "${PROJECT_ROOT}")"

if [ "${PROJECT_NAME}" != "mf-automated-perception" ]; then
  echo "ERROR: run this script inside mf-automated-perception directory"
  exit 1
fi



VENV_DIR="${PROJECT_ROOT}/.venv"

if [ -d "${VENV_DIR}" ]; then
  echo "Removing existing ${VENV_DIR}"
  rm -rf "${VENV_DIR}"
fi


SETUP_BASH="${PROJECT_ROOT}/setup.bash"

PYTHON_BIN="python${PYTHON_VERSION_REQUIRED}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  echo "[INFO] ${PYTHON_BIN} not found. Installing via deadsnakes PPA..."

  sudo apt update
  sudo apt install -y software-properties-common

  # add deadsnakes PPA only if not already added
  if ! grep -R "deadsnakes/ppa" /etc/apt/sources.list.d >/dev/null 2>&1; then
    sudo add-apt-repository -y ppa:deadsnakes/ppa
  fi

  sudo apt update
  sudo apt install -y \
    "python${PYTHON_VERSION_REQUIRED}" \
    "python${PYTHON_VERSION_REQUIRED}-venv" \
    "python${PYTHON_VERSION_REQUIRED}-dev"
else
  echo "[INFO] ${PYTHON_BIN} already installed"
fi

# install system dependency
log "installing system dependencies"
sudo apt update
sudo apt install -y python3-venv

# create venv
if [ ! -d "${VENV_DIR}" ]; then
  log "creating virtual environment"
  python${PYTHON_VERSION_REQUIRED} -m venv "${VENV_DIR}"
else
  log "virtual environment already exists"
fi

# activate venv
log "activating virtual environment"
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip setuptools wheel


log "generating setup.bash"
cat <<EOF > "${SETUP_BASH}"
#!/bin/bash
export MF_PROJECT_ROOT="${PROJECT_ROOT}"
export MF_GRAIN_DATA_ROOT="${MF_PROJECT_ROOT}/data"
export MF_BASE_SCHEMA_ROOT="${MF_PROJECT_ROOT}/mf_automated_perception/grain/schema"
export MF_LOG_DIR_ROOT="${MF_GRAIN_DATA_ROOT}/logs"
export MF_EXTERNAL_DATA_ROOT="/home/tw/mf/data/mantis-eye/dongtan_automated_perception/dongtan_mot_routine"
source "${VENV_DIR}/bin/activate"
source "${HOME}/.bash_completions/mf-eye.sh"
EOF



# install python package
log "installing python package (editable)"
pip install --upgrade pip
pip install -e .
mf-eye --install-completion


chmod +x "${SETUP_BASH}"
#echo "source ${SETUP_BASH}" >> ~/.bashrc



# build docker image
#IMAGE_NAME="mf-mantis-eye"
#log "building docker image: ${IMAGE_NAME}"
#python build_docker.py --image_name "${IMAGE_NAME}"

# done
log "done"
log "run: source setup.bash"

