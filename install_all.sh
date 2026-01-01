#!/bin/bash
set -e

# utils
log() { echo "[install] $1"; }

# check project root
PROJECT_ROOT="$(pwd)"
PROJECT_NAME="$(basename "${PROJECT_ROOT}")"

if [ "${PROJECT_NAME}" != "mf-automated-perception" ]; then
  echo "ERROR: run this script inside mf-automated-perception directory"
  exit 1
fi

# config
VENV_DIR="${PROJECT_ROOT}/.venv"
SETUP_BASH="${PROJECT_ROOT}/setup.bash"
IMAGE_NAME="mf-mantis-eye"

# install system dependency
log "installing system dependencies"
sudo apt update
sudo apt install -y python3-venv

# create venv
if [ ! -d "${VENV_DIR}" ]; then
  log "creating virtual environment"
  python3 -m venv "${VENV_DIR}"
else
  log "virtual environment already exists"
fi

# activate venv
log "activating virtual environment"
source "${VENV_DIR}/bin/activate"

# install python package
log "installing python package (editable)"
pip install --upgrade pip
pip install -e .

# generate setup.bash
log "generating setup.bash"
cat << EOF > "${SETUP_BASH}"
#!/bin/bash
export MF_PROJECT_ROOT=${PROJECT_ROOT}
export MF_GRAIN_DATA_ROOT=${PROJECT_ROOT}/data
export MF_BASE_SCHEMA_ROOT=${PROJECT_ROOT}/mf_automated_perception/grain/schema
export MF_LOG_DIR_ROOT=\$MF_GRAIN_DATA_ROOT/logs
export MF_EXTERNAL_DATA_ROOT=/home/tw/mf/data/mantis-eye/dongtan_automated_perception/dongtan_mot_routine
source ${PROJECT_ROOT}/.venv/bin/activate
source '${HOME}/.bash_completions/mf-eye.sh'
EOF

chmod +x "${SETUP_BASH}"

# build docker image
log "building docker image: ${IMAGE_NAME}"
python3 build_docker.py --image_name "${IMAGE_NAME}"

# done
log "done"
log "run: source setup.bash"

