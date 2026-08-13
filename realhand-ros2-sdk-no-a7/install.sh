#!/usr/bin/env bash
set -euo pipefail

default_ros_distro() (
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        . /etc/os-release
        if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
            printf 'jazzy'
            return
        fi
    fi

    printf 'humble'
)

ROS_DISTRO="${ROS_DISTRO:-$(default_ros_distro)}"
TZ="${TZ:-Etc/UTC}"
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_ROS=0
WITH_ARM=0
SKIP_SDK=0
LOCAL_SDK=""
WORKSPACE=""
APT_UPDATED=0

usage() {
    cat <<EOF
Usage: ./install.sh [options]

Installs host dependencies for the RealHand ROS2 SDK.

Options:
  --install-ros       Install ROS2 ${ROS_DISTRO} if it is missing.
  --with-arm          Install the Python SDK with optional A7 Lite kinetix extras.
  --local-sdk PATH    Install the Python SDK from a local editable checkout.
  --skip-sdk          Do not install the Python SDK.
  --workspace PATH    Copy this repo into PATH/src and build that workspace.
  -h, --help          Show this help.

Examples:
  ./install.sh
  ./install.sh --install-ros
  ./install.sh --install-ros --workspace ~/realhand_ros2_ws
  ./install.sh --local-sdk ~/Downloads/realbot-python-sdk-main
EOF
}

log() {
    printf '\n==> %s\n' "$*"
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --install-ros)
            INSTALL_ROS=1
            shift
            ;;
        --with-arm)
            WITH_ARM=1
            shift
            ;;
        --local-sdk)
            [[ $# -ge 2 ]] || die "--local-sdk requires a path"
            LOCAL_SDK="$2"
            shift 2
            ;;
        --skip-sdk)
            SKIP_SDK=1
            shift
            ;;
        --workspace)
            [[ $# -ge 2 ]] || die "--workspace requires a path"
            WORKSPACE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

if [[ -n "${LOCAL_SDK}" && ! -d "${LOCAL_SDK}" ]]; then
    die "local SDK checkout not found: ${LOCAL_SDK}"
fi

SUDO=()
if [[ "${EUID}" -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || die "sudo is required when not running as root"
    SUDO=(sudo)
fi

apt_update() {
    if [[ "${APT_UPDATED}" -eq 0 ]]; then
        "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive TZ="${TZ}" apt-get update
        APT_UPDATED=1
    fi
}

apt_update_force() {
    "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive TZ="${TZ}" apt-get update
    APT_UPDATED=1
}

apt_install() {
    apt_update
    "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive TZ="${TZ}" apt-get install -y "$@"
}

pip_install() {
    local args=()
    if python_externally_managed; then
        args+=(--break-system-packages --ignore-installed)
    fi

    python3 -m pip install "${args[@]}" "$@"
}

python_externally_managed() {
    local py_version marker

    py_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    marker="/usr/lib/python${py_version}/EXTERNALLY-MANAGED"
    [[ -f "${marker}" ]]
}

check_ubuntu() {
    if [[ ! -r /etc/os-release ]]; then
        log "Could not read /etc/os-release; continuing anyway."
        return
    fi

    # shellcheck disable=SC1091
    . /etc/os-release
    if [[ "${ID:-}" != "ubuntu" || ( "${VERSION_ID:-}" != "22.04" && "${VERSION_ID:-}" != "24.04" ) ]]; then
        log "Warning: this package is tested on Ubuntu 22.04/Humble and 24.04/Jazzy; detected ${PRETTY_NAME:-unknown OS}."
    fi
}

ensure_locale() {
    if locale 2>/dev/null | grep -qi 'UTF-8'; then
        return
    fi

    log "Configuring UTF-8 locale"
    apt_install locales
    "${SUDO[@]}" locale-gen en_US en_US.UTF-8
    "${SUDO[@]}" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    export LC_ALL=en_US.UTF-8
}

enable_universe() {
    log "Ensuring Ubuntu Universe repository is enabled"
    apt_install software-properties-common
    "${SUDO[@]}" add-apt-repository -y universe
    apt_update_force
}

install_ros_apt_source() {
    log "Adding ROS2 apt repository"
    apt_install curl ca-certificates

    local version codename deb_path
    version="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')"
    [[ -n "${version}" ]] || die "could not determine latest ros-apt-source version"

    # shellcheck disable=SC1091
    . /etc/os-release
    codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
    [[ -n "${codename}" ]] || die "could not determine Ubuntu codename"

    deb_path="/tmp/ros2-apt-source.deb"
    curl -L -o "${deb_path}" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
    "${SUDO[@]}" dpkg -i "${deb_path}"
    apt_update_force
}

ensure_ros() {
    local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [[ -f "${ros_setup}" ]]; then
        log "Found ROS2 setup: ${ros_setup}"
        return
    fi

    if [[ "${INSTALL_ROS}" -ne 1 ]]; then
        die "ROS2 ${ROS_DISTRO} was not found at ${ros_setup}. Install ROS2 first, or rerun with --install-ros."
    fi

    ensure_locale
    enable_universe
    install_ros_apt_source

    log "Installing ROS2 ${ROS_DISTRO} ros-base"
    "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive TZ="${TZ}" apt-get upgrade -y
    apt_install "ros-${ROS_DISTRO}-ros-base"
}

install_system_dependencies() {
    log "Installing system dependencies"
    enable_universe
    apt_install \
        git \
        ca-certificates \
        iproute2 \
        can-utils \
        python3-pip \
        python3-colcon-common-extensions \
        python3-pyqt5
}

install_python_sdk() {
    if [[ "${SKIP_SDK}" -eq 1 ]]; then
        log "Skipping Python SDK install"
        return
    fi

    if python_externally_managed; then
        log "Using system pip; Python environment is externally managed"
    else
        log "Upgrading pip"
        pip_install --upgrade pip
    fi

    if [[ -n "${LOCAL_SDK}" ]]; then
        local target="${LOCAL_SDK}"
        if [[ "${WITH_ARM}" -eq 1 ]]; then
            target="${target}[kinetix]"
        fi
        log "Installing local Python SDK: ${target}"
        pip_install -e "${target}"
    elif [[ "${WITH_ARM}" -eq 1 ]]; then
        log "Installing Python SDK with arm extras"
        pip_install "realhand[kinetix] @ git+https://github.com/RealHand-Robotics/realbot-python-sdk.git"
    else
        log "Installing Python SDK"
        pip_install git+https://github.com/RealHand-Robotics/realbot-python-sdk.git
    fi
}

copy_repo_to_workspace() {
    [[ -n "${WORKSPACE}" ]] || return

    local workspace_dir dest
    mkdir -p "${WORKSPACE}"
    workspace_dir="$(cd "${WORKSPACE}" && pwd)"
    if [[ "${workspace_dir}" == "/" || "${workspace_dir}" == "${HOME}" ]]; then
        die "--workspace must be a dedicated workspace directory, for example ~/realhand_ros2_ws"
    fi

    dest="${workspace_dir}/src/realhand-ros2-sdk"

    if [[ -e "${dest}" ]]; then
        die "workspace destination already exists: ${dest}"
    fi

    log "Copying repository to ${dest}"
    mkdir -p "${workspace_dir}/src"
    mkdir -p "${dest}"
    cp -a \
        "${REPO_DIR}/README.md" \
        "${REPO_DIR}/package.xml" \
        "${REPO_DIR}/pyproject.toml" \
        "${REPO_DIR}/setup.cfg" \
        "${REPO_DIR}/setup.py" \
        "${REPO_DIR}/run_gui.sh" \
        "${REPO_DIR}/examples" \
        "${REPO_DIR}/launch" \
        "${REPO_DIR}/resource" \
        "${REPO_DIR}/src" \
        "${REPO_DIR}/test" \
        "${dest}/"

    log "Building workspace ${workspace_dir}"
    (
        cd "${workspace_dir}"
        set +u
        # shellcheck disable=SC1091
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
        set -u
        colcon build --symlink-install
        set +u
        # shellcheck disable=SC1091
        source install/setup.bash
        set -u
        ros2 pkg prefix realhand_ros2
        ros2 pkg executables realhand_ros2
    )
}

main() {
    export ROS_DISTRO
    export TZ
    export DEBIAN_FRONTEND=noninteractive
    check_ubuntu
    ensure_ros
    install_system_dependencies
    install_python_sdk
    copy_repo_to_workspace

    log "Install checks completed"
    if [[ -z "${WORKSPACE}" ]]; then
        printf 'Source ROS2 before building or running:\n'
        printf '  source /opt/ros/%s/setup.bash\n' "${ROS_DISTRO}"
    else
        printf 'Source the workspace before running:\n'
        printf '  source %s/install/setup.bash\n' "$(readlink -f "${WORKSPACE}")"
    fi
}

main
