#!/usr/bin/env bash
# One-time macOS setup step: mujoco's `mjpython` launcher (required by
# `mujoco.viewer.launch_passive` on macOS — plain `python` raises
# "launch_passive requires that the Python script be run under mjpython
# on macOS", unconditionally, even after this fix) does
# `dlopen(@executable_path/../lib/libpythonX.Y.dylib)` relative to the
# venv's `bin/python`. A `uv`-created venv does not copy the shared
# library into `.venv/lib/` — it only symlinks `bin/python` back to the
# base interpreter install, which *does* have a real
# libpythonX.Y.dylib, just not at the path mjpython's relative lookup
# expects. Without this symlink, `uv run mjpython ...` fails with:
#   Library not loaded: @executable_path/../lib/libpython3.12.dylib
#
# This script symlinks the base interpreter's shared library into
# `.venv/lib/` so mjpython's dlopen resolves. It derives the exact
# source path programmatically (via sysconfig on the base interpreter
# found through the venv's pyvenv.cfg `home` key) rather than
# hardcoding a Python version, so it keeps working across Python
# version bumps. Safe to re-run (idempotent).
#
# Usage: simulation/scripts/fix_mjpython_dylib.sh [path-to-venv]
#   (defaults to the `.venv` next to this script's parent directory,
#   i.e. simulation/.venv)

set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "fix_mjpython_dylib.sh: not on macOS, nothing to do." >&2
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${1:-"${SCRIPT_DIR}/../.venv"}"
VENV_DIR="$(cd "${VENV_DIR}" && pwd)"
PYVENV_CFG="${VENV_DIR}/pyvenv.cfg"

if [[ ! -f "${PYVENV_CFG}" ]]; then
    echo "fix_mjpython_dylib.sh: no pyvenv.cfg at ${PYVENV_CFG}; run 'uv sync' first." >&2
    exit 1
fi

# pyvenv.cfg's `home` key points at the base interpreter's bin/ dir.
BASE_BIN="$(grep -E '^home *=' "${PYVENV_CFG}" | sed -E 's/^home *= *//')"
BASE_PYTHON="${BASE_BIN}/python3"

if [[ ! -x "${BASE_PYTHON}" ]]; then
    echo "fix_mjpython_dylib.sh: base interpreter not found at ${BASE_PYTHON}." >&2
    exit 1
fi

# Ask the base interpreter's own sysconfig where its shared library
# lives — this resolves through any uv version-alias symlinks
# (e.g. cpython-3.12-macos-aarch64-none -> cpython-3.12.11-...) and
# survives Python patch/minor version bumps.
read -r LIBDIR LIBNAME <<< "$(
    "${BASE_PYTHON}" -c "
import sysconfig
libdir = sysconfig.get_config_var('LIBDIR')
libname = sysconfig.get_config_var('INSTSONAME') or sysconfig.get_config_var('LDLIBRARY')
print(libdir, libname)
"
)"

SOURCE_LIB="${LIBDIR}/${LIBNAME}"
DEST_LIB="${VENV_DIR}/lib/${LIBNAME}"

if [[ ! -f "${SOURCE_LIB}" ]]; then
    echo "fix_mjpython_dylib.sh: expected shared library not found at ${SOURCE_LIB}." >&2
    exit 1
fi

if [[ -L "${DEST_LIB}" && "$(readlink "${DEST_LIB}")" == "${SOURCE_LIB}" ]]; then
    echo "fix_mjpython_dylib.sh: already linked (${DEST_LIB} -> ${SOURCE_LIB}); nothing to do."
    exit 0
fi

if [[ -e "${DEST_LIB}" && ! -L "${DEST_LIB}" ]]; then
    echo "fix_mjpython_dylib.sh: refusing to overwrite non-symlink at ${DEST_LIB}." >&2
    exit 1
fi

mkdir -p "${VENV_DIR}/lib"
ln -sf "${SOURCE_LIB}" "${DEST_LIB}"
echo "fix_mjpython_dylib.sh: linked ${DEST_LIB} -> ${SOURCE_LIB}"
