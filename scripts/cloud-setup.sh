#!/usr/bin/env bash
# Get a Linux machine ready to build and test this repository: a Claude Code
# cloud session (run automatically from .claude/settings.json when a session
# starts) or a CI runner. Does nothing on your own PC. Safe to run again -
# every step is skipped once it is done - and it never fails the session:
# whatever could not be set up is reported instead.
#
#   bash scripts/cloud-setup.sh           # acts only when CLAUDE_CODE_REMOTE=true
#   bash scripts/cloud-setup.sh --force   # CI, or any Linux box
#
# SKIP_PIO=1 leaves PlatformIO out (the host tests do not need it).
set -u
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ] && [ "${1:-}" != "--force" ]; then
    exit 0
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/automotive-telemetry"
PY="$(command -v python3 || command -v python)"
SUDO=""
if [ "$(id -u)" != "0" ] && command -v sudo >/dev/null 2>&1; then SUDO="sudo -n"; fi
problems=()
say() { echo "[setup] $*"; }

# 1. 32-bit C headers. The screens' test harness compiles one file with -m32
#    to learn LVGL's struct sizes on the ESP32 (a 32-bit chip) and so estimate
#    the device's LVGL heap use.
if ! echo '#include <stdint.h>' | gcc -m32 -x c -E - >/dev/null 2>&1; then
    say "installing gcc-multilib"
    { $SUDO apt-get update -qq && $SUDO apt-get install -y -qq gcc-multilib g++-multilib; } >/dev/null 2>&1 \
        || problems+=("gcc-multilib (screen harness heap estimate)")
fi

# 2. The libraries the host test harnesses compile against, at the versions
#    pinned in platformio.ini. A firmware build puts them in .pio/libdeps;
#    without one they come straight from GitHub into .pio/libdeps/host.
fetch() {  # <name> <git tag> <github repo> <project>...
    local name=$1 tag=$2 repo=$3; shift 3
    local src="$CACHE/$name-$tag" proj
    for proj in "$@"; do
        if compgen -G "$ROOT/$proj/.pio/libdeps/*/$name" >/dev/null; then continue; fi
        if [ ! -d "$src" ]; then
            mkdir -p "$CACHE"
            git clone -q --depth 1 --branch "$tag" "https://github.com/$repo.git" "$src" 2>/dev/null \
                || { rm -rf "$src"; problems+=("$name $tag from github.com/$repo"); return; }
        fi
        mkdir -p "$ROOT/$proj/.pio/libdeps/host"
        cp -r "$src" "$ROOT/$proj/.pio/libdeps/host/$name"
        say "$name $tag -> $proj"
    done
}
fetch ArduinoJson v7.4.3 bblanchon/ArduinoJson master screens/screen1-round screens/screen2-cluster
fetch lvgl        v8.4.0 lvgl/lvgl              screens/screen1-round screens/screen2-cluster

# 3. PlatformIO, for firmware builds. Its first build downloads the ESP32
#    toolchains from *.platformio.org, which the cloud environment's network
#    setting must allow (see README.md, "Using Claude on GitHub").
if [ "${SKIP_PIO:-}" != "1" ] && ! "$PY" -m platformio --version >/dev/null 2>&1; then
    say "installing PlatformIO"
    "$PY" -m pip install -q platformio >/dev/null 2>&1 \
        || "$PY" -m pip install -q --user --break-system-packages platformio >/dev/null 2>&1 \
        || problems+=("PlatformIO (pip install platformio)")
fi

if [ ${#problems[@]} -eq 0 ]; then
    say "ready: host tests (python scripts/test_all.py) and firmware builds (python3 -m platformio run -d <project>)"
else
    say "NOT set up: ${problems[*]}"
fi
exit 0
