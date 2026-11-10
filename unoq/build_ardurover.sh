#!/usr/bin/env bash
# Static aarch64 ardurover for the Arduino Uno Q (waf board "unoq"), built in a Debian arm64
# container. Native on Apple Silicon and on arm64 Linux hosts; under emulation elsewhere.
#
#   unoq/build_ardurover.sh            # build, copy to unoq/app/bin/ardurover
#   unoq/build_ardurover.sh shell      # shell in the build container
#   unoq/build_ardurover.sh test       # run the binary against the emulated microcontroller
#   unoq/build_ardurover.sh gateway-test   # the web gateway and API against the same pair
#
# The build tree lives in the Docker volume unoq-ardurover-build, so the host SITL build
# directory (ardupilot/build) is left alone.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=unoq-ardurover-build
VOLUME=unoq-ardurover-build

docker build --platform linux/arm64 -t "$IMAGE" "$ROOT/unoq/docker"
docker volume create "$VOLUME" >/dev/null

run() {
  # UNOQ_RT=1 lets the container use realtime scheduling, as ardurover does on a plain Linux board
  docker run --rm --platform linux/arm64 ${TTY:+-it} ${UNOQ_RT:+--cap-add SYS_NICE --ulimit rtprio=99} \
    -v "$ROOT:/work" -v "$VOLUME:/build" -w /work/ardupilot "$IMAGE" "$@"
}

if [[ "${1:-}" == "shell" ]]; then
  TTY=1 run bash
  exit 0
fi
if [[ "${1:-}" == "gateway-test" ]]; then
  # the web gateway and API (unoq/app/python) against the binary and the emulated microcontroller
  shift
  run env PYTHONDONTWRITEBYTECODE=1 python3 /work/tools/unoq/gateway_test.py --binary /build/unoq/bin/ardurover "$@"
  exit $?
fi
if [[ "${1:-}" == "test" ]]; then
  # the binary just built, against the emulated microcontroller (tools/unoq/bench_test.py)
  shift
  run env PYTHONDONTWRITEBYTECODE=1 python3 /work/tools/unoq/bench_test.py --binary /build/unoq/bin/ardurover "$@"
  exit $?
fi

run bash -c '
  set -e
  python3 ./waf configure --board unoq --static --no-submodule-update -o /build
  python3 ./waf rover
  mkdir -p /work/unoq/app/bin
  cp /build/unoq/bin/ardurover /work/unoq/app/bin/ardurover
  ls -l /work/unoq/app/bin/ardurover
  file /work/unoq/app/bin/ardurover 2>/dev/null || true
'
