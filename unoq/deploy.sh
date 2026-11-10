#!/usr/bin/env bash
# Copies the ArduHumanoid app to the Arduino Uno Q over adb, as ~/ArduinoApps/arduhumanoid.
#
#   unoq/deploy.sh                 # app, ardurover binary, robot and policies
#   unoq/deploy.sh --torque-off    # as above, and data/site.parm with NNM_DIS_POSE 0: the autopilot
#                                  # never powers the servos while disarmed (first connection)
#   unoq/deploy.sh --slow-bridge   # sketch built for a router still at 115200 baud (wiring check)
#
# Then on the board: arduino-app-cli app start user:arduhumanoid   (or Run in App Lab)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ADB="${ADB:-$(command -v adb || ls "$HOME"/Library/Arduino15/packages/arduino/tools/adb/*/adb 2>/dev/null | tail -1)}"
[[ -x "$ADB" ]] || { echo "adb not found: set ADB=/path/to/adb" >&2; exit 1; }
[[ -f "$ROOT/unoq/app/bin/ardurover" ]] || { echo "build the binary first: unoq/build_ardurover.sh" >&2; exit 1; }
ROBOT="$ROOT/robots/xgoduck"
ls "$ROBOT"/policies/*.nnm >/dev/null 2>&1 || { echo "no policies: run tools/robots/import_xgoduck.py" >&2; exit 1; }

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
APP="$STAGE/arduhumanoid"
mkdir -p "$APP/data/APM/nnm/xgoduck/policies"
for d in sketch python assets tools LICENSES bin; do cp -R "$ROOT/unoq/app/$d" "$APP/"; done
cp "$ROOT/unoq/app/app.yaml" "$ROOT/unoq/app/THIRD_PARTY_NOTICES.md" "$APP/"
cp "$ROBOT/robot/robot.bin" "$ROBOT/robot/skills.bin" "$APP/data/APM/nnm/xgoduck/"
cp "$ROBOT"/policies/*.nnm "$APP/data/APM/nnm/xgoduck/policies/"
find "$APP" -name '__pycache__' -prune -exec rm -rf {} + ; find "$APP" -name '._*' -delete

REMOTE=/home/arduino/ArduinoApps/arduhumanoid
SITE=keep
for arg in "$@"; do
  case "$arg" in
    --torque-off) SITE=off ;;
    --torque-on) SITE=on ;;
    --slow-bridge) sed -i.bak 's|^// #define DUCK_BRIDGE_SLOW|#define DUCK_BRIDGE_SLOW|' "$APP/sketch/protocol.h" && rm "$APP/sketch/protocol.h.bak" ;;
    *) echo "unknown option $arg" >&2; exit 1 ;;
  esac
done
[[ "$SITE" == off ]] && printf '# first connection: servos never powered while disarmed\nNNM_DIS_POSE 0\n' > "$APP/data/site.parm"

# parameters, logs and the Python environment already on the board are kept
"$ADB" shell "mkdir -p $REMOTE/data && rm -rf $REMOTE/sketch $REMOTE/python $REMOTE/assets $REMOTE/tools $REMOTE/bin $REMOTE/LICENSES"
[[ "$SITE" == on ]] && "$ADB" shell "rm -f $REMOTE/data/site.parm"
"$ADB" push "$APP/." "$REMOTE/" | tail -1
"$ADB" shell "chmod +x $REMOTE/bin/ardurover; ls $REMOTE $REMOTE/data; grep -n 'define DUCK_BRIDGE_SLOW' $REMOTE/sketch/protocol.h; cat $REMOTE/data/site.parm 2>/dev/null"
