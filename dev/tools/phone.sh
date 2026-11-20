#!/bin/sh
# Relay-phone helpers over adb (doc 36). Each command is one adb step; nothing here touches the board.
#
#   sh dev/tools/phone.sh connect [IP:PORT]   adb over Tailscale: default <phone Tailscale IP>:5555; while the
#                                             phone is plugged into the board use the wireless-debugging port
#                                             (Settings → Developer options → Wireless debugging; adb pair first)
#   sh dev/tools/phone.sh rndis               USB function → RNDIS tethering (vivo flips its own switch back;
#                                             each replug returns to charging; doc 31 §2.1d)
#   sh dev/tools/phone.sh app                 bring the App to the front (the relay resumes if it was on)
#   sh dev/tools/phone.sh sim                 adb reverse tcp:18090 tcp:8090: the phone sees this computer's
#                                             gateway as the "board" (App relay: board 127.0.0.1, port 18090)
#   sh dev/tools/phone.sh install             build and install the debug APK (vivo asks to confirm on screen)
#   sh dev/tools/phone.sh status              python3 dev/tools/check_links.py --only ts,relay,phone
#
# PHONE = Tailscale name of the relay phone (default $ARM_PHONE or v2339a); ADB_SERIAL picks one adb device.
set -eu
cd "$(dirname "$0")/../.."
PHONE="${ARM_PHONE:-v2339a}"
PKG=com.example.myapplication00001
TS=/Applications/Tailscale.app/Contents/MacOS/Tailscale
[ -x "$TS" ] || TS=tailscale

phone_ip() {
  "$TS" status --json | python3 -c '
import json, sys
want = sys.argv[1].lower()
for p in (json.load(sys.stdin).get("Peer") or {}).values():
    if want in (p.get("HostName", "").lower(), p.get("DNSName", "").split(".")[0].lower()):
        print(p["TailscaleIPs"][0]); break' "$PHONE"
}

adb_() { if [ -n "${ADB_SERIAL:-}" ]; then adb -s "$ADB_SERIAL" "$@"; else adb "$@"; fi; }

case "${1:-}" in
connect)
  target="${2:-}"
  if [ -z "$target" ]; then
    ip="$(phone_ip)"; [ -n "$ip" ] || { echo "no Tailscale peer named $PHONE"; exit 1; }
    target="$ip:5555"
  fi
  adb connect "$target"
  adb devices -l ;;
rndis)
  adb_ shell svc usb setFunctions rndis
  sleep 2
  echo "sys.usb.state=$(adb_ shell getprop sys.usb.state | tr -d '\r')" ;;
app)
  adb_ shell monkey -p "$PKG" -c android.intent.category.LAUNCHER 1 >/dev/null
  sleep 1
  echo "pid $(adb_ shell pidof "$PKG" | tr -d '\r')" ;;
sim)
  adb_ reverse tcp:18090 tcp:8090
  adb_ reverse --list ;;
install)
  cd app/mobile/android
  [ -f local.properties ] || echo "sdk.dir=$HOME/Library/Android/sdk" > local.properties
  ./gradlew -q assembleDebug
  adb_ install -r -t app/build/outputs/apk/debug/app-debug.apk ;;
status)
  exec python3 dev/tools/check_links.py --only ts,relay,phone --phone "$PHONE" ;;
*)
  sed -n '2,16p' "$0"; exit 2 ;;
esac
