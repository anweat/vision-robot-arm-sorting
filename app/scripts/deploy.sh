#!/bin/sh
# Copy the app (src/, config/, scripts/, contracts/, samples/) to /root/arm-app on the board. app/ has the same
# layout as /root/arm-app, so nothing is renamed on the way; data/, venv-vision/ and the gateway token on
# the board are left alone. Plain ssh: it asks for the board password unless a key is set up.
#
#   app/scripts/deploy.sh                       # cable: root@192.168.137.100
#   BOARD_HOST=root@100.85.35.98 BOARD_PORT=2222 app/scripts/deploy.sh      # through the phone relay (doc 31 §2.1)
#   app/scripts/deploy.sh --restart             # then stop and bring up camera, console and gateway (board_up.sh)
#
# Files deleted from the repository stay on the board; they are not imported by anything that remains.
# --restart does not move the arm (the console only reads the servos until a client sends a motion command),
# but check the arm's surroundings before driving it (docs/05-板卡连接与硬件实测.md).
set -eu

HOST="${BOARD_HOST:-root@192.168.137.100}"
PORT="${BOARD_PORT:-22}"
APP="$(cd "$(dirname "$0")/.." && pwd)"

COPYFILE_DISABLE=1 tar --no-xattrs --no-mac-metadata --exclude "__pycache__" --exclude ".DS_Store" \
  -C "$APP" -cf - src config scripts contracts samples |
  ssh -p "$PORT" "$HOST" 'mkdir -p /root/arm-app && tar -C /root/arm-app -xf -'
echo "copied to $HOST:/root/arm-app"

if [ "${1:-}" = --restart ]; then
  ssh -p "$PORT" "$HOST" 'sh /root/arm-app/scripts/board_up.sh stop; sh /root/arm-app/scripts/board_up.sh'
fi
