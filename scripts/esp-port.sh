#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
if [ "$MODE" != "--print" ] && [ "$MODE" != "" ]; then
  MODE="--exec"
fi

if [ -n "${PORT:-}" ]; then
  if [ "$MODE" = "--exec" ]; then
    ESP_PORT="$PORT" "$@"
  else
    printf '%s\n' "$PORT"
  fi
  exit 0
fi

for pattern in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/ttyUSB* /dev/ttyACM*; do
  for port in $pattern; do
    if [ -e "$port" ]; then
      if [ "$MODE" = "--exec" ]; then
        ESP_PORT="$port" "$@"
      else
        printf '%s\n' "$port"
      fi
      exit 0
    fi
  done
done

echo "No ESP serial port found. Set PORT=/dev/cu..." >&2
exit 1
