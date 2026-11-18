#!/usr/bin/env bash
# Diagnóstico del puente ESP32-C3 conectado al PC (no a la C2).
set -euo pipefail
cd "$(dirname "$0")"
PORT="${1:-/dev/ttyACM0}"
if [[ ! -e "$PORT" ]]; then
  echo "No existe $PORT. Conecta el C3 por USB y prueba: ls /dev/ttyACM*"
  exit 1
fi
if groups | grep -q dialout || [[ -w "$PORT" ]]; then
  :
else
  echo "Aviso: puede hacer falta: sudo usermod -aG dialout \$USER  (y volver a entrar)"
fi
echo "=== stats 20 s (TICK 0, AUTO ~6 Hz) ==="
python3 listen.py stats --port "$PORT" --seconds 20 --init $'TICK 0\nHZ 6\nAUTO 1'
echo ""
echo "=== captura 12 s ==="
python3 listen.py capture --port "$PORT" --seconds 12 --head motor --note "pc c3 check"
LATEST="$(ls -t captures/sc02-*.bin 2>/dev/null | head -1)"
if [[ -n "${LATEST:-}" ]]; then
  python3 listen.py analyze "$LATEST"
fi
