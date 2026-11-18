#!/usr/bin/env python3
"""Captura y lectura de prueba del Delta-SC-02.

Guarda la UART en crudo y comprueba si esos bytes encajan con tramas
ya publicadas de otros Delta. Un acierto de checksum no identifica
esta unidad: las escalas de ángulo, distancia y velocidad salen
marcadas como hipótesis de esas otras placas.

  python3 listen.py capture --port /dev/ttyUSB0 --baud 115200 --seconds 8 \
      --head mano --note "5V logica, motor sin alimentar"
  python3 listen.py analyze captures/sc02-....bin
  python3 listen.py self-test
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

SYNC = 0xAA
TYPE_BYTE = 0x61
DTYPE_MEAS = 0xAD
DTYPE_SPEED = 0xAE
MIN_LEN = 8
MAX_LEN = 250

HERE = Path(__file__).resolve().parent
CAPTURE_DIR = HERE / "captures"


def u16(buf: bytes, i: int) -> int:
    return (buf[i] << 8) | buf[i + 1]


def build_frame(version: int, dtype: int, payload: bytes) -> bytes:
    """payload empieza en el byte de velocidad (offset 8)."""
    body = bytes([version, TYPE_BYTE, dtype]) + len(payload).to_bytes(2, "big") + payload
    length = 3 + len(body)
    frame = bytes([SYNC]) + length.to_bytes(2, "big") + body
    if len(frame) != length or length != 8 + len(payload):
        raise RuntimeError("longitud de trama de prueba incoherente")
    checksum = sum(frame) & 0xFFFF
    return frame + checksum.to_bytes(2, "big")


def _samples(pkt: bytes, first: int, count: int) -> list[dict]:
    out = []
    for k in range(count):
        off = first + 3 * k
        units = u16(pkt, off + 1)
        out.append({
            "q": pkt[off],
            "raw": units,
            "mm_si_x0_25": units * 0.25,
        })
    return out


def parse_frame(pkt: bytes, offset: int) -> dict:
    length = u16(pkt, 0 + 1)
    version = pkt[3]
    dtype = pkt[5]
    dlen = u16(pkt, 6)
    consistent = (8 + dlen) == length
    hz = pkt[8] / 20.0 if length > 8 else None
    frame = {
        "offset": offset,
        "length": length,
        "version": version,
        "dtype": dtype,
        "dlen": dlen,
        "dlen_cuadra": consistent,
        "hz_si_x20": hz,
        "layout": "desconocido",
        "start_deg_si_x100": None,
        "end_deg_si_x100": None,
        "samples": [],
    }
    if not consistent:
        return frame
    if dtype == DTYPE_SPEED:
        frame["layout"] = "velocidad"
        return frame
    if dtype != DTYPE_MEAS:
        return frame
    if dlen >= 5 and (dlen - 5) % 3 == 0:
        count = (dlen - 5) // 3
        frame["layout"] = "medidas-sin-end-angle"
        frame["start_deg_si_x100"] = u16(pkt, 11) * 0.01
        frame["samples"] = _samples(pkt, 13, count)
        return frame
    if dlen >= 7 and (dlen - 7) % 3 == 0:
        count = (dlen - 7) // 3
        frame["layout"] = "medidas-con-end-angle"
        frame["start_deg_si_x100"] = u16(pkt, 11) * 0.01
        frame["end_deg_si_x100"] = u16(pkt, 13) * 0.01
        frame["samples"] = _samples(pkt, 15, count)
    return frame


def scan(data: bytes) -> tuple[list[dict], int]:
    """Tramas con checksum de suma de 16 bits, y cuántos candidatos fallaron."""
    hits: list[dict] = []
    near = 0
    i = 0
    n = len(data)
    while i + MIN_LEN <= n:
        if data[i] != SYNC:
            i += 1
            continue
        if i + 6 > n:
            break
        length = u16(data, i + 1)
        if not (MIN_LEN <= length <= MAX_LEN) or i + length + 2 > n:
            i += 1
            continue
        if data[i + 4] != TYPE_BYTE or data[i + 5] not in (DTYPE_MEAS, DTYPE_SPEED):
            i += 1
            continue
        pkt = data[i:i + length + 2]
        if (sum(pkt[:length]) & 0xFFFF) != u16(pkt, length):
            near += 1
            i += 1
            continue
        hits.append(parse_frame(pkt, i))
        i += length + 2
    return hits, near


def _hex(blob: bytes, limit: int = 48) -> str:
    shown = blob[:limit]
    text = shown.hex(" ")
    if len(blob) > limit:
        text += " …"
    return text


PWM_MARK = b"## PWM "


def pwm_segments(data: bytes) -> list[tuple[int | None, bytes]]:
    parts: list[tuple[int | None, bytes]] = []
    pct: int | None = None
    start = 0
    pos = 0
    while True:
        i = data.find(PWM_MARK, pos)
        if i < 0:
            if start < len(data):
                parts.append((pct, data[start:]))
            break
        if i > start:
            parts.append((pct, data[start:i]))
        j = i + len(PWM_MARK)
        k = j
        while k < len(data) and 48 <= data[k] <= 57:
            k += 1
        if k == j:
            pos = i + 1
            continue
        pct = int(data[j:k])
        if k < len(data) and data[k] == ord("\n"):
            k += 1
        start = k
        pos = k
    return parts


def pwm_table(data: bytes) -> list[str]:
    parts = [(pct, blob) for pct, blob in pwm_segments(data) if pct is not None]
    if not parts:
        return []
    lines = ["PWM%   bytes   tramas   medidas   solo-vel"]
    for pct, blob in parts:
        hits, _near = scan(blob)
        meas = sum(1 for h in hits if h["dtype"] == DTYPE_MEAS and h["samples"])
        speed = sum(1 for h in hits if h["dtype"] == DTYPE_SPEED or h["layout"] == "velocidad")
        lines.append(
            f"{pct:4d}  {len(blob):6d}  {len(hits):7d}  {meas:8d}  {speed:9d}"
        )
    return lines


def report(data: bytes, hits: list[dict], near: int) -> str:
    lines: list[str] = []
    table = pwm_table(data)
    if table:
        lines.extend(table)
        lines.append("")
    lines.append(f"bytes: {len(data)}")
    if not data:
        lines.append("captura vacía: el adaptador no recibió transiciones.")
        return "\n".join(lines)

    counts = Counter(data)
    common = ", ".join(f"{b:02X}:{n}" for b, n in counts.most_common(6))
    lines.append(f"0xAA: {counts[SYNC]}   bytes más frecuentes: {common}")
    lines.append(f"inicio: {_hex(data)}")
    lines.append(
        f"tramas con checksum: {len(hits)}   "
        f"cabeceras AA/61/AD|AE con checksum mal: {near}"
    )

    if not hits:
        if len(data) < 32:
            lines.append("casi no hay bytes. Cabezal parado, TX sin llegar, o masa no común.")
        elif near:
            lines.append(
                "hay cabeceras de la familia Delta pero el checksum no cuadra. "
                "Puede ser otro baudios, o esta unidad suma distinto."
            )
        else:
            lines.append(
                "no aparece la trama Delta conocida. "
                "Repite la captura al otro candidato: 115200 y 230400."
            )
        return "\n".join(lines)

    by_ver = Counter(h["version"] for h in hits)
    by_dtype = Counter(h["dtype"] for h in hits)
    by_layout = Counter(h["layout"] for h in hits)
    lines.append("versiones: " + ", ".join(f"0x{v:02X}×{n}" for v, n in sorted(by_ver.items())))
    lines.append(
        "tipos: "
        + ", ".join(f"0x{d:02X}×{n}" for d, n in sorted(by_dtype.items()))
    )
    lines.append("layouts: " + ", ".join(f"{name}×{n}" for name, n in by_layout.items()))

    speeds = [h["hz_si_x20"] for h in hits if h["hz_si_x20"] is not None]
    if speeds:
        lines.append(
            f"velocidad si el byte vale Hz×20: "
            f"{min(speeds):.2f} … {max(speeds):.2f}"
        )

    measured = [h for h in hits if h["samples"]]
    if measured:
        raws = [s["raw"] for h in measured for s in h["samples"]]
        zeros = sum(1 for h in measured for s in h["samples"] if s["q"] == 0)
        total = sum(len(h["samples"]) for h in measured)
        lines.append(
            f"muestras: {total}   q=0: {zeros}   "
            f"raw {min(raws)}…{max(raws)}   "
            f"mm si ×0.25: {min(raws) * 0.25:.1f}…{max(raws) * 0.25:.1f}"
        )

    lines.append("primeras tramas:")
    for h in hits[:3]:
        kind = "AD" if h["dtype"] == DTYPE_MEAS else "AE"
        extra = ""
        if h["start_deg_si_x100"] is not None:
            extra = f"  start≈{h['start_deg_si_x100']:.2f}°"
        if h["end_deg_si_x100"] is not None:
            extra += f"  end≈{h['end_deg_si_x100']:.2f}°"
        lines.append(
            f"  @{h['offset']} ver=0x{h['version']:02X} {kind} "
            f"len={h['length']} {h['layout']}  "
            f"hz≈{h['hz_si_x20']:.2f}  n={len(h['samples'])}{extra}"
        )
    lines.append(
        "Esto encaja con una trama de la familia. "
        "No fija todavía la escala ni el mapa angular del SC-02."
    )
    return "\n".join(lines)


def cmd_analyze(args: argparse.Namespace) -> int:
    path = Path(args.file)
    data = path.read_bytes()
    hits, near = scan(data)
    print(report(data, hits, near))
    return 0


def cmd_self_test(_args: argparse.Namespace) -> int:
    meas_2a = build_frame(0x01, DTYPE_MEAS, bytes([
        120, 0, 0, 0x03, 0xE8,
        0x90, 0x0F, 0xA0,
        0x00, 0x00, 0x00,
    ]))
    meas_2c = build_frame(0x13, DTYPE_MEAS, bytes([
        140, 0, 10, 0x08, 0xCA, 0x0B, 0xB8,
        0x80, 0x10, 0x00,
        0x40, 0x00, 0x64,
    ]))
    speed_2c = build_frame(0x13, DTYPE_SPEED, bytes([186]))
    broken = bytearray(meas_2a)
    broken[-1] ^= 0xFF
    blob = bytes([0x00, 0xFF, 0x11, 0x22, 0x33]) + meas_2a + bytes(broken) + meas_2c + speed_2c

    hits, near = scan(blob)
    layouts = [h["layout"] for h in hits]
    versions = [h["version"] for h in hits]
    ok = (
        layouts == ["medidas-sin-end-angle", "medidas-con-end-angle", "velocidad"]
        and versions == [0x01, 0x13, 0x13]
        and near == 1
        and hits[0]["samples"][0]["raw"] == 0x0FA0
        and hits[0]["samples"][1]["q"] == 0
        and abs(hits[0]["hz_si_x20"] - 6.0) < 1e-9
        and hits[1]["end_deg_si_x100"] == 30.0
        and abs(hits[2]["hz_si_x20"] - 9.3) < 1e-9
        and hits[0]["offset"] == 5
    )
    marked = b"## PWM 0\n" + bytes(8) + b"## PWM 40\n" + meas_2a
    table = pwm_table(marked)
    ok = ok and table == [
        "PWM%   bytes   tramas   medidas   solo-vel",
        "   0       8        0         0          0",
        f"  40  {len(meas_2a):6d}        1         1          0",
    ]
    print(report(blob, hits, near))
    if not ok:
        print("SELF-TEST MAL", file=sys.stderr)
        return 1
    print("SELF-TEST BIEN")
    return 0


def cmd_capture(args: argparse.Namespace) -> int:
    import serial
    from serial.tools import list_ports

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("no hay puertos serie")
            return 1
        for p in ports:
            print(f"{p.device}\t{p.description}")
        return 0

    if not args.port:
        print("falta --port.  python3 listen.py capture --list", file=sys.stderr)
        return 2

    CAPTURE_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    bin_path = CAPTURE_DIR / f"sc02-{stamp}.bin"
    meta_path = CAPTURE_DIR / f"sc02-{stamp}.json"

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.2
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    # El CP2102 de la ESP32 resetea la placa al abrir. DTR/RTS sueltos
    # la dejan arrancar el puente en vez de quedarse en el bootloader.
    time.sleep(0.3)
    ser.dtr = False
    ser.rts = False
    ser.reset_input_buffer()
    print(
        f"{args.port} @ {args.baud} 8N1 → {bin_path.name}"
        + ("  (Ctrl-C para cortar)" if args.seconds <= 0 else f"  {args.seconds:.0f} s"),
        file=sys.stderr,
    )
    total = 0
    started = time.monotonic()
    next_status = started + 1.0
    deadline = None if args.seconds <= 0 else started + args.seconds
    try:
        with bin_path.open("wb", buffering=0) as out:
            while deadline is None or time.monotonic() < deadline:
                chunk = ser.read(4096)
                if chunk:
                    out.write(chunk)
                    total += len(chunk)
                now = time.monotonic()
                if now >= next_status:
                    elapsed = now - started
                    rate = total / elapsed if elapsed else 0.0
                    print(f"\r{total} bytes  {rate:.0f} B/s", end="", file=sys.stderr, flush=True)
                    next_status = now + 1.0
    except KeyboardInterrupt:
        print("\ncorte", file=sys.stderr)
    finally:
        ser.close()
    elapsed = max(time.monotonic() - started, 1e-6)
    blob = bin_path.read_bytes()
    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "port": args.port,
        "baud": args.baud,
        "format": "8N1",
        "seconds": round(elapsed, 3),
        "bytes": len(blob),
        "sha256": hashlib.sha256(blob).hexdigest(),
        "head": args.head,
        "note": args.note,
        "logic_v": args.logic_v,
        "tx_level": args.tx_level,
        "file": bin_path.name,
    }
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    print(f"\n{len(blob)} bytes en {elapsed:.1f} s → {bin_path}", file=sys.stderr)
    hits, near = scan(blob)
    print(report(blob, hits, near))
    return 0


def _open_port(port: str, baud: int, discard: bool = True):
    import serial

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.2
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(0.3)
    ser.dtr = False
    ser.rts = False
    if discard:
        ser.reset_input_buffer()
    return ser


def _send_lines(ser, script: str) -> None:
    for line in script.splitlines():
        line = line.strip()
        if line:
            ser.write(f"{line}\n".encode("ascii"))
            ser.flush()
            time.sleep(0.05)


def cmd_stats(args: argparse.Namespace) -> int:
    """Cuenta tramas AD/AE en vivo (puente C3 o LIDAR directo)."""
    ser = _open_port(args.port, args.baud, discard=False)
    if args.init.strip():
        time.sleep(0.4)
        _send_lines(ser, args.init)
    buf = bytearray()
    ad = ae = 0
    meas_pts = 0
    last_hz = 0.0
    t0 = time.monotonic()
    last_print = t0
    print(
        f"{args.port} @ {args.baud}  cada {args.interval:.0f} s: AD AE Hz muestras  "
        "(Ctrl-C para salir)",
        flush=True,
    )
    try:
        while args.seconds <= 0 or time.monotonic() - t0 < args.seconds:
            chunk = ser.read(512)
            if chunk:
                buf.extend(chunk)
            while True:
                if buf.startswith(b"##"):
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    del buf[: nl + 1]
                    continue
                pos = buf.find(b"\n##")
                if pos >= 0:
                    end = buf.find(b"\n", pos + 1)
                    if end < 0:
                        break
                    del buf[pos : end + 1]
                    continue
                break
            hits, _ = scan(bytes(buf))
            if hits:
                used = max(h["offset"] + h["length"] + 2 for h in hits)
                del buf[:used]
                for h in hits:
                    if h["dtype"] == DTYPE_MEAS:
                        ad += 1
                        meas_pts += len(h.get("samples") or [])
                    elif h["dtype"] == DTYPE_SPEED:
                        ae += 1
                    if h.get("hz_si_x20") is not None:
                        last_hz = h["hz_si_x20"]
            elif len(buf) > 32768:
                del buf[:-512]
            now = time.monotonic()
            if now - last_print >= args.interval:
                print(
                    f"  AD={ad:4d}  AE={ae:4d}  ~{last_hz:5.2f} Hz  muestras={meas_pts:5d}",
                    flush=True,
                )
                ad = ae = meas_pts = 0
                last_print = now
            if not chunk:
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\ncorte", flush=True)
    finally:
        ser.close()
    return 0


def cmd_watch(args: argparse.Namespace) -> int:
    """Imprime todo lo que llega. Las líneas de texto salen tal cual; el resto, en hex."""
    CAPTURE_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    bin_path = CAPTURE_DIR / f"sc02-watch-{stamp}.bin"
    ser = _open_port(args.port, args.baud, discard=False)
    print(f"{args.port} @ {args.baud}  todo el tráfico, sin filtrar  → {bin_path.name}", flush=True)
    pending = bytearray()
    try:
        with bin_path.open("wb", buffering=0) as out:
            while True:
                chunk = ser.read(1024)
                if not chunk:
                    if pending and not any(b == 10 for b in pending):
                        # Sigue siendo binario a medias: no lo ocultes si ya hay un bloque.
                        if len(pending) >= 64:
                            out.write(pending)
                            print(pending.hex(" "), flush=True)
                            pending.clear()
                    continue
                pending.extend(chunk)
                while True:
                    nl = pending.find(b"\n")
                    if nl < 0:
                        break
                    line = bytes(pending[:nl])
                    del pending[:nl + 1]
                    out.write(line + b"\n")
                    if line.endswith(b"\r"):
                        line = line[:-1]
                    if all(32 <= b < 127 for b in line):
                        print(line.decode("ascii"), flush=True)
                    else:
                        print((line + b"\n").hex(" "), flush=True)
    except KeyboardInterrupt:
        print("\ncorte", flush=True)
    finally:
        if pending:
            with bin_path.open("ab", buffering=0) as out:
                out.write(pending)
            print(pending.hex(" "), flush=True)
        ser.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Captura cruda y contraste de tramas del Delta-SC-02")
    sub = parser.add_subparsers(dest="cmd", required=True)

    cap = sub.add_parser("capture", help="volcar la UART a un .bin sin convertir a texto")
    cap.add_argument("--port")
    cap.add_argument("--baud", type=int, default=115200)
    cap.add_argument("--seconds", type=float, default=8, help="0 = hasta Ctrl-C")
    cap.add_argument("--head", choices=("parado", "mano", "motor"), default="mano")
    cap.add_argument("--note", default="")
    cap.add_argument("--logic-v", default="")
    cap.add_argument("--tx-level", default="")
    cap.add_argument("--list", action="store_true")
    cap.set_defaults(func=cmd_capture)

    ana = sub.add_parser("analyze", help="contrastar un .bin con las tramas Delta publicadas")
    ana.add_argument("file")
    ana.set_defaults(func=cmd_analyze)

    test = sub.add_parser("self-test", help="comprobar el lector con tramas sintéticas")
    test.set_defaults(func=cmd_self_test)

    stats = sub.add_parser("stats", help="contar AD/AE en vivo (ideal con C3 en USB)")
    stats.add_argument("--port", default="/dev/ttyACM0")
    stats.add_argument("--baud", type=int, default=115200)
    stats.add_argument("--seconds", type=float, default=0, help="0 = hasta Ctrl-C")
    stats.add_argument("--interval", type=float, default=2.0)
    stats.add_argument(
        "--init",
        default="TICK 0\nHZ 6\nAUTO 1",
        help="comandos al abrir (una línea por comando)",
    )
    stats.set_defaults(func=cmd_stats)

    watch = sub.add_parser("watch", help="mostrar en consola cada byte que llegue")
    watch.add_argument("--port", default="/dev/ttyUSB0")
    watch.add_argument("--baud", type=int, default=115200)
    watch.set_defaults(func=cmd_watch)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
