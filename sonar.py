#!/usr/bin/env python3
"""Sonar del Delta-SC-02.

Lee la UART, busca la ventana de giro en la que aparecen distancias y
dibuja un PPI. La escala (Hz = byte/20, mm = raw×0.25, ángulos×0.01)
es la de otros Delta con la misma cabecera. No está confirmada en esta unidad.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path

SYNC = 0xAA
TYPE_BYTE = 0x61
DTYPE_MEAS = 0xAD
DTYPE_SPEED = 0xAE
PWM_MIN = 40
PWM_MAX = 90
PWM_BOOT = 70

HERE = Path(__file__).resolve().parent
CAPTURE_DIR = HERE / "captures"

BG = "#07110c"
SCOPE = "#03140c"
RING = "#1c6a38"
TICK = "#8fd9a0"
SWEEP = "#d8ffc8"
TEXT = "#d7f5cf"
DIM = "#7dbe8c"
AMBER = "#e6c56a"
BLIP_COLORS = ("#12381f", "#1d5c32", "#2f8a4a", "#5fce78", "#b6f5c4", "#f4fff4")


def u16(buf: bytes, i: int) -> int:
    return (buf[i] << 8) | buf[i + 1]


class Parser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, chunk: bytes) -> list[dict]:
        self.buf.extend(chunk)
        frames: list[dict] = []
        data = self.buf
        i = 0
        n = len(data)
        while i + 8 <= n:
            if data[i] != SYNC:
                i += 1
                continue
            length = u16(data, i + 1)
            if not (8 <= length <= 250):
                i += 1
                continue
            if i + length + 2 > n:
                break
            if data[i + 4] != TYPE_BYTE or data[i + 5] not in (DTYPE_MEAS, DTYPE_SPEED):
                i += 1
                continue
            pkt = data[i:i + length + 2]
            if (sum(pkt[:length]) & 0xFFFF) != u16(pkt, length):
                i += 1
                continue
            frame = self._parse(pkt)
            if frame is not None:
                frames.append(frame)
            i += length + 2
        del self.buf[:i]
        if len(self.buf) > 8192:
            del self.buf[:-512]
        return frames

    def _parse(self, pkt: bytes) -> dict | None:
        length = u16(pkt, 1)
        dtype = pkt[5]
        dlen = u16(pkt, 6)
        if 8 + dlen != length or length < 9:
            return None
        hz = pkt[8] / 20.0
        frame = {"dtype": dtype, "hz": hz, "samples": []}
        if dtype != DTYPE_MEAS:
            return frame
        if dlen >= 7 and (dlen - 7) % 3 == 0:
            count = (dlen - 7) // 3
            start = u16(pkt, 11) * 0.01
            end = u16(pkt, 13) * 0.01
            first = 15
        elif dlen >= 5 and (dlen - 5) % 3 == 0:
            count = (dlen - 5) // 3
            start = u16(pkt, 11) * 0.01
            end = None
            first = 13
        else:
            return frame
        span = None
        if end is not None:
            span = (end - start) % 360.0
        samples = []
        for k in range(count):
            off = first + 3 * k
            raw = u16(pkt, off + 1)
            q = pkt[off]
            if raw == 0 or q == 0:
                continue
            if span is None:
                ang = start
            elif count == 1:
                ang = start
            else:
                ang = (start + span * k / (count - 1)) % 360.0
            samples.append((ang, raw * 0.25 / 1000.0, q))
        frame["samples"] = samples
        frame["end"] = end if end is not None else start
        return frame


class Sonar:
    def __init__(self, port: str, baud: int) -> None:
        import serial

        self.port = port
        self.baud = baud
        self.parser = Parser()
        self.points: list[tuple[float, float, float]] = []
        self.hz = 0.0
        self.bearing = 0.0
        self.view_deg = 0.0
        self.pwm = PWM_BOOT
        self.target = 6.0
        self.manual = False
        self.locked = False
        self.stuck = 0
        self.hold = 0
        self.med_before: float | None = None
        self.span_m = 6.0
        self.recent_hz: list[float] = []
        self.meas_samples = 0
        self.speed_frames = 0
        self.meas_frames = 0
        self.last_step = time.monotonic()
        self.opened = time.monotonic()
        self.last_draw = time.monotonic()
        self.status = "lazo de revoluciones"
        self.textbuf = bytearray()
        CAPTURE_DIR.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.log_path = CAPTURE_DIR / f"sc02-sonar-{stamp}.bin"
        self.log = self.log_path.open("wb", buffering=0)

        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0
        ser.xonxoff = False
        ser.rtscts = False
        ser.dsrdtr = False
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(0.25)
        ser.dtr = False
        ser.rts = False
        self.ser = ser

        self.root = tk.Tk()
        self.root.title("SC-02 sonar")
        self.root.configure(bg=BG)
        self.root.geometry("1040x760")
        self.canvas = tk.Canvas(self.root, width=720, height=720, bg=BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, padx=12, pady=12)
        side = tk.Frame(self.root, bg=BG)
        side.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 16), pady=18)
        self.labels: dict[str, tk.Label] = {}
        for key, text in (
            ("title", "SC-02"),
            ("pwm", "PWM —"),
            ("hz", "— / 6.00 Hz"),
            ("hit", "sin distancias"),
            ("mode", "lazo de revoluciones"),
            ("scale", "escala de otros Delta\nHz = byte/20\nmm = raw × 0.25\ngrados × 0.01"),
            ("view", "vista +0°"),
            ("keys", "↑ ↓ PWM ±1 %\n← → gira la imagen, 1°"),
        ):
            font = ("DejaVu Sans", 22 if key == "title" else 14)
            color = TEXT if key in ("title", "pwm", "hz", "hit", "view") else DIM
            if key == "mode":
                color = AMBER
            label = tk.Label(
                side, text=text, bg=BG, fg=color, font=font,
                justify=tk.LEFT, anchor="w",
            )
            label.pack(fill=tk.X, pady=(0 if key == "title" else 8, 0))
            self.labels[key] = label
        self.root.bind("<Up>", lambda _e: self.set_pwm(self.pwm + 1))
        self.root.bind("<Down>", lambda _e: self.set_pwm(self.pwm - 1))
        self.root.bind("<Left>", lambda _e: self.turn(-1))
        self.root.bind("<Right>", lambda _e: self.turn(1))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(200, self._raise)
        self.root.after(30, self.tick)
        print(f"sonar {port} @ {baud} → {self.log_path.name}", flush=True)

    def _raise(self) -> None:
        self.root.lift()
        self.root.attributes("-topmost", True)
        self.root.after(600, lambda: self.root.attributes("-topmost", False))

    def turn(self, delta: float) -> None:
        self.view_deg = (self.view_deg + delta) % 360.0
        shown = self.view_deg if self.view_deg <= 180.0 else self.view_deg - 360.0
        self.labels["view"].configure(text=f"vista {shown:+.0f}°")
        print(f"vista {shown:+.0f}", flush=True)

    def nudge(self, delta: float) -> None:
        self.target = min(20.0, max(0.0, self.target + delta))
        self.ser.write(f"HZ {self.target:.2f}\n".encode("ascii"))
        self.ser.flush()
        self.labels["hz"].configure(text=f"{self.hz:.2f} / {self.target:.2f} Hz")
        self.labels["mode"].configure(text="lazo de revoluciones", fg=TEXT)
        print(f"HZ {self.target:.2f}", flush=True)

    def apply_status(self, line: str) -> None:
        parts = line.split()
        kv: dict[str, str] = {}
        i = 1
        while i + 1 < len(parts):
            kv[parts[i]] = parts[i + 1]
            i += 2
        if "TGT" in kv:
            try:
                self.target = float(kv["TGT"])
            except ValueError:
                pass
        if "PWM" in kv:
            self.labels["pwm"].configure(text=f"PWM {kv['PWM']} %")
        auto = kv.get("AUTO")
        if auto == "1":
            self.labels["mode"].configure(text="lazo de revoluciones", fg=TEXT)
        elif auto == "0":
            self.labels["mode"].configure(text="PWM manual", fg=AMBER)

    def note_text(self, chunk: bytes) -> None:
        self.textbuf.extend(chunk)
        while True:
            nl = self.textbuf.find(b"\n")
            if nl < 0:
                break
            line = bytes(self.textbuf[:nl]).strip(b"\r")
            del self.textbuf[:nl + 1]
            if line.startswith(b"##"):
                self.apply_status(line.decode("ascii", "replace"))
        if len(self.textbuf) > 512:
            del self.textbuf[:-64]

    def set_pwm(self, pct: int) -> None:
        pct = max(PWM_MIN, min(PWM_MAX, pct))
        if pct == self.pwm and self.ser is not None:
            pass
        self.pwm = pct
        self.ser.write(f"PWM {pct}\n".encode("ascii"))
        self.ser.flush()
        self.labels["pwm"].configure(text=f"PWM {pct} %")
        print(f"PWM {pct}", flush=True)

    def close(self) -> None:
        try:
            self.ser.write(b"PWM 0\n")
            self.ser.flush()
        except Exception:
            pass
        try:
            self.log.close()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        self.root.destroy()

    def handle(self, frame: dict) -> None:
        self.hz = frame["hz"]
        self.recent_hz.append(frame["hz"])
        if frame["dtype"] == DTYPE_SPEED:
            self.speed_frames += 1
            return
        self.meas_frames += 1
        now = time.monotonic()
        for ang, meters, _q in frame["samples"]:
            self.points.append((ang, meters, now))
            self.meas_samples += 1
        if frame["samples"]:
            self.bearing = frame.get("end", frame["samples"][-1][0])

    def hunt(self, now: float) -> None:
        if self.manual or self.locked:
            return
        if now - self.opened < 3.0 or now - self.last_step < 2.5:
            return
        self.last_step = now
        samples = self.meas_samples
        speeds = self.recent_hz
        self.meas_samples = 0
        self.recent_hz = []
        if samples >= 12:
            self.locked = True
            self.status = f"distancias a PWM {self.pwm}"
            self.labels["mode"].configure(text=self.status, fg=TEXT)
            print(f"fijado PWM {self.pwm} muestras {samples}", flush=True)
            return
        if not speeds:
            self.status = "sin tramas"
            self.labels["mode"].configure(text=self.status, fg=AMBER)
            print(self.status, flush=True)
            return
        med = statistics.median(speeds)
        if self.med_before is not None and abs(med - self.med_before) < 0.4:
            self.stuck += 1
        else:
            self.stuck = 0
        self.med_before = med
        if med >= 8.0 and self.stuck >= 1:
            self.manual = True
            self.status = (
                f"{med:.1f} Hz fijos. El PWM no cambia el giro.\n"
                "Baja la tensión de M+ hasta que baje de unos 8 Hz."
            )
            self.labels["mode"].configure(text=self.status, fg=AMBER)
            print(self.status.replace("\n", " "), flush=True)
            return
        if med >= 8.0:
            self.hold = 0
            nxt = max(PWM_MIN, self.pwm - 10)
            self.status = f"{med:.1f} Hz, solo velocidad, probando PWM {nxt}"
        elif med <= 4.5:
            self.hold = 0
            nxt = min(PWM_MAX, self.pwm + 5)
            self.status = f"{med:.1f} Hz, solo velocidad, subiendo"
        else:
            nxt = self.pwm
            self.hold += 1
            if self.hold >= 3:
                self.manual = True
                self.status = (
                    f"{med:.1f} Hz estables.\n"
                    "Solo llegan tramas de velocidad, ninguna de distancia."
                )
                self.labels["mode"].configure(text=self.status, fg=AMBER)
                print(self.status.replace("\n", " "), flush=True)
                return
            self.status = f"{med:.1f} Hz, sin distancias"
        self.labels["mode"].configure(text=self.status, fg=AMBER)
        print(self.status, flush=True)
        if nxt != self.pwm:
            self.set_pwm(nxt)

    def draw(self, now: float) -> None:
        c = self.canvas
        c.delete("all")
        cx, cy, radius = 360, 360, 300
        c.create_oval(cx - radius - 18, cy - radius - 18, cx + radius + 18, cy + radius + 18,
                      outline=RING, width=8)
        c.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill=SCOPE, outline=RING, width=2)
        rings = 4
        for i in range(1, rings + 1):
            r = radius * i / rings
            c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=RING)
            meters = self.span_m * i / rings
            c.create_text(cx + 8, cy - r + 12, text=f"{meters:.0f} m", fill=DIM, anchor="w",
                          font=("DejaVu Sans", 9))
        for deg in range(0, 360, 30):
            x0, y0 = self._xy(cx, cy, radius - 8, deg)
            x1, y1 = self._xy(cx, cy, radius, deg)
            c.create_line(x0, y0, x1, y1, fill=TICK)
            lx, ly = self._xy(cx, cy, radius + 16, deg)
            c.create_text(lx, ly, text=str(deg), fill=DIM, font=("DejaVu Sans", 9))
        # Estela del barrido.
        for k in range(18, 0, -1):
            ang = (self.bearing - k * 2.2 + self.view_deg) % 360.0
            x, y = self._xy(cx, cy, radius - 4, ang)
            shade = BLIP_COLORS[min(5, k // 3)]
            c.create_line(cx, cy, x, y, fill=shade)
        x, y = self._xy(cx, cy, radius - 4, (self.bearing + self.view_deg) % 360.0)
        c.create_line(cx, cy, x, y, fill=SWEEP, width=2)

        fresh: list[tuple[float, float, float]] = []
        plotted: list[float] = []
        for ang, meters, born in self.points:
            age = now - born
            if age > 1.8:
                continue
            fresh.append((ang, meters, born))
            if meters <= 0 or meters > self.span_m * 1.05:
                continue
            plotted.append(meters)
            frac = meters / self.span_m
            px, py = self._xy(cx, cy, radius * frac, (ang + self.view_deg) % 360.0)
            tone = BLIP_COLORS[min(5, int((1.8 - age) / 1.8 * 5))]
            c.create_oval(px - 2.5, py - 2.5, px + 2.5, py + 2.5, fill=tone, outline=tone)
        self.points = fresh
        if len(plotted) >= 30:
            hi = sorted(plotted)[int(len(plotted) * 0.95)]
            for span in (2.0, 4.0, 6.0, 8.0, 12.0):
                if hi <= span * 0.8:
                    self.span_m = span
                    break
            else:
                self.span_m = 12.0
        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=SWEEP, outline=SWEEP)
        self.labels["hz"].configure(text=f"{self.hz:.2f} / {self.target:.2f} Hz")
        if self.meas_frames:
            self.labels["hit"].configure(text=f"{self.meas_frames} tramas con distancia")
        else:
            self.labels["hit"].configure(text=f"{self.speed_frames} tramas de velocidad")

    @staticmethod
    def _xy(cx: float, cy: float, r: float, deg: float) -> tuple[float, float]:
        a = math.radians(deg - 90.0)
        return cx + r * math.cos(a), cy + r * math.sin(a)

    def tick(self) -> None:
        now = time.monotonic()
        try:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting or 1)
        except Exception as exc:
            if getattr(self, "_last_err", None) != str(exc):
                print(f"serial: {exc}", flush=True)
                self._last_err = str(exc)
            chunk = b""
        angled = False
        if chunk:
            self.log.write(chunk)
            self.note_text(chunk)
            for frame in self.parser.feed(chunk):
                if frame["dtype"] == DTYPE_MEAS and frame["samples"]:
                    angled = True
                self.handle(frame)
        if self.hz > 0 and not angled:
            dt = now - self.last_draw
            self.bearing = (self.bearing + self.hz * 360.0 * dt) % 360.0
        self.last_draw = now
        self.draw(now)
        self.root.after(33, self.tick)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="PPI del Delta-SC-02")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    Sonar(args.port, args.baud).run()


if __name__ == "__main__":
    main()
