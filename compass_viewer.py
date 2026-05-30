#!/usr/bin/env python3
"""
SensorDuck compass viewer
Reads JSON telemetry from the KitProg3 USB-UART and shows:
  - Live compass needle (left)
  - Heading + turn rate over time (right, dual y-axis)

Turn rate (°/s) is the first derivative of heading — it tells you how fast
the duck is *rotating*, not how fast it is moving through the water.
Positive = clockwise (heading increasing), negative = counter-clockwise.

Note: the gyro.z value in the JSON is the same quantity measured directly
by the IMU gyroscope. The computed derivative will be noisier (magnetometer
jitter) but serves as a useful cross-check.

Usage:
    python3 compass_viewer.py                      # auto-detect KitProg3 port
    python3 compass_viewer.py --port /dev/ttyACM0
    python3 compass_viewer.py --port COM3          # Windows
    python3 compass_viewer.py --baud 115200
"""

import argparse
import json
import threading
import sys
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BAUD_DEFAULT = 115200
MAX_HISTORY  = 300    # samples kept in the time plot
KITPROG3_VID = 0x04B4  # Cypress/Infineon USB vendor ID

# ---------------------------------------------------------------------------
# Shared state — written by reader thread, read by animation callback
# ---------------------------------------------------------------------------
_lock        = threading.Lock()
_latest_hdg  = None   # degrees (0–360)
_latest_rate = None   # °/s  (first derivative of heading, wrap-corrected)
# Each entry: (timestamp_s: float, heading_deg: float, turn_rate_dps: float)
_history     = deque(maxlen=MAX_HISTORY)

# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
_serial_buf = ""

def _reader_thread(port: serial.Serial):
    global _serial_buf, _latest_hdg, _latest_rate
    prev_t   = None
    prev_hdg = None

    while True:
        try:
            chunk = port.read(port.in_waiting or 1).decode("utf-8", errors="replace")
        except serial.SerialException:
            break

        _serial_buf += chunk
        while "\n" in _serial_buf:
            line, _serial_buf = _serial_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                doc = json.loads(line)
                if not doc.get("ok", {}).get("mag", False):
                    continue

                hdg = float(doc["mag"]["hdg"])
                t   = doc.get("t", 0) / 1000.0  # ms → s

                # First derivative of heading, corrected for the 0°/360° wrap.
                # ((delta + 180) % 360) - 180 maps any angle difference into [-180, +180],
                # so a jump from 355° to 5° gives +10°/s, not -350°/s.
                rate = 0.0
                if prev_t is not None:
                    dt = t - prev_t
                    if dt > 0:
                        dh   = ((hdg - prev_hdg + 180) % 360) - 180
                        rate = dh / dt

                prev_t   = t
                prev_hdg = hdg

                with _lock:
                    _latest_hdg  = hdg
                    _latest_rate = rate
                    _history.append((t, hdg, rate))

            except (json.JSONDecodeError, KeyError, ValueError):
                pass

# ---------------------------------------------------------------------------
# Auto-detect KitProg3 port
# ---------------------------------------------------------------------------
def _find_kitprog3() -> str | None:
    for p in serial.tools.list_ports.comports():
        if p.vid == KITPROG3_VID:
            return p.device
    return None

# ---------------------------------------------------------------------------
# Build the figure
# ---------------------------------------------------------------------------
COLOR_HDG  = "#4a9eff"   # blue  — heading
COLOR_RATE = "#ffaa44"   # amber — turn rate

def _build_figure():
    fig, (ax_compass, ax_plot) = plt.subplots(
        1, 2, figsize=(13, 5),
        gridspec_kw={"width_ratios": [1, 1.8]}
    )
    fig.patch.set_facecolor("#1a1a2e")
    fig.suptitle("SensorDuck — Magnetometer", color="white", fontsize=13)

    # ------------------------------------------------------------------ compass
    ax_compass.set_aspect("equal")
    ax_compass.set_xlim(-1.6, 1.6)
    ax_compass.set_ylim(-1.75, 1.6)
    ax_compass.set_facecolor("#1a1a2e")
    ax_compass.axis("off")

    ax_compass.add_patch(plt.Circle((0, 0), 1.1, fill=False,
                                    color=COLOR_HDG, linewidth=1.5))

    for deg in range(0, 360, 10):
        r   = np.radians(90 - deg)
        inn = 0.92 if deg % 30 != 0 else 0.85
        lw  = 0.8  if deg % 30 != 0 else 1.5
        ax_compass.plot([inn * np.cos(r), 1.1 * np.cos(r)],
                        [inn * np.sin(r), 1.1 * np.sin(r)],
                        color=COLOR_HDG, linewidth=lw, alpha=0.6)

    for label, cdeg, col in [("N", 0, "#ff4444"), ("E", 90, "white"),
                               ("S", 180, "white"), ("W", 270, "white")]:
        r = np.radians(90 - cdeg)
        ax_compass.text(1.38 * np.cos(r), 1.38 * np.sin(r), label,
                        ha="center", va="center",
                        fontsize=16, fontweight="bold", color=col)

    for cdeg in range(0, 360, 30):
        r = np.radians(90 - cdeg)
        ax_compass.text(0.75 * np.cos(r), 0.75 * np.sin(r), str(cdeg),
                        ha="center", va="center", fontsize=7, color="#888888")

    needle_tip,  = ax_compass.plot([], [], color="#ff4444", linewidth=4,
                                   solid_capstyle="round")
    needle_tail, = ax_compass.plot([], [], color="#aaaaaa", linewidth=2,
                                   solid_capstyle="round")
    ax_compass.add_patch(plt.Circle((0, 0), 0.04, color="white", zorder=5))

    hdg_text  = ax_compass.text(0, -1.38, "---°",
                                 ha="center", va="center",
                                 fontsize=20, fontweight="bold", color="white")
    rate_text = ax_compass.text(0, -1.60, "",
                                 ha="center", va="center",
                                 fontsize=11, color=COLOR_RATE)

    # ------------------------------------------------------------------ time plot
    ax_plot.set_facecolor("#1a1a2e")
    ax_plot.set_ylim(-5, 365)
    ax_plot.set_xlabel("time (s)", color="white")
    ax_plot.set_ylabel("heading (°)", color=COLOR_HDG)
    ax_plot.set_title("heading  &  turn rate", color="white", fontsize=10)
    ax_plot.tick_params(colors="white")
    ax_plot.tick_params(axis="y", colors=COLOR_HDG)
    for spine in ax_plot.spines.values():
        spine.set_edgecolor("#444444")

    for cdeg, label in [(0, "N"), (90, "E"), (180, "S"), (270, "W"), (360, "N")]:
        ax_plot.axhline(cdeg, color="#222244", linewidth=0.8, linestyle="--")
        ax_plot.text(0, cdeg + 5, label, color="#444477", fontsize=8)

    ax_plot.set_yticks([0, 90, 180, 270, 360])
    ax_plot.set_yticklabels(["0° N", "90° E", "180° S", "270° W", "360°"],
                             color=COLOR_HDG)

    hdg_line, = ax_plot.plot([], [], color=COLOR_HDG, linewidth=1.8,
                              label="heading (°)")

    # Twin axis for turn rate
    ax_rate = ax_plot.twinx()
    ax_rate.set_facecolor("#1a1a2e")
    ax_rate.set_ylabel("turn rate (°/s)", color=COLOR_RATE)
    ax_rate.tick_params(axis="y", colors=COLOR_RATE)
    ax_rate.axhline(0, color="#443300", linewidth=0.8, linestyle=":")
    ax_rate.set_ylim(-90, 90)  # will auto-expand if needed

    rate_line, = ax_rate.plot([], [], color=COLOR_RATE, linewidth=1.2,
                               alpha=0.85, linestyle="--", label="turn rate (°/s)")

    # Combined legend
    lines  = [hdg_line, rate_line]
    labels = [l.get_label() for l in lines]
    ax_plot.legend(lines, labels, loc="upper left",
                   facecolor="#2a2a3e", edgecolor="#444444",
                   labelcolor="white", fontsize=8)

    plt.tight_layout()

    return (fig,
            needle_tip, needle_tail, hdg_text, rate_text,
            hdg_line, rate_line, ax_plot, ax_rate)

# ---------------------------------------------------------------------------
# Animation update
# ---------------------------------------------------------------------------
def _make_update(needle_tip, needle_tail, hdg_text, rate_text,
                 hdg_line, rate_line, ax_plot, ax_rate):

    def update(_frame):
        with _lock:
            hdg  = _latest_hdg
            rate = _latest_rate
            snap = list(_history)

        if hdg is not None:
            r = np.radians(90 - hdg)
            needle_tip.set_data([0,  0.85 * np.cos(r)], [0,  0.85 * np.sin(r)])
            needle_tail.set_data([0, -0.30 * np.cos(r)], [0, -0.30 * np.sin(r)])
            hdg_text.set_text(f"{hdg:.1f}°")

        if rate is not None:
            arrow = "→" if rate > 0.5 else ("←" if rate < -0.5 else "·")
            rate_text.set_text(f"{rate:+.1f} °/s  {arrow}")

        if snap:
            times = [p[0] for p in snap]
            heads = [p[1] for p in snap]
            rates = [p[2] for p in snap]

            t_min = max(times[-1] - 30, times[0])
            ax_plot.set_xlim(t_min, times[-1] + 0.5)
            hdg_line.set_data(times, heads)
            rate_line.set_data(times, rates)

            # Auto-scale rate axis with a minimum window of ±10 °/s
            visible = [r for t, _, r in snap if t >= t_min]
            if visible:
                lo = min(min(visible), -10)
                hi = max(max(visible),  10)
                pad = (hi - lo) * 0.1
                ax_rate.set_ylim(lo - pad, hi + pad)

        return needle_tip, needle_tail, hdg_text, rate_text, hdg_line, rate_line

    return update

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="SensorDuck compass viewer")
    parser.add_argument("--port", help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    args = parser.parse_args()

    port_name = args.port
    if not port_name:
        port_name = _find_kitprog3()
        if port_name:
            print(f"Auto-detected KitProg3 on {port_name}")
        else:
            print("KitProg3 not found — pass --port explicitly.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}  {p.description}")
            sys.exit(1)

    try:
        ser = serial.Serial(port_name, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open {port_name}: {e}")
        sys.exit(1)

    print(f"Reading from {port_name} @ {args.baud} baud — close the window to quit")

    threading.Thread(target=_reader_thread, args=(ser,), daemon=True).start()

    (fig,
     needle_tip, needle_tail, hdg_text, rate_text,
     hdg_line, rate_line, ax_plot, ax_rate) = _build_figure()

    update_fn = _make_update(needle_tip, needle_tail, hdg_text, rate_text,
                             hdg_line, rate_line, ax_plot, ax_rate)

    _ani = animation.FuncAnimation(fig, update_fn, interval=100, blit=True)  # noqa: F841

    plt.show()
    ser.close()

if __name__ == "__main__":
    main()
