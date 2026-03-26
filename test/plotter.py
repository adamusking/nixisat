"""
MPU9250 live plotter
────────────────────
Reads CSV from the ESP32 serial port and plots all channels live.

Requirements:
    pip install pyserial matplotlib

Usage:
    python plotter.py                      # auto-detects port
    python plotter.py --port COM5          # Windows
    python plotter.py --port /dev/ttyUSB0  # Linux

Fix vs previous version:
    The script no longer waits for the CSV header line.
    Instead it resets the ESP32 via DTR when the port opens
    (same as PlatformIO monitor does), so the header arrives fresh.
    It also accepts data rows directly if reset is not needed.
"""

import argparse
import collections
import sys
import threading
import time

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Config ────────────────────────────────────────────────────────────────────
BAUD        = 115200
WINDOW      = 500       # samples in rolling window (~10 s at 50 Hz)
INTERVAL_MS = 40        # plot refresh rate (25 fps)

COLS = ["ts_ms",
        "aX",  "aY",  "aZ",
        "gX",  "gY",  "gZ",
        "mX",  "mY",  "mZ",
        "hdgMag", "hdgGyro"]
N_COLS = len(COLS)

# ── Auto-detect serial port ───────────────────────────────────────────────────
def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description + (p.manufacturer or "")).lower()
        if any(k in desc for k in ("cp210", "ch340", "esp32", "usb serial", "uart")):
            return p.device
    return ports[0].device if ports else None

# ── Shared rolling buffers ────────────────────────────────────────────────────
data = {col: collections.deque(maxlen=WINDOW) for col in COLS}
lock = threading.Lock()
status_msg = ["Waiting for data..."]

def serial_reader(port, baud):
    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 2
        ser.dtr      = False   # hold reset line LOW before opening
        ser.open()
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        sys.exit(1)

    print(f"Opened {port} @ {baud} baud")

    # Pulse DTR to reset the ESP32 — it will reboot and send the header fresh
    time.sleep(0.05)
    ser.dtr = True
    print("Reset pulse sent — waiting for ESP32 to boot...")
    time.sleep(1.5)    # give the ESP32 time to boot and send its first lines
    ser.reset_input_buffer()

    status_msg[0] = f"Connected on {port} — receiving data"

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            continue

        if not raw:
            continue

        # Print comment / status lines to console, ignore for plotting
        if raw.startswith("#"):
            print(raw)
            continue

        # Skip the CSV header row — we already know the column order
        if raw.startswith("ts_ms"):
            print("CSV header received.")
            continue

        # Parse data row
        parts = raw.split(",")
        if len(parts) != N_COLS:
            continue
        try:
            values = [float(p) for p in parts]
        except ValueError:
            continue

        with lock:
            for col, val in zip(COLS, values):
                data[col].append(val)


# ── Plot layout ───────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(12, 9))
fig.suptitle("MPU9250 Live Data", fontsize=13, fontweight="bold")
plt.subplots_adjust(hspace=0.55, left=0.07, right=0.97, top=0.93, bottom=0.05)

ax_acc, ax_gyro, ax_mag, ax_hdg = axes

lax, = ax_acc.plot([], [], color="#e74c3c", lw=1.2, label="aX")
lay, = ax_acc.plot([], [], color="#2ecc71", lw=1.2, label="aY")
laz, = ax_acc.plot([], [], color="#3498db", lw=1.2, label="aZ")
ax_acc.set_title("Accelerometer", fontsize=10)
ax_acc.set_ylabel("g")
ax_acc.legend(loc="upper right", fontsize=8, ncol=3)
ax_acc.set_xlim(0, WINDOW)

lgx, = ax_gyro.plot([], [], color="#e74c3c", lw=1.2, label="gX")
lgy, = ax_gyro.plot([], [], color="#2ecc71", lw=1.2, label="gY")
lgz, = ax_gyro.plot([], [], color="#3498db", lw=1.2, label="gZ")
ax_gyro.set_title("Gyroscope", fontsize=10)
ax_gyro.set_ylabel("deg/s")
ax_gyro.legend(loc="upper right", fontsize=8, ncol=3)
ax_gyro.set_xlim(0, WINDOW)

lmx, = ax_mag.plot([], [], color="#e74c3c", lw=1.2, label="mX")
lmy, = ax_mag.plot([], [], color="#2ecc71", lw=1.2, label="mY")
lmz, = ax_mag.plot([], [], color="#3498db", lw=1.2, label="mZ")
ax_mag.set_title("Magnetometer", fontsize=10)
ax_mag.set_ylabel("uT")
ax_mag.legend(loc="upper right", fontsize=8, ncol=3)
ax_mag.set_xlim(0, WINDOW)

lhm, = ax_hdg.plot([], [], color="#9b59b6", lw=1.5, label="Mag heading")
lhg, = ax_hdg.plot([], [], color="#e67e22", lw=1.5, label="Gyro heading", linestyle="--")
ax_hdg.set_title("Heading comparison  (gyro drift visible over time)", fontsize=10)
ax_hdg.set_ylabel("deg")
ax_hdg.set_ylim(-10, 370)
ax_hdg.axhline(0,   color="gray", lw=0.5, linestyle=":")
ax_hdg.axhline(360, color="gray", lw=0.5, linestyle=":")
ax_hdg.legend(loc="upper right", fontsize=8, ncol=2)
ax_hdg.set_xlim(0, WINDOW)

status_text = fig.text(0.5, 0.005, status_msg[0],
                       ha="center", fontsize=8, color="gray")

def animate(_frame):
    with lock:
        n = len(data["ts_ms"])
        if n < 2:
            return

        xs = list(range(n))

        def get(col):
            return list(data[col])

        a = get("aX") + get("aY") + get("aZ")
        ax_acc.set_ylim(min(a) - 0.2, max(a) + 0.2)
        lax.set_data(xs, get("aX"))
        lay.set_data(xs, get("aY"))
        laz.set_data(xs, get("aZ"))

        g = get("gX") + get("gY") + get("gZ")
        ax_gyro.set_ylim(min(g) - 5, max(g) + 5)
        lgx.set_data(xs, get("gX"))
        lgy.set_data(xs, get("gY"))
        lgz.set_data(xs, get("gZ"))

        m = get("mX") + get("mY") + get("mZ")
        ax_mag.set_ylim(min(m) - 5, max(m) + 5)
        lmx.set_data(xs, get("mX"))
        lmy.set_data(xs, get("mY"))
        lmz.set_data(xs, get("mZ"))

        lhm.set_data(xs, get("hdgMag"))
        lhg.set_data(xs, get("hdgGyro"))

    status_text.set_text(status_msg[0])


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="MPU9250 live plotter")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--no-reset", action="store_true",
                        help="Skip the DTR reset pulse (if your board resets unexpectedly)")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("No serial port found. Connect the ESP32 or use --port.")
        sys.exit(1)

    print(f"Using port: {port}")

    t = threading.Thread(target=serial_reader, args=(port, args.baud), daemon=True)
    t.start()

    ani = animation.FuncAnimation(fig, animate,
                                  interval=INTERVAL_MS,
                                  blit=False,
                                  cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    main()