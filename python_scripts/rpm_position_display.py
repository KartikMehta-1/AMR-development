import serial
import re
import matplotlib.pyplot as plt
from collections import deque
import time

# =========================
# USER CONFIGURATION
# =========================
PORT = "COM4"          # <-- change this to your STM32's COM port
BAUD = 115200
MAX_POINTS = 200        # how many samples to show in live window
REFRESH_INTERVAL = 0.1  # seconds between graph updates

# =========================
# INITIAL SETUP
# =========================
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # allow time for STM32 reset

# Use deque for efficient sliding window
positions = deque(maxlen=MAX_POINTS)
rpms = deque(maxlen=MAX_POINTS)
times = deque(maxlen=MAX_POINTS)

start_time = time.time()

plt.ion()
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

pos_line, = ax1.plot([], [], 'b-', label='Position (counts)')
rpm_line, = ax2.plot([], [], 'r-', label='RPM')

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Position (counts)", color='b')
ax2.set_ylabel("RPM", color='r')
ax1.grid(True)

pattern = re.compile(r"Pos:\s*[-+]?\d+\s*cnt, d=[-+]?\d+, RPM=([-+]?[0-9]*\.?[0-9]+)")

print("Reading from serial... Press Ctrl+C to stop.\n")

try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line.startswith("Pos:"):
            # Example line: Pos: 12345 cnt, d=-20, RPM=950.00
            match = re.search(r"Pos:\s*(-?\d+)\s*cnt,\s*d=(-?\d+),\s*RPM=([-+]?\d*\.?\d+)", line)
            if match:
                pos = int(match.group(1))
                rpm = float(match.group(3))
                t = time.time() - start_time

                positions.append(pos)
                rpms.append(rpm)
                times.append(t)

                pos_line.set_data(times, positions)
                rpm_line.set_data(times, rpms)

                ax1.relim()
                ax1.autoscale_view()
                ax2.relim()
                ax2.autoscale_view()

                plt.pause(REFRESH_INTERVAL)
        else:
            # Optional: print non-matching lines for debug
            pass

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    ser.close()
    plt.ioff()
    plt.show()