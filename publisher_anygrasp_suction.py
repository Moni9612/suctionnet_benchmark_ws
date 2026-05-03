import serial
import time
import csv
import matplotlib.pyplot as plt
import os

# ✅ Fixed log directory
log_dir = '/home/moniesha/Evaluation metrics/Stability/suctionnet'
os.makedirs(log_dir, exist_ok=True)

# Replace with your port
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

logging = False
start_time = None
timestamps = []
fsr_values = []

print("✅ Listening for Arduino data...")

while True:
    try:
        line = ser.readline().decode().strip()
        
        if not line:
            continue

        # 🔵 Start logging if not already logging
        if line == "START_LOGGING" and not logging:
            print("🟢 Logging started.")
            logging = True
            start_time = time.time()
            timestamps = []
            fsr_values = []

        # 🔴 Stop logging and save session
        elif line == "STOP_LOGGING" and logging:
            print("🔴 Logging stopped.")
            logging = False

            t4 = time.time() - start_time
            t3 = t4 - 1.9

            t1 = None
            t2 = None
            fsr_at_t1 = None
            fsr_at_t2 = None

            # Modified grasp/drop detection logic
            for i in range(1, len(fsr_values)):
                drop = fsr_values[i-1] - fsr_values[i]

                # Detect grasp point (drop > 8)
                if drop > 8 and t1 is None:
                    t1 = timestamps[i]
                    fsr_at_t1 = fsr_values[i]
                    continue

                # After t1, detect drop point (rise > 8)
                if t1 is not None:
                    rise = fsr_values[i] - fsr_values[i-1]
                    if rise > 8 and t2 is None:
                        t2 = timestamps[i]
                        fsr_at_t2 = fsr_values[i]
                        break

            # Save to CSV
            timestamp_str = time.strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(log_dir, f'fsr_data_{timestamp_str}.csv')
            plot_file = os.path.join(log_dir, f'fsr_plot_{timestamp_str}.png')

            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time (s)', 'FSR Reading'])
                if t1 is not None:
                    writer.writerow([round(t1, 3), fsr_at_t1])
                if t2 is not None:
                    writer.writerow([round(t2, 3), fsr_at_t2])
                writer.writerow([round(t3, 3), 0])
            print(f"📄 CSV saved to: {csv_file}")

            # Plotting
            plt.figure()
            plt.plot(timestamps, fsr_values, label='FSR Reading', color='blue')
            if t1 is not None:
                plt.plot(t1, fsr_at_t1, 'go', label='Grasp point (t1)')
            if t2 is not None:
                plt.plot(t2, fsr_at_t2, 'ro', label='Drop point (t2)')
            plt.plot(t3, 0, 'bo', label='End point (t3)')

            plt.xlabel('Time (s)')
            plt.ylabel('Force')
            plt.title('Force vs Time')
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(plot_file)
            print(f"📊 Plot saved to: {plot_file}")
            plt.close()

        # 📥 Record FSR readings only when logging
        elif line.startswith("FSR:") and logging:
            try:
                parts = line.split(',')
                fsr = float(parts[0].split(':')[1].strip())

                if fsr > 100:
                    print(f"⚠️ Ignored FSR value > 100: {fsr}")
                    continue

                now = time.time()
                t_rel = now - start_time
                timestamps.append(t_rel)
                fsr_values.append(fsr)

                print(f"📈 FSR: {fsr}, Time: {t_rel:.3f}s")

            except Exception as e:
                print("⚠️ Parse error:", line)

    except KeyboardInterrupt:
        print("\n👋 Exiting on user interrupt.")
        break
    except Exception as e:
        print("❌ Error:", str(e))

