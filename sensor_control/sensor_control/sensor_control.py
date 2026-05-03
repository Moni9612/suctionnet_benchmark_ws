import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import csv
import os
import matplotlib.pyplot as plt

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600
LOG_DIR = '/home/moniesha/Evaluation metrics/Stability/suctionnet'
os.makedirs(LOG_DIR, exist_ok=True)

class RelaySensorFSRLogger(Node):
    def __init__(self):
        super().__init__('relay_sensor_fsr_logger')

        self.create_subscription(String, '/relay_control', self.relay_callback, 1)
        self.create_subscription(String, '/sensor_control', self.sensor_callback, 1)

        self.connect_serial()

        self.logging = False
        self.start_time = None
        self.timestamps = []
        self.fsr_values = []

        self.create_timer(0.01, self.read_serial_loop)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"✅ Serial connection established on {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial connection failed: {e}")
            self.ser = None

    def relay_callback(self, msg):
        if self.ser:
            command = msg.data.strip().upper()
            if command == "ON":
                self.ser.write(b"ON\n")
                self.get_logger().info("➡️ Sent 'ON' (Relay Control)")
            elif command == "OFF":
                self.ser.write(b"OFF\n")
                self.get_logger().info("➡️ Sent 'OFF' (Relay Control)")

    def sensor_callback(self, msg):
        if self.ser:
            command = msg.data.strip().upper()
            if command == "SON":
                self.ser.write(b"SON\n")
                self.get_logger().info("➡️ Sent 'SON' (Sensor Control)")
            elif command == "SOFF":
                self.ser.write(b"SOFF\n")
                self.get_logger().info("➡️ Sent 'SOFF' (Sensor Control)")

    def read_serial_loop(self):
        try:
            if not self.ser:
                return

            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode().strip()
            if not line:
                return

            if line == "START_LOGGING" and not self.logging:
                self.logging = True
                self.start_time = time.time()
                self.timestamps = []
                self.fsr_values = []
                self.get_logger().info("🟢 Logging started.")

            elif line == "STOP_LOGGING" and self.logging:
                self.logging = False
                self.get_logger().info("🔴 Logging stopped.")
                self.process_and_save_fsr_data()

            elif line.startswith("FSR:") and self.logging:
                try:
                    parts = line.split(',')
                    fsr = float(parts[0].split(':')[1].strip())
                    if fsr > 100:
                        self.get_logger().warn(f"⚠️ Ignored FSR value > 100: {fsr}")
                        return
                    now = time.time()
                    t_rel = now - self.start_time
                    self.timestamps.append(t_rel)
                    self.fsr_values.append(fsr)
                    self.get_logger().info(f"📈 FSR: {fsr}, Time: {t_rel:.3f}s")
                except Exception:
                    self.get_logger().warn(f"⚠️ Parse error: {line}")

        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"❌ Serial read error: {e}")
            try:
                self.ser.close()
            except:
                pass
            self.ser = None

            # ✅ Attempt reconnect with retry loop
            self.get_logger().info("🔄 Attempting to reconnect to serial...")

            for attempt in range(10):  # Try up to 5 seconds
                try:
                    time.sleep(0.5)
                    self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                    time.sleep(2)
                    self.get_logger().info(f"✅ Reconnected successfully on attempt {attempt + 1}")
                    return
                except serial.SerialException:
                    continue  # Try again

            self.get_logger().error("❌ Failed to reconnect after multiple attempts.")
            self.ser = None

    def process_and_save_fsr_data(self):
        if not self.timestamps:
            self.get_logger().warn("⚠️ No data recorded.")
            return

        t4 = self.timestamps[-1]
        t3_estimate = t4 - 1.8
        t3_index = min(range(len(self.timestamps)), key=lambda i: abs(self.timestamps[i] - t3_estimate))
        t3 = self.timestamps[t3_index]
        fsr_at_t3 = self.fsr_values[t3_index]

        t1 = t2 = fsr_at_t1 = fsr_at_t2 = None
        for i in range(1, len(self.fsr_values)):
            drop = self.fsr_values[i-1] - self.fsr_values[i]
            if drop > 8 and t1 is None:
                t1 = self.timestamps[i]
                fsr_at_t1 = self.fsr_values[i]
                continue
            if t1 is not None:
                rise = self.fsr_values[i] - self.fsr_values[i-1]
                if rise > 8 and t2 is None:
                    t2 = self.timestamps[i]
                    fsr_at_t2 = self.fsr_values[i]
                    break

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(LOG_DIR, f'fsr_data_{timestamp_str}.csv')
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time (s)', 'FSR Reading'])
            if t1 is not None:
                writer.writerow([round(t1, 3), fsr_at_t1])
            if t2 is not None:
                writer.writerow([round(t2, 3), fsr_at_t2])
            writer.writerow([round(t3, 3), fsr_at_t3])
        self.get_logger().info(f"📄 CSV saved to: {csv_file}")

        plot_file = os.path.join(LOG_DIR, f'fsr_plot_{timestamp_str}.png')
        plt.figure()
        plt.plot(self.timestamps, self.fsr_values, label='Pressure Reading', color='blue')
        if t1 is not None:
            plt.plot(t1, fsr_at_t1, 'go', label='Grasp point (t1)')
        if t2 is not None:
            plt.plot(t2, fsr_at_t2, 'ro', label='Drop point (t2)')
        plt.plot(t3, fsr_at_t3, 'bo', label='End point (t3)')
        plt.xlabel('Time (s)')
        plt.ylabel('Pressure')
        plt.title('Pressure vs Time')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_file)
        plt.close()
        self.get_logger().info(f"📊 Plot saved to: {plot_file}")

def main(args=None):
    rclpy.init(args=args)
    node = RelaySensorFSRLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

