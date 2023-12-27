import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import String, Int8
import serial

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")

        self.get_logger().info("Subscriber is listening...")

        # Declare parameters for serial port and baud rate
        sp1 = "/dev/ttyACM1"
        sp2 = "/dev/ttyACM0"
        br = 115200

        # Create a serial_comm object for each instance
        self.serial_comm1 = serial.Serial()
        self.serial_comm2 = serial.Serial()

        self.subscription = self.create_subscription(Int8, "controller_signal", self.write_serial_data, 10)
        self.connect_serial_port(serial_port=sp1, baud_rate=br, serial_comm=self.serial_comm1)
        self.connect_serial_port(serial_port=sp2, baud_rate=br, serial_comm=self.serial_comm2)

        time.sleep(0.1)  # Wait for connection.

        self.get_logger().info("Serial communication has started.")

    def connect_serial_port(self, serial_port, baud_rate, serial_comm):
        serial_comm.port = serial_port
        serial_comm.baudrate = baud_rate
        serial_comm.timeout = 1
        if not serial_comm.is_open:
            serial_comm.open()

    def write_serial_data(self, msg: String):
        try:
            serial_data = str(msg.data)
            self.serial_comm1.write(serial_data.encode("utf-8"))
            self.serial_comm2.write(serial_data.encode("utf-8"))
            self.get_logger().info("Received signal: {0}".format(serial_data))
        except Exception as e:
            self.get_logger().warn("Serial communication error. -Writing")
            self.get_logger().warn(repr(e))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
