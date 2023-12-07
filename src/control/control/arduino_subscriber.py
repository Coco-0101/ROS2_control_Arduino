import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import String ,Int8
import serial

serial_comm_1 = serial.Serial()
serial_comm_2 = serial.Serial()

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(Int8, "controller_signal", self.callback, 10)
        self.get_logger().info("Subscriber is listening...")

        # Declare parameters for serial ports and baud rates
        self.declare_parameter("serial_port1", "/dev/ttyACM0")
        self.declare_parameter("baud_rate1", 115200)
        self.declare_parameter("subscribe_to1", "arduino_command1")
        self.declare_parameter("serial_port2", "/dev/ttyACM1")
        self.declare_parameter("baud_rate2", 115200)
        self.declare_parameter("subscribe_to2", "arduino_command2")

        sp1, br1, sub1, sp2, br2, sub2 = self.read_parameters()

        # Connect to given ports at given baud rates
        self.connect_serial_port_1(serial_port=sp1, baud_rate=br1)
        self.connect_serial_port_2(serial_port=sp2, baud_rate=br2)

        time.sleep(0.1) # Wait for connection.

        self.data_publisher_1 = self.create_publisher(String, "arduino_command1", 10) # Start publising serial port data.
        self.data_publisher_2 = self.create_publisher(String, "arduino_command2", 10) # Start publising serial port data.

        self.get_logger().info("Serial communication has started.")
        self.timer_ = self.create_timer(0.01, self.read_serial_data)

    def read_parameters(self):
        serial_port_1 = self.get_parameter("serial_port1").get_parameter_value().string_value
        baud_rate_1 = self.get_parameter("baud_rate1").get_parameter_value().integer_value
        subscribe_1 = self.get_parameter("subscribe_to1").get_parameter_value().string_value

        serial_port_2 = self.get_parameter("serial_port2").get_parameter_value().string_value
        baud_rate_2 = self.get_parameter("baud_rate2").get_parameter_value().integer_value
        subscribe_2 = self.get_parameter("subscribe_to2").get_parameter_value().string_value

        self.get_logger().info("Port: " + serial_port_1 + " Baud Rate: " + str(baud_rate_1) + " Subscribed To: " + subscribe_1)
        self.get_logger().info("Port: " + serial_port_2 + " Baud Rate: " + str(baud_rate_2) + " Subscribed To: " + subscribe_2)
        

        return serial_port_1, baud_rate_1, subscribe_1, serial_port_2, baud_rate_2, subscribe_2

    def connect_serial_port_1(self, serial_port, baud_rate):
        serial_comm_1.port = serial_port
        serial_comm_1.baudrate = baud_rate
        serial_comm_1.timeout = 0.05
        serial_comm_1.open()

    def connect_serial_port_2(self, serial_port, baud_rate):
        serial_comm_2.port = serial_port
        serial_comm_2.baudrate = baud_rate
        serial_comm_2.timeout = 0.05
        serial_comm_2.open()

    def read_serial_data(self):
        try:
            msg = String()
            msg.data = serial_comm_1.readline().decode("utf-8").rstrip("\n").rstrip("\r")
            msg.data = serial_comm_2.readline().decode("utf-8").rstrip("\n").rstrip("\r")
            self.data_publisher_1.publish(msg)
            self.data_publisher_2.publish(msg)
        except Exception as e:
            self.get_logger().warn("Serial communication error. -Reading")
            self.get_logger().warn(repr(e))

    def callback(self, msg):
        signal_value = str(msg.data)
        serial_comm_1.write(signal_value.encode("utf-8"))
        serial_comm_2.write(signal_value.encode("utf-8"))
        self.get_logger().info("Received signal: {0}".format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
