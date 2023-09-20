import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import serial

serial_comm = serial.Serial()

class CommunicationNode(Node):

    def __init__(self):
        super().__init__("serial_comm")

        # Declare parameters for serial port and baud rate
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("subscribe_to", "arduino_command")

        sp, br, sub = self.read_parameters() # Read parameters which are coming from terminal or launch file.

        self.connect_serial_port(serial_port=sp, baud_rate=br) # Connect to given port at given baud rate.

        time.sleep(0.5) # Wait for connection.

        self.data_publisher_ = self.create_publisher(String, "arduino_command", 10) # Start publising serial port data.

        if sub != "None":
            self.data_subscriber_ = self.create_subscription(String, sub, self.write_serial_data, 10)

        self.get_logger().info("Serial communication has started.")
        self.timer_ = self.create_timer(0.1, self.read_serial_data)

    def read_parameters(self):
        serial_port_ = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate_ = self.get_parameter("baud_rate").get_parameter_value().integer_value
        subscribe_ = self.get_parameter("subscribe_to").get_parameter_value().string_value

        self.get_logger().info("Port: " + serial_port_ + " Baud Rate: " + str(baud_rate_) + " Subscribed To: " + subscribe_)

        return serial_port_, baud_rate_, subscribe_

    def connect_serial_port(self, serial_port, baud_rate):
        serial_comm.port = serial_port
        serial_comm.baudrate = baud_rate
        serial_comm.timeout = 1
        serial_comm.open()

    def read_serial_data(self):
        try:
            msg = String()
            msg.data = serial_comm.readline().decode("utf-8").rstrip("\n").rstrip("\r")
            self.data_publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn("Serial communication error. -Reading")
            self.get_logger().warn(repr(e))

    def write_serial_data(self, msg: String):
            try:
                msg = input("Enter a command (0 ~ 8) to control Arduino: ")
                if msg in ('0', '1', '2', '3', '4', '5', '6', '7', '8'):
                    serial_comm.write(msg.encode("utf-8"))
                    # self.get_logger().info(f"Sent command to Arduino: {msg}")
                else:
                    self.get_logger().info("Invalid command. Please enter command in 0 ~ 8.")
            except Exception as e:
                self.get_logger().warn("Serial communication error. -Writing")
                self.get_logger().warn(repr(e))

    def cache_data(self, data: String):
        self.get_logger().info(str(data.data))
        
def main(args=None):
    rclpy.init(args=args)

    node = CommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()