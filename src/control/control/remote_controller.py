import time
import serial
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String ,Int8
from std_msgs.msg import Int8
from xbox360controller import Xbox360Controller

serial_comm = serial.Serial()

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Int8, "controller_signal", 10)
        self.signal = 0

    def send_message(self, signal, function):
        msg = Int8()
        msg.data = signal
        self.publisher_.publish(msg)
        self.get_logger().info("{0}".format(function))

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
        minimal_publisher = MinimalPublisher()
        def on_button_pressed(button):
            if button.name == "button_start":
                minimal_publisher.send_message(0, "Shutdown")
                msg = '0'
                serial_comm.write(msg.encode("utf-8"))
            elif button.name == "button_a":
                minimal_publisher.send_message(7, "Fire!!")
                msg = '7'
                serial_comm.write(msg.encode("utf-8"))
            elif button.name == "button_b":
                minimal_publisher.send_message(6, "Open the gate")   
                msg = '6' 
                serial_comm.write(msg.encode("utf-8"))
            elif button.name == "button_trigger_l" or button.name == "button_trigger_r":
                minimal_publisher.send_message(8, "Reload")
                msg = '8'
                serial_comm.write(msg.encode("utf-8"))

        def on_axis_moved(axis):
            if axis.name == "axis_l":
                if 0.77 >= axis.x >= -0.77 and -0.77 >= axis.y >= -1:
                    minimal_publisher.send_message(1, "Forward")
                    msg = '1'
                    serial_comm.write(msg.encode("utf-8"))
                elif 0.77 >= axis.x >= -0.77 and 1 >= axis.y >= 0.77:
                    minimal_publisher.send_message(2, "Backward")
                    msg = '2'
                    serial_comm.write(msg.encode("utf-8"))
                elif -0.77 >= axis.x >= -1 and 0.77 >= axis.y >= -0.77:
                    minimal_publisher.send_message(3, "Shift left")
                    msg = '3'
                    serial_comm.write(msg.encode("utf-8"))
                elif 1 >= axis.x >= 0.77 and 0.77 >= axis.y >= -0.77:
                    minimal_publisher.send_message(4, "Shift right")
                    msg = '4'
                    serial_comm.write(msg.encode("utf-8"))

        def on_trigger_moved(trigger):
            if trigger.name == "trigger_l" or trigger.name == "trigger_r":
                if trigger.value == 1:
                    minimal_publisher.send_message(5, "Stop")
                    msg = '5'
                    serial_comm.write(msg.encode("utf-8"))
        try:
            with Xbox360Controller(index=0, axis_threshold=0.2) as controller:
                # Button events
                for button in controller.buttons:
                    button.when_pressed = on_button_pressed
                    
                # Axis and trigger events
                for axis in controller.axes[0:3]:
                    axis.when_moved = on_axis_moved
                    
                for trigger in controller.axes[3:5]:
                    trigger.when_moved = on_trigger_moved

                signal.pause()

        except KeyboardInterrupt:
            pass

        minimal_publisher.destroy_node()


    def cache_data(self, data: String):
        self.get_logger().info(str(data.data))
        
def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()