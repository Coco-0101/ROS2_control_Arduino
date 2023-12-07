import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from xbox360controller import Xbox360Controller


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


def main(args=None):
    rclpy.init(args=None)

    minimal_publisher = MinimalPublisher()

    def on_button_pressed(button):
        if button.name == "button_start":
            minimal_publisher.send_message(0, "Shutdown")
        elif button.name == "button_a":
            minimal_publisher.send_message(6, "Fire")
        elif button.name == "button_y":
            minimal_publisher.send_message(7, "Reload")    
        elif button.name == "button_trigger_l":
            minimal_publisher.send_message(8, "Turn left")
        elif button.name == "button_trigger_r":
            minimal_publisher.send_message(9, "Turn right")

    def on_axis_moved(axis):
        if axis.name == "axis_l":
            if 0.77 >= axis.x >= -0.77 and -0.77 >= axis.y >= -1:
                minimal_publisher.send_message(1, "Forward")
            elif 0.77 >= axis.x >= -0.77 and 1 >= axis.y >= 0.77:
                minimal_publisher.send_message(2, "Backward")
            elif -0.77 >= axis.x >= -1 and 0.77 >= axis.y >= -0.77:
                minimal_publisher.send_message(3, "Shift left")
            elif 1 >= axis.x >= 0.77 and 0.77 >= axis.y >= -0.77:
                minimal_publisher.send_message(4, "Shift right")

    def on_trigger_moved(trigger):
        if trigger.name == "trigger_l" or trigger.name == "trigger_r":
            if trigger.value == 1:
                minimal_publisher.send_message(5, "Stop")

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
    rclpy.shutdown()


if __name__ == "__main__":
    main()
