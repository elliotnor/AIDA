import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from aida_interfaces.msg import Joystick


class MinimalSubscriber(Node):
    joystickDirection = ""

    def __init__(self):
        super().__init__('joystick_converter')
        self.subscription = self.create_subscription(Joystick, 'joystick/pos', self.converter, 10)
        self.publisher = self.create_publisher(String, 'joystick/direction', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def converter(self, msg):
        x = msg.x
        y = -(msg.y)
        square = 1 / math.sqrt(2)
        
        if(y<=1) or (y> square):
            self.joystickDirection = "forward"
        elif(y >= -1) or (y < -square):
            self.joystickDirection = "backward"
        
        elif(x <= 1) or (x > square):
            self.joystickDirection = "rigth"
        
        elif(x <= -1) or (x < square):
            self.joystickDirection = "left"

        msg = String()
        msg.data = self.joystickDirection
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

