import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from bender_input.joystick_reader import JoystickReader

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_pub')
        self.left_publisher_ = self.create_publisher(Float32, 'bender/left_speed', 10)
        self.right_publisher_ = self.create_publisher(Float32, 'bender/right_speed', 10)

        # Try to setup the joystick
        try:
            self.jr = JoystickReader()
        except:
            print("No compatible joysticks attached")
            exit(1)

        # Start polling the joystick in a seperate thread
        self.should_run = True
        self.poll_thread = threading.Thread(target=self.poll_controller)
        self.poll_thread.start()

    def publish_speeds(self, left_speed: float, right_speed: float):
        left_msg = Float32()
        right_msg = Float32()

        left_msg.data = left_speed
        right_msg.data = right_speed

        self.left_publisher_.publish(left_speed)
        self.right_publisher_.publish(right_speed)

        self.get_logger().info(F"Publishing Speeds: L: [{left_speed}] R: [{right_speed}]")
    
    def poll_controller(self):
        while self.should_run:
            sticks = self.jr.get_thumbsticks()
            left = -sticks['left_y']
            right = -sticks['right_y']
            print("Left Speed:", left)
            print("Right Speed:", right)
            self.publish_speeds(left, right)
            


def main(args=None):
    rclpy.init(args=args)
    
    tpub = TeleopPublisher()

    print("Starting node...")

    rclpy.spin(tpub)

    tpub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
