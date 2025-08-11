import threading
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from bender_input.joystick_reader import JoystickReader

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_pub')
        self.twist_publisher_ = self.create_publisher(Twist, 'bender/twist', 10)

        # Try to setup the joystick
        try:
            self.jr = JoystickReader()
        except:
            print("No compatible joysticks attached")
            exit(1)
        
        # Create safety timer
        self.last_publish = datetime.now()
        self.safety_timer = self.create_timer(0.1, self._safety_routine)

        # Start polling the joystick in a seperate thread
        self.should_run = True
        self.poll_thread = threading.Thread(target=self.poll_controller)
        self.poll_thread.daemon = True  # Ensure thread exits when main program does
        self.poll_thread.start()

    def publish_speeds(self, left_stick_up: float, right_stick_side: float):
        self.last_publish = datetime.now()
        twist_msg = Twist()

        twist_msg.linear.x = left_stick_up
        twist_msg.angular.z = right_stick_side

        self.twist_publisher_.publish(twist_msg)

        self.get_logger().info(F"Publishing Twist: L: [{left_stick_up}] R: [{right_stick_side}]")

    def poll_controller(self):
        while self.should_run:
            if self.jr.poll():
                sticks = self.jr.get_thumbsticks()
                left = self._round_2(self._clamp(-sticks['left_y'], -1.0, 1.0))
                right = self._round_2(self._clamp(-sticks['right_x'], -1.0, 1.0))
                self.publish_speeds(float(left), float(right))
    
    def _safety_routine(self):
        if (datetime.now() - self.last_publish).total_seconds() > 5.0:
            self.publish_speeds(0.0, 0.0)
    
    def _clamp(self, value: float, min: float, max: float) -> float:
        if (value < min):
            return min
        elif (value > max):
            return max
        else:
            return value
    
    def _round_2(self, value: float):
        return math.ceil(value*100)/100
            


def main(args=None):
    rclpy.init(args=args)
    
    tpub = TeleopPublisher()

    print("Starting node...")

    rclpy.spin(tpub)

    tpub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
