import threading
import math
from datetime import datetime

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
        
        # Create safety timer
        self.last_publish = datetime.now()
        self.safety_timer = self.create_timer(0.1, self._safety_routine)

        # Start polling the joystick in a seperate thread
        self.should_run = True
        self.poll_thread = threading.Thread(target=self.poll_controller)
        self.poll_thread.start()

    def publish_speeds(self, left_speed: float, right_speed: float):
        self.last_publish = datetime.now()
        left_msg = Float32()
        right_msg = Float32()

        left_msg.data = left_speed
        right_msg.data = right_speed

        self.left_publisher_.publish(left_msg)
        self.right_publisher_.publish(right_msg)

        self.get_logger().info(F"Publishing Speeds: L: [{left_speed}] R: [{right_speed}]")
    
    def poll_controller(self):
        while self.should_run:
            if self.jr.poll():
                sticks = self.jr.get_thumbsticks()
                left = self._round_2(self._clamp(-sticks['left_y'], -1.0, 1.0))
                right = self._round_2(self._clamp(-sticks['right_y'], -1.0, 1.0))
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
