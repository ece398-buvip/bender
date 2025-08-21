import time
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from bender_tf2.bender_tag_detector import BenderTagDetector
from bender_tf2.bender_tag_common import rotation_matrix_to_quaternion
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# Camera parameters - TODO: Move to a config file
cam_index = 0
fx = float(636.56831794)
fy = float(632.28979379)
cx = float(307.50223257)
cy = float(238.50969476)
tag_size = 0.16 # tag size is actually .16 m
tag_family = "tag36h11" # family of tags to detect

camera_params = tuple(
    [fx, fy, cx, cy]
)

class TagLocalizer(Node):

    def __init__(self, robot_name):
        super().__init__("tag_localizer")

        # Initialize tag detector
        self.detector = BenderTagDetector(cam_index, camera_params, tag_size)

        # Initialize tf2 publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        self.robot_name = robot_name

        # Setup tag detection thread
        self.should_run = True
        self.poll_thread = threading.Thread(target=self.poll_tags)
        self.poll_thread.daemon = True  # Ensure thread exits when main program does
        self.poll_thread.start()
    
    def poll_tags(self):
        while self.should_run:
            detections = self.detector.run_detect()

            if detections is not None and len(detections) > 0:
                # TODO: Do something with all the tags, like average our position or apply a filter
                t = TransformStamped()
                d = detections[0]  # Assuming we only care about the first detected tag
                # pose = d.pose_t
                # err = d.pose_err
                # print(F"Err: [{err}], X: [{pose[0]}], Y: [{pose[1]}], Z: [{pose[2]}]")

                # Set Header data
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'tag_2'
                t.child_frame_id = "bender_cam"

                # Solve for the camera's position in tag space
                # (-t)
                cam_pos = np.array([-float(d.pose_t[0]), -float(d.pose_t[1]), -float(d.pose_t[2])])

                # (R^(-1))
                cam_rot = d.pose_R.T

                # (-t)*(R^(-1)) where R is rotation matrix, and t is 
                #   translation to tag center
                cam_pos = cam_rot @ cam_pos

                t.transform.translation.x = float(cam_pos[0])
                t.transform.translation.y = float(cam_pos[1])
                t.transform.translation.z = float(cam_pos[2])
                rclpy.logging.get_logger("bender_tf2").debug(F"Publishing pose [{t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}]")
                rclpy.logging.get_logger("bender_tf2").debug(F"Rotation matrix: [{d.pose_R}]")

                quat = rotation_matrix_to_quaternion(cam_rot)
                
                t.transform.rotation.w = float(quat[0])
                t.transform.rotation.x = float(quat[1])
                t.transform.rotation.y = float(quat[2])
                t.transform.rotation.z = float(quat[3])

                # Send the transform
                self.tf_broadcaster.sendTransform(t)

            # Put 50ms between detection cycles
            time.sleep(0.05)
            pass
    
    def publish_pose(self):
        pass
    
def main(args=None):
    rclpy.init(args=args)

    localizer = TagLocalizer("bender")

    print("Starting tag localizer...")

    rclpy.spin(localizer)

    localizer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()