import time
import threading
import rclpy
from rclpy.node import Node
from bender_tf2.bender_tag_detector import BenderTagDetector
from geometry_msgs.msg import TransformStamped

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
                t.header.frame_id = 'world'
                t.child_frame_id = self.robot_name

                # TODO: We care about rotation... but start with pose
                # TODO: I think this is wrong - tf2 is built to solve the problem of translating frames, maybe we need to rotate frame somehow?
                t.transform.translation.x = float(d.pose_t[0]) # Camera x is world x - leave as is
                t.transform.translation.y = float(d.pose_t[2]) # Camera z is wold y
                t.transform.translation.z = float(d.pose_t[1]) # Camera y is world z
                rclpy.logging.get_logger("bender_tf2").debug(F"Publishing pose [{t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}]")

                # TODO: Set rotation here

                # Send the transform
                self.tf_broadcaster.sendTransform(t)

            # Put 10ms between detection cycles
            time.sleep(0.01)
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