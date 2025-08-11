import cv2
from pupil_apriltags import Detector, Detection

class BenderTagDetector:
    def __init__(self, camera_index: int, camera_parameters: tuple, tag_size: float, tag_family: str = "tag36h11"):
        self.camera_index = camera_index
        self.camera_parameters = camera_parameters
        self.tag_size = tag_size
        self.tag_family = tag_family

        self.cap = cv2.VideoCapture(self.camera_index)
        self.at_detector = Detector(
            families=self.tag_family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

    def _to_grayscale(self, image):
        """
        Converts a BGR image to grayscale using OpenCV.

        Args:
            image (numpy.ndarray): Input image in BGR format.

        Returns:
            numpy.ndarray: Grayscale image.
        """
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return grayscale

    def _draw_tag(self, img, d: Detection):
        # Draw the detected AprilTag on the image
        cv2.circle(img, (int(d.center[0]), int(d.center[1])), 5, (0, 0, 255), -1)
        for corner in d.corners:
            cv2.circle(img, (int(corner[0]), int(corner[1])), 5, (0, 255, 0), -1)
        return img
    
    def run_detect(self) -> list[Detection] | None:
        ret, frame = self.cap.read()
        if not ret:
            return None
        # Turn to grayscale
        gray = self._to_grayscale(frame)
        # Detect tags
        detection = self.at_detector.detect(gray, estimate_tag_pose=True, camera_params=self.camera_parameters, tag_size=self.tag_size)

        # frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        # for d in detection:
        #     frame = self._draw_tag(frame, d)

        # cv2.imshow("Tag Detector", frame)

        return detection

    def release_resources(self):
        self.cap.release()
        cv2.destroyAllWindows()