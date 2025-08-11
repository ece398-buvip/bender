from bender_tag_detector import BenderTagDetector
import cv2

fx = float(636.56831794)
fy = float(632.28979379)
cx = float(307.50223257)
cy = float(238.50969476)

camera_params = tuple(
    [fx, fy, cx, cy]
)

def main():
    detector = BenderTagDetector(4, camera_params, 0.2)
    while True:
        detections = detector.run_detect()
        if detections is not None:
            for d in detections:
                print(F"X: [{d.pose_t[0]}] Y: [{d.pose_t[1]}] Z: [{d.pose_t[2]}]")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    detector.release_resources()

if __name__ == "__main__":
    main()