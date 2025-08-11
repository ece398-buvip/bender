import cv2
import numpy as np

def capture_checkerboard_images(num_images=15, checkerboard_dims=(7, 5)):
    """
    Captures images of a checkerboard pattern from a webcam.
    Returns a list of saved image filenames.
    """
    cap = cv2.VideoCapture(4)
    saved_images = []
    print(f"Press SPACE to capture an image of the checkerboard. Need {num_images} images. Press ESC to exit.")
    count = 0

    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        display_frame = frame.copy()
        cv2.putText(display_frame, f"Image {count+1}/{num_images}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow('Checkerboard Capture', display_frame)
        key = cv2.waitKey(1)

        if key == 27:  # ESC
            break
        elif key == 32:  # SPACE
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, checkerboard_dims, None)
            if found:
                filename = f"checkerboard_{count+1}.png"
                cv2.imwrite(filename, frame)
                saved_images.append(filename)
                print(f"Captured and saved {filename}")
                count += 1
            else:
                print("Checkerboard not detected. Try again.")

    cap.release()
    cv2.destroyAllWindows()
    return saved_images

def calibrate_camera(images, checkerboard_dims=(7,6), square_size=1.0):
    """
    Given checkerboard images, calibrate the camera and return calibration parameters.
    """
    objp = np.zeros((checkerboard_dims[0]*checkerboard_dims[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_dims[0], 0:checkerboard_dims[1]].T.reshape(-1,2)
    objp *= square_size

    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, checkerboard_dims, None)
        if found:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, checkerboard_dims, corners2, found)
            cv2.imshow('Corners', img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()

    if len(objpoints) < 5:
        print("Not enough checkerboard images for calibration.")
        return None

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print("Calibration successful!")
    print(f"Camera matrix:\n{mtx}")
    print(f"Distortion coefficients:\n{dist}")
    return mtx, dist, rvecs, tvecs

def main():
    print("Checkerboard Camera Calibration Tool")
    print("Step 1: Capture images of the checkerboard.")
    checkerboard_dims = (7, 5)
    num_images = 15
    square_size = 1.0  # Adjust to your checkerboard's actual square size (e.g., in cm or mm)
    images = capture_checkerboard_images(num_images, checkerboard_dims)
    if images:
        print("Step 2: Calibrating camera...")
        calibrate_camera(images, checkerboard_dims, square_size)
    else:
        print("No images were captured.")

if __name__ == "__main__":
    main()