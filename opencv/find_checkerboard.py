# https://stackoverflow.com/questions/66225558/cv2-findchessboardcorners-fails-to-find-corners

import cv2

def main():
    cap = cv2.VideoCapture(4)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cv2.imshow('Checkerboard Capture', gray)
        key = cv2.waitKey(1)

        if key == 27:  # ESC
            break
        elif key == 32:  # SPACE
            found, corners = cv2.findChessboardCorners(gray, (5, 7))
            if found:
                print(corners)

if __name__ == "__main__":
    main()