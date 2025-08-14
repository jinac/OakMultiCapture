import numpy as numpy
import cv2

# ------------------------------
# ENTER YOUR PARAMETERS HERE:
ARUCO_DICT = cv2.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 7
SQUARES_HORIZONTALLY = 5
SQUARE_LENGTH = 0.033
MARKER_LENGTH = 0.017
LENGTH_PX = 640   # total length of the page in pixels
MARGIN_PX = 20    # size of the margin in pixels
SAVE_NAME = 'ChArUco_Marker.png'
# ------------------------------

def get_charucoboard():
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, d)
    # size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
    # # charucoParams = cv2.aruco.CharucoParameters()
    # # detectorParams = cv2.aruco.DetectorParameters()
    # # detector = cv2.aruco.CharucoDetector(board, charucoParams, detectorParams)
    return board

def main():
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, d)
    size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
    img = cv2.aruco.CharucoBoard.generateImage(board, (LENGTH_PX, int(LENGTH_PX*size_ratio)), marginSize=MARGIN_PX)
    cv2.imshow("img", img)
    cv2.waitKey(2000)
    cv2.imwrite(SAVE_NAME, img)

if __name__ == "__main__":
    main()