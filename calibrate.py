import contextlib
from pathlib import Path

import cv2
import numpy as np
import depthai as dai

import utils
import charuco as chboard


def estimate_frame_pose(cam, frame, cboard):
    detector = cv2.aruco.CharucoDetector(cboard)
    c_corners, c_ids, m_corners, m_ids = detector.detectBoard(frame)
    if c_corners is None:
        return None

    obj_pts, img_pts = cboard.matchImagePoints(c_corners, c_ids)

    _, rvec, tvec = cv2.solvePnP(obj_pts, img_pts,
                                 cam.intrinsics, cam.distortion)

    return(rvec, tvec, c_corners, c_ids)

def update_cam(cam, rvec, tvec):
    rotM = cv2.Rodrigues(rvec)[0]
    cam.world_to_cam = np.vstack((np.hstack((rotM, tvec)), np.array([0,0,0,1])))
    cam.cam_to_world = np.linalg.inv(cam.world_to_cam)
    cam.rvec = rvec
    cam.tvec = tvec

    cam.save()

def calibrate(record_data, checkerboard, visualize=False):
    cam = record_data.cam_params

    video_dir = record_data.video_dir
    mp4_fp = video_dir / "CameraCAM_A.mp4"
    if not video_dir.exists():
        raise Exception("No RGB Camera file found")

    frames = []
    cap = cv2.VideoCapture(str(mp4_fp))

    rot, trans = [], []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        ret = estimate_frame_pose(cam, frame, checkerboard)
        if ret is not None:
            r, t, corners, marker_ids = ret
            rot.append(r)
            trans.append(t)
            rgb_frame = cv2.aruco.drawDetectedCornersCharuco(
                rgb_frame, corners, marker_ids, (255, 0, 0))
            rgb_frame = cv2.drawFrameAxes(
                rgb_frame, cam.intrinsics, cam.distortion,
                r, t, 0.05)
        if visualize:
            cv2.imshow("test", rgb_frame)
            cv2.waitKey(100)

    cap.release()

    return(np.array(rot), np.array(trans))


def main():
    visualize = True
    data_dir = Path("calibration_20250815_1428")
    device_dirs = [_ for _ in data_dir.glob("*")]

    checkerboard = chboard.get_charucoboard()

    print(device_dirs)
    for d in device_dirs:
        record_data = utils.RecordData(d)

        r, t = calibrate(record_data, checkerboard, visualize)

        r = r.mean(axis=0)
        t = t.mean(axis=0)
        cam_params = record_data.cam_params
        update_cam(cam_params, r, t)
        print(r, t)


if __name__ == "__main__":
    main()
