import contextlib
from pathlib import Path

import cv2
import numpy as np
import depthai as dai

import oak_pipelines as oakpipe
import charuco as chboard
from camParams import CamData


def estimate_frame_pose(cam, frame, cboard):
    detector = cv2.aruco.CharucoDetector(cboard)
    c_corners, c_ids, m_corners, m_ids = detector.detectBoard(frame)

    obj_pts, img_pts = cboard.matchImagePoints(c_corners, c_ids)

    # print(cam.intrinsics.shape, cam.distortion.shape)
    ret, rvec, tvec = cv2.solvePnP(obj_pts, img_pts,
                                   cam.intrinsics, cam.distortion)

    rotM = cv2.Rodrigues(rvec)[0]
    cam.world_to_cam = np.vstack((np.hstack((rotM, tvec)), np.array([0,0,0,1])))
    cam.cam_to_world = np.linalg.inv(cam.world_to_cam)
    cam.rvec = rvec
    cam.tvec = tvec

    return(rvec, tvec, c_corners, c_ids)

def calibrate(cam, checkerboard, mp4_fp, visualize=False):
    mp4_fp = Path(mp4_fp)
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
            # corners = np.squeeze(corners)
            # print(corners.shape, marker_ids.shape)
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

        # frames.append(frame)

    cap.release()

    # print(len(frames))
    # rot, trans = estimate_extrinsics(cam, frames, checkerboard)
    return(np.array(rot), np.array(trans))
    # return(None, None)


def main():
    visualize = True
    prefix = [
        "calibration_20250813/0",
        "calibration_20250813/1"
    ]

    cams = []

    for idx, pre in enumerate(prefix):
        c = CamData()
        param_fp = pre + "_calib.npz"
        c.load(param_fp)

        cams.append(c)

    # checkerboard = load_checkerboard()
    checkerboard = chboard.get_charucoboard()

    for c, pre in zip(cams, prefix):
        r, t = calibrate(c, checkerboard, pre+".mp4", visualize)
        r = r.mean(axis=0)
        t = t.mean(axis=0)
        c.save(param_fp)
        # print(r[0], t[0])
        # print(r.mean(axis=0))
        # print(t.mean(axis=0))


if __name__ == "__main__":
    main()
