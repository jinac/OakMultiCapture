import math
from datetime import timedelta
from pathlib import Path

import cv2
import numpy as np
import depthai as dai
import contextlib
import oak_pipelines as oakpipe
from camParams import CamData


def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")

def initStream(stack, deviceInfo, dev_idx):
    pipeline = stack.enter_context(dai.Pipeline())
    device = pipeline.getDefaultDevice()
    # device = dai.Device(deviceInfo)

    print("===Connected to ", deviceInfo.getDeviceId())
    mxId = device.getDeviceId()
    cameras = device.getConnectedCameras()
    usbSpeed = device.getUsbSpeed()
    eepromData = device.readCalibration2().getEepromData()

    print("   >>> Device ID:", mxId)
    print("   >>> Num of cameras:", len(cameras))
    if eepromData.boardName != "":
        print("   >>> Board name:", eepromData.boardName)
    if eepromData.productName != "":
        print("   >>> Product name:", eepromData.productName)

    calibData = device.readCalibration2()

    prefix = str(dev_idx)
    intrinsics = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)).reshape(3, 3)
    dist = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
    cam_data = CamData()
    cam_data.intrinsics = intrinsics
    cam_data.distortion = dist
    cam_data.save(param_fp = prefix + "/calib.npz")

    pipeline, output = oakpipe.createPipeline(
        pipeline, 
        cam_type=oakpipe.CamType.RGBD,
        rec_flag=True,
        output_prefix=prefix)
    pipeline.start()

    return pipeline, output

def displayFrame(msg, i):
    assert isinstance(msg, dai.ImgFrame)
    frame = msg.getCvFrame()
    seqNum = msg.getSequenceNum()
    timestamp = msg.getTimestamp()
    frame = cv2.putText(
        frame,
        str(timestamp.total_seconds()),
        (50,50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0))
    cv2.imshow(f"video_device{i}", frame)

def displayJoinedFrames(queues):
    frames = []
    timestamps = []
    for queue in queues:
        msg = queue.tryGet()
        if msg is not None:
            frames.append(msg.getCvFrame())
            timestamps.append(msg.getTimestamp().total_seconds())
    if len(frames) > 0:
        cv2.imshow(f"video_streams", np.hstack(frames))

def check_sync(queues, timestamp):
    matching_frames = []
    for q in queues:
        for i, msg in enumerate(q):
            time_diff = abs(msg.getTimestamp() - timestamp)
            # So below 17ms @ 30 FPS => frames are in sync
            if time_diff <= timedelta(milliseconds=math.ceil(500 / 30)):
                matching_frames.append(i)
                break

    if len(matching_frames) == len(queues):
        # We have all frames synced. Remove the excess ones
        for i, q in enumerate(queues):
            q = q[matching_frames[i]:]
        return True
    else:
        return False


def run():
    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        queues = []
        pipelines = []

        for dev_idx, deviceInfo in enumerate(deviceInfos):
            pipeline, output = initStream(stack, deviceInfo, dev_idx)

            pipelines.append(pipeline)
            queues.append(output)

        # print(pipelines)
        # print(queues)

        msgs = [[] for _ in queues]
        while True:

            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Hello from luxtest!")

    check_devices()
    run()


if __name__ == "__main__":
    main()
