import math
from datetime import timedelta

import cv2
import numpy as np
import depthai as dai
import contextlib

def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")


def createPipeline(pipeline):
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    # output = camRgb.requestFullResolutionOutput().createOutputQueue()
    output = camRgb.requestOutput((1280, 800), dai.ImgFrame.Type.NV12 ,dai.ImgResizeMode.CROP, 20).createOutputQueue()
    return pipeline, output

def createRGBDPipeline(pipeline):
    rgbd = pipeline.create(dai.node.RGBD).build()
    # camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    # camRgb.link(rgbd.inColor)
    output = rgbd.rgbd.createOutputQueue()
    return pipeline, output


def initStream(stack, deviceInfo):
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

    pipeline, output = createPipeline(pipeline)
    # pipeline, output = createRGBDPipeline(pipeline)
    pipeline.start()

    return pipeline, output

def displayFrame(msg, i):
    # print(msg)
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
    print(timestamps)
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

        for deviceInfo in deviceInfos:
            pipeline, output = initStream(stack, deviceInfo)

            pipelines.append(pipeline)
            queues.append(output)

        print(pipelines)
        print(queues)

        msgs = [[] for _ in queues]
        while True:
            # displayJoinedFrames(queues)
            # for i, stream in enumerate(queues):
            #     videoIn = stream.get()
            #     displayFrame(videoIn, i)
            for i, stream in enumerate(queues):
                new_msg = stream.tryGet()
                if new_msg is not None:
                    msgs[i].append(new_msg)
                    if check_sync(msgs, new_msg.getTimestamp()):
                        for i, q in enumerate(msgs):
                            msg = q.pop(0)
                            displayFrame(msg, i)
            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Hello from luxtest!")

    check_devices()
    run()


if __name__ == "__main__":
    main()
