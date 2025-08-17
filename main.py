import math
from datetime import timedelta, datetime
from pathlib import Path

import cv2
import numpy as np
import depthai as dai
import contextlib
import utils

def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")

def init_stream(stack):
    pipeline = stack.enter_context(dai.Pipeline())
    out = utils.create_watch_pipeline(pipeline)
    pipeline, output = out

    # pipeline.start()
    return(out)

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

        for idx in range(len(deviceInfos)):
            pipeline, output = init_stream(stack)

            pipelines.append(pipeline)
            queues.append(output)

        # print(pipelines)
        # print(queues)

        # msgs = [[] for _ in queues]
        pipeline.run()
        while True:
            for i, stream in enumerate(queues):
                rgbd_frame = stream.get()
                # print(type(rgbd_frame))
                # displayFrame(rgbd_frame.getRGBFrame(), i)
                # displayFrame(rgbd_frame.getDepthFrame(), i)

            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Hello from luxtest!")

    check_devices()
    run()


if __name__ == "__main__":
    main()
