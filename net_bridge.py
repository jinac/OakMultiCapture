import math
from datetime import timedelta, datetime
from pathlib import Path
import time

import cv2
import numpy as np
import depthai as dai
import contextlib
import utils
import nodes


def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")

def init_stream(stack):
    pipeline = stack.enter_context(dai.Pipeline())
    out = utils.create_watch_pipeline(pipeline, True)
    pipeline, output = out

    # pipeline.start()
    return(pipeline)

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

def run():
    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        queues = []
        pipelines = []
        # rgbd_sync = utils.HostRGBDQueueSync()

        for idx in range(len(deviceInfos)):
            pipeline = init_stream(stack)

            # rgbd_sync.add_queue(output)
            pipelines.append(pipeline)
            # queues.append(output)

        for p in pipelines:
            p.start()


        try:
            while True:
                print([p.isRunning() for p in pipelines])
                print("Looping... Press Ctrl+C to exit.")
                time.sleep(1)  # Simulate some work being done
        except KeyboardInterrupt:
            print("Exiting...")
            for p in pipelines:
                p.stop()
def main():
    check_devices()
    run()


if __name__ == "__main__":
    main()
