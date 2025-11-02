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

def init_stream(stack, remote_connector):
    pipeline = stack.enter_context(dai.Pipeline())
    out = utils.create_watch3D_pipeline(pipeline, False)
    pipeline, output = out

    remote_connector.addTopic("pcl", output, "common")
    # pipeline.start()
    return(pipeline, output)

# def displayFrame(msg, i):
#     assert isinstance(msg, dai.ImgFrame)
#     frame = msg.getCvFrame()
#     seqNum = msg.getSequenceNum()
#     timestamp = msg.getTimestamp()
#     frame = cv2.putText(
#         frame,
#         str(timestamp.total_seconds()),
#         (50,50),
#         cv2.FONT_HERSHEY_SIMPLEX,
#         1,
#         (0, 255, 0))
#     cv2.imshow(f"video_device{i}", frame)

def run():
    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        # queues = []
        pipelines = []
        # rgbd_sync = utils.HostRGBDQueueSync()

        remoteConnector = dai.RemoteConnection(
            webSocketPort=8765, httpPort=8081
        )

        for idx in range(len(deviceInfos)):
            pipeline, output = init_stream(stack, remoteConnector)

            # rgbd_sync.add_queue(output)
            pipelines.append(pipeline)
            # queues.append(output)

        for pipeline in pipelines:
            pipeline.start()
            remoteConnector.registerPipeline(pipeline)

        try:
            while True:
                for p in pipelines:
                    print(p.isRunning())
                print("Looping... Press Ctrl+C to exit.")
                time.sleep(1)  # Simulate some work being done
        except KeyboardInterrupt:
            print("Exiting...")
            for p in pipelines:
                p.stop()

def main():
    print("Hello from luxtest!")

    check_devices()
    run()


if __name__ == "__main__":
    main()
