import math
from datetime import timedelta, datetime
from pathlib import Path

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

    remote_connector.addTopic("pcl", output, "commoon")
    # pipeline.start()
    return(pipeline, output)

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

        while True:
            # msgs = rgbd_sync.tryGetSample()
            # for idx, data in enumerate(msgs):
            #     rgb_frame = data.getRGBFrame()
            #     d_frame = data.getDepthFrame()
            #     displayFrame(rgb_frame, f"{idx}_0")
            #     displayFrame(d_frame, f"{idx}_1")

            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Hello from luxtest!")

    check_devices()
    run()


if __name__ == "__main__":
    main()
