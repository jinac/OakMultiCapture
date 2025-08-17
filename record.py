import math
from datetime import datetime, timedelta
from pathlib import Path

import cv2
import numpy as np
import depthai as dai
import contextlib

import utils

import oak_pipelines as oakpipe
from camParams import CamData


def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")

def init_stream(stack, out_dir):
    pipeline = stack.enter_context(dai.Pipeline())
    out = utils.create_record_pipeline(pipeline, str(out_dir), False)
    # pipeline, output, record_data = out

    pipeline.start()
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

def run():
    out_dir = Path(datetime.now().strftime("calibration_%Y%m%d_%H%M"))
    if not out_dir.exists():
        out_dir.mkdir()
    print(f"Saving to {out_dir}")

    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        # rgbd_sync = utils.HostRGBDQueueSync()
        # queues = []
        pipelines = []
        cam_data = []

        for idx in range(len(deviceInfos)):
            pipeline, record_data = init_stream(stack, out_dir / str(idx))

            # rgbd_sync.add_queue(output)
            cam_data.append(record_data)
            pipelines.append(pipeline)

        # print(pipelines)
        # print(queues)

        # msgs = [[] for _ in queues]
        # while True:
        #     # msgs = rgbd_sync.tryGetSample()
        #     print(msgs)
        #     for idx, data in enumerate(msgs):
        #         rgb_frame = data.getRGBFrame()
        #         # d_frame = data.getDepthFrame()
        #         displayFrame(rgb_frame, f"{idx}_0")
        #         # displayFrame(d_frame, f"{idx}_1")
        while True:
            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Start record.py")
    check_devices()
    run()


if __name__ == "__main__":
    main()
