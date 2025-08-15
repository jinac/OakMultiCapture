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
    out = utils.create_record_pipeline(pipeline, str(out_dir))
    pipeline, output, record_data = out

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
    out_dir = Path(datetime.now().strftime("calibration_%Y%m%d_%H%M"))
    if not out_dir.exists():
        out_dir.mkdir()
    print(f"Saving to {out_dir}")

    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        queues = []
        pipelines = []
        cam_data = []

        for idx in range(len(deviceInfos)):
            pipeline, output, record_data = init_stream(stack, out_dir / str(idx))

            cam_data.append(record_data)
            pipelines.append(pipeline)
            queues.append(output)

        # print(pipelines)
        # print(queues)

        msgs = [[] for _ in queues]
        while True:
            for i, stream in enumerate(queues):
                rgbd_frame = stream.get()
                displayFrame(rgbd_frame.getRGBFrame(), i)
                # displayFrame(rgbd_frame.getDepthFrame(), i)

            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Start record.py")
    check_devices()
    run()


if __name__ == "__main__":
    main()
