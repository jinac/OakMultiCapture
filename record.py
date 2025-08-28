import math
from datetime import datetime, timedelta
from pathlib import Path

import cv2
import numpy as np
import depthai as dai
import contextlib

import utils


def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")

def init_stream(stack, out_dir, vis):
    pipeline = stack.enter_context(dai.Pipeline())
    out = utils.create_record_pipeline(pipeline, str(out_dir), True, True, vis)

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
        pipelines = []
        cam_data = []

        for idx in range(len(deviceInfos)):
            vis = idx == 0
            pipeline, record_data = init_stream(stack, out_dir / str(idx), vis)

            cam_data.append(record_data)
            pipelines.append(pipeline)

        for p in pipelines:
            p.start()

        while True:
            if cv2.waitKey(100) == ord('q'):
                break

def main():
    print("Start record.py")
    check_devices()
    run()


if __name__ == "__main__":
    main()
