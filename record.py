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

def init_stream(stack, remote_connector, dir_str, idx):
    pipeline = stack.enter_context(dai.Pipeline())
    pipeline, record_data, pcl_out = utils.create_record_pipeline(pipeline, Path(dir_str, str(idx)), True, True, False)

    remote_connector.addTopic("pcl_{}".format(idx), pcl_out.pcl, "common")
    # pipeline.start()
    return(pipeline)

def run():
    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        pipelines = []

        remoteConnector = dai.RemoteConnection(
            webSocketPort=8765, httpPort=8081
        )

        dir_str = datetime.now().strftime("calibration_%Y%m%d_%H%M")
        out_dir = Path(dir_str)
        if not out_dir.exists():
            out_dir.mkdir()

        for idx in range(len(deviceInfos)):
            pipeline = init_stream(stack, remoteConnector, dir_str, idx)
            pipelines.append(pipeline)

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
