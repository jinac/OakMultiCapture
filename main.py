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

def init_stream(stack, rec_dir):
    record_data = utils.RecordData(rec_dir)
    pipeline = stack.enter_context(dai.Pipeline())
    out = utils.create_watch3D_pipeline(pipeline, False, record_data)
    pipeline, output = out

    # remote_connector.addTopic("pcl{}".format(rec_dir), output, "common")
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
    cam_data = [
        "calibration_20251102_0346/1",
        "calibration_20251102_0346/0"
    ]

    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        # queues = []
        pipelines = []
        # rgbd_sync = utils.HostRGBDQueueSync()

        # remoteConnector = dai.RemoteConnection(
        #     webSocketPort=8765, httpPort=8081
        # )

        merge_node = utils.HostQueueSync()

        for idx in range(len(deviceInfos)):
            pipeline, output = init_stream(stack, cam_data[idx])

            # rgbd_sync.add_queue(output)
            pipelines.append(pipeline)
            # queues.append(output)
            merge_node.add_queue(output)


        for pipeline in pipelines:
            pipeline.start()
            # remoteConnector.registerPipeline(pipeline)

        try:
            import rerun as rr
            rr.init("visualize_merged_pointcloud", spawn=True)
            while True:
                data = merge_node.tryGetSample()
                if data:
                    # outPointCloud = dai.PointCloudData()
                    combined_points = []
                    combined_colors = []

                    for msg in data:
                        points, colors = msg.getPointsRGB()
                        print(points.shape, colors.shape)
                        # ones = np.ones((points.shape[0], 1))
                        # points = np.concatenate((points, ones), axis=1)
                        combined_points.append(points)
                        combined_colors.append(colors)

                    combined_points = np.concatenate(combined_points)
                    combined_colors = np.concatenate(combined_colors)

                    print(combined_colors.shape)
                    print(combined_points.shape)

                    rr.log(
                        "points",
                        rr.Points3D(combined_points, colors=combined_colors, radii=0.1)
                    )

                    # outPointCloud.setPointsRGB(combined_points, combined_colors)

                # for p in pipelines:
                    # print(p.isRunning())
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
