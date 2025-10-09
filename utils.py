import uuid
import math
from datetime import timedelta
from enum import Enum
from typing import Tuple, Union
from pathlib import Path
import tarfile

import depthai as dai
import numpy as np

import nodes

class CamType(Enum):
    NONE = 0
    RGB = 1
    RGBD = 2


class CamParams():
    def __init__(self, filepath=None):
        self.intrinsics = np.eye(4)
        self.distortion = np.zeros((1,5))
        self.world_to_cam = np.eye(4)
        self.cam_to_world = np.eye(4)
        self.rvec = np.zeros(3)
        self.tvec = np.zeros(3)
        self.filepath = Path(filepath)

        if self.filepath.exists():
            self.load()
        else:
            self.save()

    def load(self):
        with np.load(self.filepath) as data:
            for k in data.files:
                setattr(self, k, data[k])

    def save(self):
        np.savez(self.filepath,
                 intrinsics=self.intrinsics,
                 distortion=self.distortion,
                 world_to_cam=self.world_to_cam,
                 cam_to_world=self.cam_to_world,
                 rvec=self.rvec,
                 tvec=self.tvec)


class RecordData():
    def __init__(self, dir_path):
        self.dir = Path(dir_path)
        if not self.dir.exists():
            self.dir.mkdir()

        self.cam_params = CamParams(self.dir / "calib.npz")
        self.holistic_record = self.dir / "recording.tar"
        self.pcl = self.dir / "pcl.mcap"
        # self.rgbd_video = self.dir / "rgbd.mp4"
        self.video_dir = self.get_recordings()

    def get_recordings(self):
        record_dir = self.dir / "recordings"
        if not record_dir.exists() and self.holistic_record.exists():
            with tarfile.open(self.holistic_record) as f:
                f.extractall(record_dir)

        return record_dir


class HostRGBDQueueSync():
    def __init__(self, threshold=math.ceil(500 / 30)):
        self.threshold = threshold
        self.message_buffers = []
        self.queues = []

    def add_queue(self, queue):
        self.queues.append(queue)
        self.message_buffers.append([])

    def check_sync(self, timestamp):
        matching_frames = []
        for q in self.message_buffers:
            for i, msg in enumerate(q):
                time_diff = abs(msg.getTimestamp() - timestamp)
                # So below 17ms @ 30 FPS => frames are in sync
                if time_diff <= timedelta(milliseconds=self.threshold):
                    matching_frames.append(i)
                    break

        if len(matching_frames) == len(self.queues):
            # We have all frames synced. Remove the excess ones
            for i, q in enumerate(self.message_buffers):
                q = q[matching_frames[i]:]
            return True
        else:
            return False

    def tryGetSample(self):
        out = []
        for i, q in enumerate(self.queues):
            new_msg = q.tryGet()
            if new_msg is not None:
                self.message_buffers[i].append(new_msg)
                if self.check_sync(new_msg.getTimestamp()):
                    for buffer in self.message_buffers:
                        out.append(buffer.pop(0))
        return out


def create_record_pipeline(pipeline: dai.Pipeline,
                           record_dir_path: Union[Path | str],
                           record_pcl_flag: bool = True,
                           record_holistic_flag: bool = True,
                           visualize: bool = False) -> Tuple[dai.Pipeline, dai.node, RecordData]:
    device = pipeline.getDefaultDevice()

    print("===Connected to ", device.getDeviceId())
    mxId = device.getDeviceId()
    cameras = device.getConnectedCameras()
    usbSpeed = device.getUsbSpeed()
    eepromData = device.readCalibration2().getEepromData()

    print("   >>> Device ID:", mxId)
    print("   >>> Num of cameras:", len(cameras))
    print("   >>> USB Speed:", usbSpeed)
    if eepromData.boardName != "":
        print("   >>> Board name:", eepromData.boardName)
    if eepromData.productName != "":
        print("   >>> Product name:", eepromData.productName)
    
    calibData = device.readCalibration2()
    intrinsics = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)).reshape(3, 3)
    dist = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))

    if isinstance(record_dir_path, str) :
        record_dir_path = Path(record_dir_path)
    record_data = RecordData(record_dir_path)
    record_data.cam_params.intrinsics = intrinsics
    record_data.cam_params.distortion = dist
    record_data.cam_params.save()

    mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY
    size = (640, 480)
    rgbd = pipeline.create(dai.node.RGBD).build(True, mode, size)
    # output = rgbd.rgbd.createOutputQueue()
    if visualize:
        view = pipeline.create(nodes.RGBDDisplay).build(rgbd.inColor.getParent().out)

    if record_pcl_flag:
        record = pipeline.create(dai.node.RecordMetadataOnly)
        record.setRecordFile(str(record_data.pcl))
        rgbd.pcl.link(record.input)

    if record_holistic_flag:
        config = dai.RecordConfig()
        config.outputDir = str(record_data.dir)
        pipeline.enableHolisticRecord(config)

    return(pipeline, record_data)

def create_watch_pipeline(pipeline: dai.Pipeline,
                          sockForward: bool) -> Tuple[dai.Pipeline, dai.node]:
    device = pipeline.getDefaultDevice()

    print("=== Connected to ", device.getDeviceId())
    mxId = device.getDeviceId()
    cameras = device.getConnectedCameras()
    usbSpeed = device.getUsbSpeed()
    eepromData = device.readCalibration2().getEepromData()

    print("   >>> Device ID:", mxId)
    print("   >>> Num of cameras:", len(cameras))
    print("   >>> USB Speed:", usbSpeed)
    if eepromData.boardName != "":
        print("   >>> Board name:", eepromData.boardName)
    if eepromData.productName != "":
        print("   >>> Product name:", eepromData.productName)
    
    mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY
    size = (640, 480)
    rgbd = pipeline.create(dai.node.RGBD).build(True, mode, size)
    if sockForward:
        print(rgbd.inColor.getParent().out)
        pipeline.create(nodes.SocketForwarder).build(rgbd.inColor.getParent().out)
    output = rgbd.rgbd.createOutputQueue()

    return(pipeline, output)
    # return pipeline

# def create_replay_pipeline(pipeline: dai.Pipeline,
#                            record_dir_path: Union[Path | str]) -> Tuple[dai.Pipeline, dai.node, RecordData]:
#     record_data = RecordData(record_dir_path)

#     pipeline.enableHolisticReplay(str(record_data.holistic_record))
#     print(record_data.holistic_record)

#     mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY
#     size = (640, 480)
#     rgbd = pipeline.create(dai.node.RGBD).build(True, mode, size)
#     # output = rgbd.pcl
#     output = None

#     return(pipeline, output, record_data)
