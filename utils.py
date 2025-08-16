import uuid
from enum import Enum
from typing import Tuple, Union
from pathlib import Path
import tarfile

import depthai as dai
import numpy as np


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


def create_record_pipeline(pipeline: dai.Pipeline,
                           record_dir_path: Union[Path | str],
                           record_pcl_flag: bool = True,
                           record_holistic_flag: bool = True) -> Tuple[dai.Pipeline, dai.node, RecordData]:
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
    rgbd = pipeline.create(dai.node.RGBD).build(True, mode)
    output = rgbd.rgbd.createOutputQueue()

    if record_pcl_flag:
        record = pipeline.create(dai.node.RecordMetadataOnly)
        record.setRecordFile(str(record_data.pcl))
        rgbd.pcl.link(record.input)

    if record_holistic_flag:
        config = dai.RecordConfig()
        config.outputDir = str(record_data.dir)
        pipeline.enableHolisticRecord(config)

    return(pipeline, output, record_data)

def create_replay_pipeline(pipeline: dai.Pipeline,
                           record_dir_path: Union[Path | str]) -> Tuple[dai.Pipeline, dai.node, RecordData]:
    record_data = RecordData(record_dir_path)

    pipeline.enableHolisticReplay(str(record_data.holistic_record))
    mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY
    rgbd = pipeline.create(dai.node.RGBD).build(True, mode)
    output = rgbd.pcl

    return(pipeline, output, record_data)
