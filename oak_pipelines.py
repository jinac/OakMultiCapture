from enum import Enum

from pathlib import Path
import depthai as dai

from typing import Tuple

class CamType(Enum):
    RGB = 1
    RGBD = 2

def createPipeline(pipeline: dai.Pipeline,
                   cam_type: CamType,
                   rec_flag: bool,
                   output_prefix: str | None) -> Tuple[dai.Pipeline, dai.node]:
    match cam_type:
        case CamType.RGB:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            output = cam.requestOutput((1280, 800), dai.ImgFrame.Type.NV12 ,dai.ImgResizeMode.CROP, 20)

            if rec_flag:
                record = pipeline.create(dai.node.RecordVideo)
                record.setRecordVideoFile(f"{output_prefix}.mp4")
                output.link(record.input)

        case CamType.RGBD:
            mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY
            rgbd = pipeline.create(dai.node.RGBD).build(True, mode)
            output = rgbd.rgbd

            if rec_flag:
                record = pipeline.create(dai.node.RecordMetadataOnly)
                record.setRecordFile(f"{output_prefix}.mcap")
                rgbd.pcl.link(record.input)

    return pipeline, output.createOutputQueue()