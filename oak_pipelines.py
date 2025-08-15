import uuid
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
    if output_prefix is None:
        output_prefix = str(uuid.uuid4())

    match cam_type:
        case CamType.RGB:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            output = cam.requestOutput((1280, 800), dai.ImgFrame.Type.NV12 ,dai.ImgResizeMode.CROP, 20)

            if rec_flag:
                record = pipeline.create(dai.node.RecordVideo)
                record.setRecordVideoFile(f"{output_prefix}.mp4")
                record.setRecordMetadataFile(f"{output_prefix}.mcap")
                output.link(record.input)

        case CamType.RGBD:
            """
            IMG_SHAPE = (640, 400)
            rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            rgb_out = rgb.requestOutput(IMG_SHAPE, dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 20)

            l = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            r = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            l_out = l.requestOutput(IMG_SHAPE, type=dai.ImgFrame.Type.NV12)
            r_out = r.requestOutput(IMG_SHAPE, type=dai.ImgFrame.Type.NV12)
            mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY

            stereo = pipeline.create(dai.node.StereoDepth).build(
                left=l_out, right=r_out,
                presetMode=mode
            )

            rgbd = pipeline.create(dai.node.RGBD).build()
            rgb_out.link(rgbd.inColor)
            stereo.depth.link(rgbd.inDepth)

            output = rgbd.rgbd
            # output = rgbd.inColor.getParent() #.getInputs()[0] # Get color output of sync nodezs
            # print(output)
            # print(output.__dir__())
            # print(output.getParent().getOutputs()[0].__dir__())

            if rec_flag:
                record_rgb = pipeline.create(dai.node.RecordVideo)
                record_rgb.setRecordVideoFile(f"{output_prefix}_rgb.mp4")
                record_rgb.setRecordMetadataFile(f"{output_prefix}_rgb.mcap")
                rgb_out.link(record_rgb.input)

                record_stereo = pipeline.create(dai.node.RecordVideo)
                record_stereo.setRecordVideoFile(f"{output_prefix}_depth.mp4")
                record_stereo.setRecordMetadataFile(f"{output_prefix}_depth.mcap")
                stereo.depth.link(record_stereo.input)

                record_pcl = pipeline.create(dai.node.RecordMetadataOnly)
                record_pcl.setRecordFile(f"{output_prefix}_pcl.mcap")
                rgbd.pcl.link(record_pcl.input)
            """
            mode = dai.node.StereoDepth.PresetMode.FAST_ACCURACY
            rgbd = pipeline.create(dai.node.RGBD).build(True, mode)
            # demux = pipeline.create(dai.node.MessageDemux)
            # sync_out = rgbd.inColor.getParent()

            output = rgbd.rgbd
            # sync_out.getOutputs()[0].link(demux.input)
            # print(sync_out.getOutputs()[0].__dir__())

            if rec_flag:
                config = dai.RecordConfig()
                config.outputDir = f"./{output_prefix}_data"
                # config.videoEncoding.enabled = True
                # config.videoEncoding.profile = dai.VideoEncoderProperties.Profile.H264_MAIN
                pipeline.enableHolisticRecord(config)
                # record_rgb = pipeline.create(dai.node.RecordVideo)
                # record_rgb.setRecordVideoFile(f"{output_prefix}_rgb.mp4")
                # record_rgb.setRecordMetadataFile(f"{output_prefix}_rgb.mcap")
                # demux.outputs["rgb"].link(record_rgb.input)

                # record_stereo = pipeline.create(dai.node.RecordVideo)
                # record_stereo.setRecordVideoFile(f"{output_prefix}_depth.mp4")
                # record_stereo.setRecordMetadataFile(f"{output_prefix}_depth.mcap")
                # demux.outputs["depth"].link(record_stereo.input)

                record = pipeline.create(dai.node.RecordMetadataOnly)
                record.setRecordFile(f"{output_prefix}_data/pcl.mcap")
                rgbd.pcl.link(record.input)

    return pipeline, output.createOutputQueue()