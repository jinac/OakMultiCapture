import argparse
import contextlib
from pathlib import Path

import depthai as dai
import cv2

import utils

def init_replay(stack, remote, record_dir, idx):
    pipeline = stack.enter_context(dai.Pipeline(False))

    # replay = pipeline.create(dai.node.ReplayMetadataOnly)
    # record_data = utils.RecordData(record_dir)
    # replay.setReplayFile(record_data.pcl)
    # replay.setLoop(True)

    replay = pipeline.create(dai.node.ReplayVideo)
    record_data = utils.RecordData(record_dir)
    replay.setReplayVideoFile(record_data.video_dir / "CameraCAM_A.mp4")
    replay.setReplayMetadataFile(record_data.video_dir / "CameraCAM_A.mcap")
    replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
    replay.setSize(600, 480)
    replay.setFps(30)
    replay.setLoop(True)

    # pipeline, replay, record_data = utils.create_replay_pipeline(pipeline, record_dir)

    remote.addTopic(f"pcl{idx}", replay.out, "common")

    return pipeline


def main():
    parser = argparse.ArgumentParser(description="Replay Recorded Oak Data")
    parser.add_argument("dir", help="Directory containing RecordData (i.e. calibration_20250816_2041)")
    args = parser.parse_args()

    with contextlib.ExitStack() as stack:
        data_dir = Path(args.dir)
        record_dirs = [_ for _ in data_dir.glob("*")]
        pipelines = []

        remoteConnector = dai.RemoteConnection(
            webSocketPort=8765, httpPort=8081
        )

        for idx, record_dir in enumerate(record_dirs):
            pipeline = init_replay(stack, remoteConnector, record_dir, idx)
            pipelines.append(pipeline)
            pipeline.start()

        while True:
            if cv2.waitKey(1) == ord('q'):
                break
    for pipeline in pipelines:
        pipeline.stop()

if __name__ == "__main__":
    main()
