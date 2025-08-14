import depthai as dai
import cv2

def displayFrame(msg):
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
    cv2.imshow("video_device", frame)

def main():
    with dai.Pipeline() as pipeline:
        remoteConnector = dai.RemoteConnection(
            webSocketPort=8765, httpPort=8080
        )

        # replay = pipeline.create(dai.node.ReplayMetadataOnly)
        # replay.setReplayFile("0.mcap")
        # replay.setLoop(True)
        # output = replay.out.createOutputQueue()
        # remoteConnector.addTopic("pcl", replay.out, "common")

        prefix = "1"
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(f"{prefix}.mp4")
        replay.setReplayMetadataFile(f"{prefix}.mcap")
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
        replay.setSize(1200, 800)
        replay.setFps(30)
        replay.setLoop(True)
        output = replay.out.createOutputQueue()
        remoteConnector.addTopic("video", replay.out, "common")

        pipeline.start()

        while pipeline.isRunning:
            key = remoteConnector.waitKey(1)
            # pcl = output.get()
            # print(pcl)
            if cv2.waitKey(1) == ord('q'):
                break

if __name__ == "__main__":
    main()
