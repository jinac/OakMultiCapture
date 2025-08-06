import cv2
import depthai as dai
import contextlib

def check_devices():
    for device in dai.Device.getAllAvailableDevices():
        print(f"{device.getDeviceId()} {device.state}")


def createPipeline(pipeline):
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    # output = camRgb.requestFullResolutionOutput().createOutputQueue()
    output = camRgb.requestOutput((1280, 800), dai.ImgFrame.Type.NV12 ,dai.ImgResizeMode.CROP, 20).createOutputQueue()
    return pipeline, output

def initStream(stack, deviceInfo):
    pipeline = stack.enter_context(dai.Pipeline())
    device = pipeline.getDefaultDevice()

    print("===Connected to ", deviceInfo.getDeviceId())
    mxId = device.getDeviceId()
    cameras = device.getConnectedCameras()
    usbSpeed = device.getUsbSpeed()
    eepromData = device.readCalibration2().getEepromData()

    print("   >>> Device ID:", mxId)
    print("   >>> Num of cameras:", len(cameras))
    if eepromData.boardName != "":
        print("   >>> Board name:", eepromData.boardName)
    if eepromData.productName != "":
        print("   >>> Product name:", eepromData.productName)

    pipeline, output = createPipeline(pipeline)
    pipeline.start()

    return pipeline, output

def displayFrame(msg):
        assert isinstance(msg, dai.ImgFrame)
        frame = msg.getCvFrame()
        seqNum = msg.getSequenceNum()
        timestamp = msg.getTimestamp()
        frame = cv2.putText(
            frame,
            str(timestamp),
            (50,50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0))
        cv2.imshow(f"video_device{i}", frame)

def run():
    with contextlib.ExitStack() as stack:
        deviceInfos = dai.Device.getAllAvailableDevices()
        print("=== Found devices: ", deviceInfos)
        queues = []
        pipelines = []

        for deviceInfo in deviceInfos:
            pipeline, output = initStream(stack, deviceInfo)

            pipelines.append(pipeline)
            queues.append(output)

        # print(pipelines)
        # print(queues)

        while True:
            for i, stream in enumerate(queues):
                videoIn = stream.get()
                displayFrame(videoIn)
            if cv2.waitKey(1) == ord('q'):
                break

def main():
    print("Hello from luxtest!")

    check_devices()
    run()


if __name__ == "__main__":
    main()
