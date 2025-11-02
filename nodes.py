import depthai as dai
import cv2
import numpy as np

from time import sleep

import zmq
import matrix_pb2
import calibrate

class RGBDDisplay(dai.node.HostNode):
    def build(self, rgbd_out):
        self.link_args(rgbd_out)

        self.sendProcessingToPipeline(True)

        return self
    
    def process(self, rgbd):
        # print(rgbd)
        # print(rgbd["inColorSync"])
        rgb_frame = rgbd["inColorSync"].getCvFrame()
        # d_frame = rgbd["inDepthSync"].getCvFrame()

        cv2.imshow("HostDisplayRGB", rgb_frame)
        # cv2.imshow("HostDisplayDepth", d_frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Detected 'q' - stopping the pipeline...")
            self.stopPipeline()


# class ZMQPub(dai.node.HostNode):
#     def build(self, rgbd_out):
#         self.link_args(rgbd_out)

#         self.context = zmq.Context()
#         self.sock = None
#         ip = "localhost"
#         port = "8081"
#         self.addr = "tcp://{}:{}".format(ip, port)

#         self.sendProcessingToPipeline(True)

#         return self
    
#     def onStart(self) -> None:
#         print("ZMQPub started")

#         self.sock = self.context.socket(zmq.PUB)
#         self.sock.connect(self.addr)

#     def onStop(self) -> None:
#         self.sock.close()
#         self.context.term()

#     def process(self, rgbd):
#         # print(rgbd["inColorSync"])
#         rgb_frame = rgbd["inColorSync"].getCvFrame()
#         d_frame = rgbd["inDepthSync"].getCvFrame()

#         cv2.imshow("HostDisplayRGB", rgb_frame)

#         mat_pb = matrix_pb2.MatProto()
#         mat_pb.rows, mat_pb.cols, _ = rgb_frame.shape
#         mat_pb.dtype = 2
#         mat_pb.data = rgb_frame.tobytes()
#         mat_pb.depth_data = d_frame.tobytes()
#         self.sock.send(mat_pb.SerializeToString())

#         key = cv2.waitKey(10)
#         if key == ord('q'):
#             print("Detected 'q' - stopping the pipeline...")
#             self.stopPipeline()


class ZMQThreadedPub(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputRGBD = self.createInput()

        self.context = None
        self.sock = None
        self.addr = None

    def onStart(self) -> None:
        print("ZMQPub started")

        self.context = zmq.Context()
        self.sock = None
        ip = "localhost"
        port = "8081"
        self.addr = "tcp://{}:{}".format(ip, port)

        self.sock = self.context.socket(zmq.PUB)
        self.sock.connect(self.addr)

    def onStop(self) -> None:
        self.sock.close()
        self.context.term()

    def run(self):
        while self.isRunning():
            try:
                rgbd = self.inputRGBD.tryGet()
            except dai.MessageQueue.QueueException:
                return # Pipeline closed
            if rgbd is not None:
                rgb_frame = rgbd.getRGBFrame().getCvFrame()
                d_frame = rgbd.getDepthFrame().getCvFrame()
                # print(rgb_frame.shape, d_frame.shape)

                # cv2.imshow("HostDisplayRGB", rgb_frame)

                mat_pb = matrix_pb2.MatProto()
                mat_pb.rows, mat_pb.cols, _ = rgb_frame.shape
                mat_pb.dtype = 2
                mat_pb.data = rgb_frame.tobytes()
                mat_pb.depth_data = d_frame.tobytes()
                self.sock.send(mat_pb.SerializeToString())


class Cam2WorldNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()
        self.outputPCL = self.createOutput()
        self.tsfm = np.eye(4)
        # self.tsfm = np.array([
        #     [1, 0, 0, 0],
        #     [0, 1, 0, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1],
        # ])

    def setTsfm(self, tsfm):
        if tsfm is None:
            self.tsfm = np.eye(4)

    def run(self):
        while self.isRunning():
            try:
                inPointCloud = self.inputPCL.get()
            except dai.MessageQueue.QueueException:
                return # Pipeline closed
            if inPointCloud is not None:
                outPointCloud = dai.PointCloudData()
                points, colors = inPointCloud.getPointsRGB()
                ones = np.ones((points.shape[0], 1))
                points = np.concatenate((points, ones), axis=1)

                updatedPoints = points.dot(self.tsfm)[:, :3]

                outPointCloud.setPointsRGB(updatedPoints, colors)
                self.outputPCL.send(outPointCloud)


class CalibrateExtrinsicsNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputRGB = self.createInput()

        self.chboard = calibrate.get_checkerboard()
        self.intrinsics = None
        self.distortion = None

        self.rotation = None
        self.translation = None

    def setCamParams(self, intrinsics, distortion):
        self.intrinsics = intrinsics
        self.distortion = distortion

    def run(self):
        while self.isRunning():
            try:
                rgbFrame = self.inputRGB.get()
            except dai.MessageQueue.QueueException:
                return
            if rgbFrame is not None:
                frame = cv2.cvtColor(rgbFrame, cv2.COLOR_BGR2GRAY)
                ret = calibrate.estimate_frame_pose(self.intrinsics, self.distortion, frame, self.checkerboard)
                if ret is not None:
                    r, t, corners, marker_ids = ret
                    self.rotation = r
                    self.translation = t
                    # rgb_frame = cv2.aruco.drawDetectedCornersCharuco(
                    #     rgb_frame, corners, marker_ids, (255, 0, 0))
                    # rgb_frame = cv2.drawFrameAxes(
                    #     rgb_frame, cam.intrinsics, cam.distortion,
                    #     r, t, 0.05)
            

# class MultiRGBDDisplay(dai.node.HostNode):
#     def build(self, rgbd_out1, rgbd_out2):
#         self.link_args(rgbd_out1, rgbd_out2)

#         self.sendProcessingToPipeline(True)

#         return self
    
#     def process(self, rgbd1, rgbd2):
#         print(rgbd1)
#         print(rgbd1["inColorSync"])
#         rgb_frame = rgbd1["inColorSync"].getCvFrame()
#         d_frame = rgbd1["inDepthSync"].getCvFrame()

#         rgb_frame2 = rgbd2["inColorSync"].getCvFrame()
#         d_frame2 = rgbd2["inDepthSync"].getCvFrame()

#         cv2.imshow("HostDisplayRGB", rgb_frame)
#         cv2.imshow("HostDisplayDepth", d_frame)

#         cv2.imshow("HostDisplayRGB2", rgb_frame2)
#         cv2.imshow("HostDisplayDepth2", d_frame2)

#         key = cv2.waitKey(1)
#         if key == ord('q'):
#             print("Detected 'q' - stopping the pipeline...")
#             self.stopPipeline()
