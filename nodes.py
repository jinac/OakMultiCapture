import depthai as dai
import cv2

import socket
import struct
import pickle
from time import sleep

class RGBDDisplay(dai.node.HostNode):
    def build(self, rgbd_out):
        self.link_args(rgbd_out)

        self.sendProcessingToPipeline(True)

        return self
    
    def process(self, rgbd):
        print(rgbd)
        # print(rgbd["inColorSync"])
        rgb_frame = rgbd["inColorSync"].getCvFrame()
        # d_frame = rgbd["inDepthSync"].getCvFrame()

        cv2.imshow("HostDisplayRGB", rgb_frame)
        # cv2.imshow("HostDisplayDepth", d_frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Detected 'q' - stopping the pipeline...")
            self.stopPipeline()


class SocketForwarder(dai.node.HostNode):
    def build(self, rgbd_out):
        self.link_args(rgbd_out)

        self.sock = None
        self.conn = None
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        self.sendProcessingToPipeline(True)

        return self
    
    def onStart(self) -> None:
        print("SocketForwarder started")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 8080))
        self.conn = self.sock.makefile('wb')
        
    def onStop(self) -> None:
        self.sock.close()

    def process(self, rgbd):
        # print(rgbd["inColorSync"])
        rgb_frame = rgbd["inColorSync"].getCvFrame()
        # d_frame = rgbd["inDepthSync"].getCvFrame()

        # cv2.imshow("HostDisplayRGB", rgb_frame)
        _, data = cv2.imencode('.jpg', rgb_frame, self.encode_param)
        data = pickle.dumps(data, 0)
        sz = len(data)
        self.sock.sendall(struct.pack(">L", sz) + data)
        # cv2.imshow("HostDisplayDepth", d_frame)

        # key = cv2.waitKey(10)
        # if key == ord('q'):
        #     print("Detected 'q' - stopping the pipeline...")
        #     self.stopPipeline()

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