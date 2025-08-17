import depthai as dai
import cv2

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

class MultiRGBDDisplay(dai.node.HostNode):
    def build(self, rgbd_out1, rgbd_out2):
        self.link_args(rgbd_out1, rgbd_out2)

        self.sendProcessingToPipeline(True)

        return self
    
    def process(self, rgbd1, rgbd2):
        print(rgbd1)
        print(rgbd1["inColorSync"])
        rgb_frame = rgbd1["inColorSync"].getCvFrame()
        d_frame = rgbd1["inDepthSync"].getCvFrame()

        rgb_frame2 = rgbd2["inColorSync"].getCvFrame()
        d_frame2 = rgbd2["inDepthSync"].getCvFrame()

        cv2.imshow("HostDisplayRGB", rgb_frame)
        cv2.imshow("HostDisplayDepth", d_frame)

        cv2.imshow("HostDisplayRGB2", rgb_frame2)
        cv2.imshow("HostDisplayDepth2", d_frame2)

        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Detected 'q' - stopping the pipeline...")
            self.stopPipeline()