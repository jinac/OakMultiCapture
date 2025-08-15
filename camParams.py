import numpy as np

class CamData():
    def __init__(self, filepath):
        self.intrinsics = np.eye(4)
        self.distortion = np.zeros((1,5))
        self.world_to_cam = np.eye(4)
        self.cam_to_world = np.eye(4)
        self.rvec = None
        self.tvec = None

    def load(self, filepath):
        with np.load(filepath) as data:
            for k in data.files:
                setattr(self, k, data[k])

    def save(self, filepath):
        np.savez(filepath,
                 intrinsics=self.intrinsics,
                 distortion=self.distortion,
                 world_to_cam=self.world_to_cam,
                 cam_to_world=self.cam_to_world,
                 rvec=self.rvec,
                 tvec=self.tvec)