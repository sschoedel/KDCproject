import numpy as np

class TwoFingersEE():
    frame_to_tip_dist = np.array([0.00474, 0, 0.0712]) # In meters
    frame_to_tip_transform = np.array([[1, 0, 0, frame_to_tip_dist[0]],
                                       [0, 1, 0, frame_to_tip_dist[1]],
                                       [0, 0, 1, frame_to_tip_dist[2]],
                                       [0, 0, 0, 1]])
    open_dist = 0
    finger_dist = open_dist/2
    
    def __init__(self, min_open_dist=0, max_open_dist=0.04):
        self.min_open_dist = min_open_dist
        self.max_open_dist = max_open_dist
        
    def setOpenDist(self, open_dist):
        if open_dist > self.max_open_dist:
            open_dist = self.max_open_dist
        if open_dist < self.min_open_dist:
            open_dist = self.min_open_dist
        self.open_dist = open_dist
        self.finger_dist = open_dist/2
        return self.finger_dist


class SuctionEE():
    frame_to_tip_dist = np.array([0, 0, 0.1]) # In meters
    frame_to_tip_transform = np.array([[1, 0, 0, frame_to_tip_dist[0]],
                                       [0, 1, 0, frame_to_tip_dist[1]],
                                       [0, 0, 1, frame_to_tip_dist[2]],
                                       [0, 0, 0, 1]])
    
    
EE_TYPES = {"TWOFINGERS": TwoFingersEE(),
            "SUCTION": SuctionEE()}
    
class EndEffector():
    """
    End effector class for controlling and receiving data from
    different end effectors
    
    End effector types:
    two-finger gripper: "TWOFINGERS"
    suction cup: "SUCTION"
    """
    
    def __init__(self, ee_type="TWOFINGERS"):
        self.ee_type_name = ee_type
        self.ee = EE_TYPES[ee_type]
        
    def setOpenDist(self, open_dist):
        return self.ee.setOpenDist(open_dist)

    def getFrameToTipDist(self):
        return self.ee.frame_to_tip_dist
    
    def getFrameToTipTransform(self):
        return self.ee.frame_to_tip_transform
    
    def __str__(self):
        return "<end effector: " + self.ee_type + ">"
    
    
    