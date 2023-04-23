import numpy as np
from linalg_utils import LinalgUtils as LU

# inputs: xEnd, yEnd, zEnd, alpha, beta, gamma

class Vector(object):
    def __init__(self, xEnd, yEnd, zEnd, xBegin, yBegin, zBegin, length):
        self.distance = length
        self.xDir = (xEnd - xBegin)/self.distance
        self.yDir = (yEnd - yBegin)/self.distance
        self.zDir = (zEnd - zBegin)/self.distance
        
    def get_direction(self):
        return [self.xDir, self.yDir, self.zDir]

def invPose(goal_pose, verbosity=0):
    """
    Analytically compute joint angles given end effector goal pose
    """
    lenEnd = 0.1343
    lengths = np.array([0.134, 0.30032, 0.297])
    d2 = 0.04537
    # length1diag = 0.30373
    d2_1 = d2 * lengths[1] / (lengths[2] + lenEnd)
    d2_2 = d2 - d2_1
    
    length1diag = np.sqrt(lengths[1]**2 + d2_1**2)
    length2diag = np.sqrt((lengths[2]+lenEnd)**2 + d2_2**2)
    
    xEnd, yEnd, zEnd = goal_pose[:3]
    # alpha, beta, gamma = LU.quatToEuler(goal_pose[3:])
    gamma, beta, alpha = LU.quatToEuler(goal_pose[3:])
    # xSw = -lenEnd*np.sin(beta) + xEnd
    # ySw = lenEnd*np.cos(beta)*np.sin(alpha) + yEnd
    # zSw = -lenEnd*np.cos(alpha)*np.cos(beta) + zEnd
    transform0End = np.array([[np.cos(beta)*np.cos(gamma), -np.cos(beta)*np.sin(gamma), np.sin(beta), xEnd],
                              [np.cos(alpha)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha)*np.sin(beta), np.cos(alpha)*np.cos(gamma) - np.sin(alpha)*np.sin(beta)*np.sin(gamma), -np.cos(beta)*np.sin(alpha), yEnd],
                              [-np.cos(alpha)*np.cos(gamma)*np.sin(beta) + np.sin(alpha)*np.sin(gamma), np.cos(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha), np.cos(alpha)*np.cos(beta), zEnd],
                              [0, 0, 0, 1]])
    transformEndSw = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, -lenEnd],
                               [0, 0, 0, 1]])
    transform0Sw = transform0End @ transformEndSw

    # back up from end position to spherical joint position
    sphericalPos = np.vstack(transform0Sw[:-1,3])
 
    # xSw = sphericalPos[0]
    # ySw = sphericalPos[1]
    # zSw = sphericalPos[2]
    xSw = xEnd
    ySw = yEnd
    zSw = zEnd
    
    hSw = zSw - lengths[0]
    
    # print(f"xSw: {xSw}, ySw: {ySw}, zSw: {zSw}")
    # print(f"np.sqrt(xSw**2 + ySw**2 + hSw**2): {np.sqrt(xSw**2 + ySw**2 + hSw**2)}")
    # print(f"sum(lengths[1:]): {sum(lengths[1:]) + lenEnd}")

 
    if np.sqrt(xSw**2 + ySw**2 + hSw**2) > sum(lengths[1:]) + lenEnd:
        print("Desired position and orientation not in workspace.")
    else:
        RSw = np.sqrt(xSw**2 + ySw**2)
        # rSw = np.sqrt(hSw**2 + RSw**2)
        alpha1 = np.arcsin(d2/RSw)
        
        parallel_armlen = np.sqrt(RSw**2 - d2**2)
        
        rSw = np.sqrt(hSw**2 + parallel_armlen**2)
        alpha2 = np.arcsin(hSw/rSw)
        
        # First three joint angles responsible for placing end effector
        # Using law of cosines:
        theta1 = np.arctan2(ySw, xSw) + alpha1
        e = lenEnd
        # Home position is vertical so all thetas relative to z axis (np.pi/2 - angle)
        theta2 = np.pi/2 - ( np.arccos((lengths[1]**2 - (lengths[2]+e)**2 + rSw**2)/(2*lengths[1]*rSw)) + alpha2 )
        theta3 = -( np.pi - np.arccos(((lengths[2]+e)**2 + lengths[1]**2 - rSw**2)/(2*lengths[1]*(lengths[2]+e))) )
        # # Final three joint angles specify rotation only
        # # Multi stuff:
        # dirEnd = Vector(xEnd, yEnd, zEnd, xSw, ySw, zSw, lenEnd)
        # rElbow = lengths[1] * np.cos(theta2)
        # xElbow = rElbow * np.cos(theta1)
        # yElbow = rElbow * np.sin(theta1)
        # zElbow = lengths[0] + lengths[1] * np.sin(theta2)
        # dirForearm = Vector(xEnd, yEnd, zEnd, xElbow, yElbow, zElbow, lengths[2])
        # # print(dirForearm.get_direction())
        
        # remove list att from th 1 2 and 3
        theta1 = float(theta1)
        theta2 = float(theta2)
        theta3 = float(theta3)


        # Using ZYZ rotation matrix:
        theta4 = np.pi - np.arctan2(np.sqrt(1-np.cos(beta)**2), np.cos(beta))
        theta5 = np.arctan2(np.sin(alpha)*np.sin(beta), np.cos(alpha)*np.sin(beta))
        theta6 = np.pi - np.arctan2(np.sin(beta)*np.sin(gamma), -np.cos(gamma)*np.sin(beta))
        
        theta4 = 0
        theta5 = 0
        theta6 = 0
        
        if verbosity > 0:
            print("IK solution:")
            print(f"\ngoal pose: {goal_pose}")
            print(f'\nxSw: {xSw}, ySw: {ySw}, zSw: {zSw}')
            print(f'\nAngles in radians:\ntheta1: {theta1}\ntheta2: {theta2}\ntheta3: {theta3}\ntheta4: {theta4}\ntheta5: {theta5}\ntheta6: {theta6}')
        if verbosity > 1:
            print(f'\nAngles in degrees:\ntheta1: {theta1*180/np.pi}\ntheta2: {theta2*180/np.pi}\ntheta3: {theta3*180/np.pi}\ntheta4: {theta4*180/np.pi}\ntheta5: {theta5*180/np.pi}\ntheta6: {theta6*180/np.pi}')

        thetas = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
        # thetas = thetas * 180/np.pi
  
        # #recalculate forward kinematics to compare
        # jointPos, jointRot, _ = fk.updateMatrices(thetas)
        # print(f"jointPos[5]: {jointPos[5]}")
        # print(f"jointPos[6]: {jointPos[6]}")
        # print(f"jointRot[5]: {jointRot[5]}")		# todo: otolpos and toolrot don't match with original values
        # print(f"jointRot[6]: {jointRot[6]}")
        # print(f"\ntransform0End: {transform0End}")
        # print(f"xError: {xEnd - jointPos[5][0]}")
        # print(f"xError: {yEnd - jointPos[5][1]}")
        # print(f"xError: {zEnd - jointPos[5][2]}")
    return thetas, sphericalPos


# x, y, z, qw, qx, qy, qz
# invPose([300, 250, 400, 1, 0, 0, 0], verbosity=1)