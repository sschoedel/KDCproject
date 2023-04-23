from arm import Arm
from endeffector import EndEffector
from armvisualizer import ArmVisualizer
from wordfinder import wordFinder
from IK import invPose
from linalg_utils import LinalgUtils as LU
from meshcat_utils import Arrow

import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

def main():
    arm = Arm("6dof.urdf", ee=EndEffector("TWOFINGERS"))
    armvis = ArmVisualizer(arm)
    # armvis.startMeshcat()
    
    seconds = 2
    # joint_traj = [[t, t, t, t, t, t, t, d, d] for t, d in zip(np.linspace(0, 2*np.pi, seconds*60), np.linspace(0, 0.05, seconds*60))]
    # armvis.animateArm(joint_traj)
    
    start_thetas = [0, 0, 0, 0, 0, 0]
    
    goal_config = np.array([.4, .4, .2, 1, 0, 0, 0]) # x y z qw qx qy qz
    # end_thetas, wrist_pos = invPose(goal_config, verbosity=1)
    end_thetas = arm.IK(goal_config, verbosity=1)[-1] # Take last set of joint angles in joint_trajectory
    
    ee_pose = arm.getArmTipTransform(end_thetas)
    print(ee_pose)
    
    # print(start_thetas)
    
    # end_thetas = [1, 1, 1, 1, 1, 1, .020, .020]
    
    # goal_config = np.array([.3, -.2, .8, -0.11698, 0.07755, 0.82524, 0.54706]) # x y z qw qx qy qz
    # goal_thetas = invPose(goal_config)
    
    # # TODO: end_thetas = arm.getJointAngles(endeffector_pose_end)
    
    # joint_traj = mr.JointTrajectory(start_thetas, end_thetas, seconds, seconds*60, method=3)
    # armvis.animateArm(joint_traj)
    
    # import meshcat.geometry as geom
    # import meshcat.transformations as tf
    
    # armvis.vis["sphere_tip"].set_object(geom.Sphere(.02), 
    #                                 geom.MeshLambertMaterial(
    #                                     color=0xff22dd,
    #                                     reflectivity=0.8))
    
    # armvis.vis["sphere_wrist"].set_object(geom.Sphere(.02), 
    #                                 geom.MeshLambertMaterial(
    #                                     # color=0x002255,
    #                                     color=0xff0000,
    #                                     reflectivity=0.8))
    
    # wrist_p = [wrist_pos[0][0], wrist_pos[1][0], wrist_pos[2][0]]
    # print(f"wrist_pos: {wrist_p}")
    
    # armvis.vis["sphere_tip"].set_transform(tf.translation_matrix([.4, .4, .2]))
    # armvis.vis["sphere_wrist"].set_transform(tf.translation_matrix(wrist_p))
    
    # quat_arrow_tip = Arrow(armvis.vis, "quadarrow_tip", color=0xff0000)
    # t = LU.xyzQuatToTransform(goal_config)
    # # print(t)
    # quat_arrow_tip.set_transform(t)
    
    # player_letters = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'H', 'I', 'J', 'J', 'K', 'L', 'M', 'N', 'O', 'A', 'C', 'E', 'K']
    # finder = wordFinder(player_letters)

    # finder.findBestMove()
    # finder.findBestMove()
        
if __name__ == '__main__':
    main()