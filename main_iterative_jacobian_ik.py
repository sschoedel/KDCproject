from arm import Arm
from endeffector import EndEffector
from armvisualizer import ArmVisualizer
from wordfinder import wordFinder
from linalg_utils import LinalgUtils as LU

import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

def main():
    const_r = 0
    const_r_mat = np.array([[1, 0, 0, 0],
                            [0, np.cos(const_r), -np.sin(const_r), 0],
                            [0, np.sin(const_r), np.cos(const_r), 0],
                            [0, 0, 0, 1]])
    arm = Arm("6dof.urdf", const_r_mat)
    armvis = ArmVisualizer(arm)
    # armvis.startMeshcat()
    
    seconds = 2
    # joint_traj = [[t, t, t, t, t, t, t, d, d] for t, d in zip(np.linspace(0, 2*np.pi, seconds*60), np.linspace(0, 0.05, seconds*60))]
    # armvis.animateArm(joint_traj)
    
    start_thetas = [0, 0, 0, 0, 0, 0, 0, 0]
    
    goal_config = np.array([.2, .2, .2, 1, 0, 0, 0]) # x y z qw qx qy qz
    joint_traj = arm.IK(goal_config, start_thetas=start_thetas)
    
    start_p = arm.getArmTipTransform(start_thetas)[:3,-1]
    endpositions = [start_p]
    for thetas in joint_traj:
        pos = arm.getArmTipTransform(thetas)[:3,-1]
        endpositions.append(pos.copy())
    endpositions = np.array(endpositions)
    
    # print(start_thetas)
    
    # end_thetas = [1, 1, 1, 1, 1, 1, .020, .020]
    
    # goal_config = np.array([.3, -.2, .8, -0.11698, 0.07755, 0.82524, 0.54706]) # x y z qw qx qy qz
    # goal_thetas = arm.IK(goal_config)
    
    # # TODO: end_thetas = arm.getJointAngles(endeffector_pose_end)
    # joint_traj = mr.JointTrajectory(start_thetas, end_thetas, seconds, seconds*60, method=3)
    # armvis.animateArm(joint_traj)
    
    
    
    
    # player_letters = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'H', 'I', 'J', 'J', 'K', 'L', 'M', 'N', 'O', 'A', 'C', 'E', 'K']
    # finder = wordFinder(player_letters)

    # finder.findBestMove()
    # finder.findBestMove()
        
        
    # Plot solution
    fig = plt.figure(figsize=(5, 6))
    ax = fig.add_subplot(projection='3d')
    ax.scatter(goal_config[0], goal_config[1], goal_config[2], color='blue')
    ax.scatter(start_p[0], start_p[1], start_p[2], color='orange')
    ax.plot3D(endpositions[:,0], endpositions[:,1], endpositions[:,2], 'chartreuse')


    # Compute goal coordinate frame axes
    axis_length = .01
    g_goal = LU.xyzQuatToTransform(goal_config)
    goal_p = g_goal[:3,-1]
    goal_axes = np.array([(g_goal @ np.append(np.eye(3)[:,i]*axis_length, 1))[:-1] for i in range(3)])

    # Draw coordinate frames
    ax.plot3D((goal_p[0], goal_axes[0,0]), (goal_p[1], goal_axes[0,1]), (goal_p[2], goal_axes[0,2]), color='red')
    ax.plot3D((goal_p[0], goal_axes[1,0]), (goal_p[1], goal_axes[1,1]), (goal_p[2], goal_axes[1,2]), color='green')
    ax.plot3D((goal_p[0], goal_axes[2,0]), (goal_p[1], goal_axes[2,1]), (goal_p[2], goal_axes[2,2]), color='blue')

    # Compute start coordinate frame axes
    g_start = arm.getArmTipTransform(joint_traj[0])
    start_s = g_start[:3,-1]
    start_axes = np.array([(g_start @ np.append(np.eye(3)[:,i]*axis_length, 1))[:-1] for i in range(3)])

    # Draw coordinate frames
    ax.plot3D((start_p[0], start_axes[0,0]), (start_p[1], start_axes[0,1]), (start_p[2], start_axes[0,2]), color='red')
    ax.plot3D((start_p[0], start_axes[1,0]), (start_p[1], start_axes[1,1]), (start_p[2], start_axes[1,2]), color='green')
    ax.plot3D((start_p[0], start_axes[2,0]), (start_p[1], start_axes[2,1]), (start_p[2], start_axes[2,2]), color='blue')

    # set axis limits
    # maxZ = 2.25
    # minZ = maxZ - 0.02
    # maxX = 0.55
    # minX = maxX - 0.12
    # maxY = 1.12
    # minY = maxY - 0.12

    # ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)
    # ax.axis([minX, maxX, minY, maxY])

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.legend(["start pose", "goal pose 2"])
    plt.show()


if __name__ == '__main__':
    main()