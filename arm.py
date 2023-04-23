from linalg_utils import LinalgUtils as LU
from urdf_parser_py.urdf import URDF
from endeffector import EndEffector, TwoFingersEE

import numpy as np


class Arm():
    """
    Small 6-DOF arm with swappable end effector
    """
    
    # gs should change with thetas
    gs = {} # Each joint transformation at current joint angles
    # gs_home and xis shouldn't change after initialization
    gs_home = {} # Each joint transformation in home position
    xis = {} # Each xi in home position
    robot_urdf = 0
    link_to_joint_name = {} # Map between link and joint names
    
    logfile_path = "arm_log.txt"
    logfile = 0
    
    def __init__(self, urdf_file_path, ee=None, const_offset_g=np.eye(4)):
        np.set_printoptions(precision=3, suppress=True)
        self.logfile = open(self.logfile_path, "w")
        self.createArmFromURDF(urdf_file_path, const_offset_g)
        self.ee = ee
    
    def __del__(self):
        self.logfile.close()
    
    
    def IK(self, goal_config_xyz_quat, start_thetas=None, use_damped_least_squares=False, verbosity=0):
        """
        Compute a trajectory that moves the arm from the start configuration to the goal 
        configuration using the jacobian pseudoinverse iterative method

        Args:
            goal_config (list[float]): end effector pose in the form 
                        [x, y, z, q_w, q_i, q_j, q_k].
            start_thetas (list[float], optional): list of n initial joint angles.
                        Defaults to zeros.
            use_damped_least_squares (bool, optional): Use damped least squares to solve
                        for joint angles. Defaults to False.
        """
        
        if start_thetas is None:
            start_thetas = [0]*len(self.xis)
        
        # Convert start thetas to transformation matrix
        g_os = self.getArmTipTransform(start_thetas) # transform from origin to start
        
        # Convert goal config to transformation matrix
        # transform from origin to destination
        g_od = LU.xyzQuatToTransform(goal_config_xyz_quat) # takes quat in form q_w, q_i, q_j, q_k
        
        # Get transform between start and destination        
        g_sd = LU.invTransform(g_os) @ g_od
        
        pose_diff_start = np.hstack([g_sd[:3,-1], LU.rotToEuler(g_sd[:3,:3])])
        if verbosity > 0: print(f"pose_diff_start: {pose_diff_start}")
        
        # Convert pose_diff_start to origin frame
        pose_diff_origin_frame = LU.adjMatrix(g_os) @ pose_diff_start
        
        # Initialize loop variables
        joint_trajectory = [np.array(start_thetas)]
        curr_thetas = start_thetas.copy()
        lamb = 0.005
        h = 0.02 # Step size
        tol = 1e-5 # Stop condition tolerance
        converged = False
        i = 0
        max_iters = 10000
        while not converged and i < max_iters:
            J = self.getJacobian(curr_thetas)
            if use_damped_least_squares:
                delta_thetas = h * J.T @ np.linalg.inv(J @ J.T + lamb**2*np.eye(6)) @ (pose_diff_origin_frame)
            else:
                delta_thetas = h * J.T @ np.linalg.inv(J @ J.T) @ (pose_diff_origin_frame)
            curr_thetas += delta_thetas
            joint_trajectory.append(curr_thetas.copy())
            
            # Convert current thetas to current end effector transform
            g_oc = self.getArmTipTransform(curr_thetas)
            
            # Get transform between current (computed in loop) and destination (computed before loop)
            g_cd = LU.invTransform(g_oc) @ g_od
            
            # Convert g_sd to velocity representation
            pose_diff_curr_frame = np.hstack([g_cd[:3,-1], LU.rotToEuler(g_cd[:3,:3])])
            
            # Convert pose_diff_curr_frame to origin frame
            pose_diff_origin_frame = LU.adjMatrix(g_oc) @ pose_diff_curr_frame
            
            # Check stopping conditions
            if np.linalg.norm(pose_diff_origin_frame) < tol:
                converged = True
            
            if verbosity > 0:
                print(f"pose_diff: {pose_diff_origin_frame}, \
                        norm(pd): {np.linalg.norm(pose_diff_origin_frame)}, \
                        norm(dist): {np.linalg.norm(goal_config_xyz_quat[:3] - g_oc[:3,-1])}")
            i += 1
        
        return joint_trajectory
    
    def getArmTipTransform(self, thetas):
        # Update exponential twist compositions for each joint so we can convert each xi to xi' (xip (prime))
        e_matrices = [self.xiThetaExponential(self.xis[joint_name], theta) for joint_name, theta in zip(self.xis, thetas)]
        
        # Make sure robot has at least one joint
        if len(self.xis) == 0:
            print(f"ERROR: No joints in arm")
            return np.zeros((6,0))
        
        # Create transformation matrices for each joint's new twist location in spatial frame
        es_composed = [0]*len(self.xis)
        
        # Transform for exponential twist of base joint is identity
        es_composed[0] = np.eye(4)
        
        # Use previous transforms to compute next twist transforms
        i = 0
        for joint_name in self.xis:
            if i > 0:
                es_composed[i] = es_composed[i-1] @ e_matrices[i-1]
                if joint_name == "ee_joint": # TODO: make it so users don't have to name their end effector joint "ee_joint"
                    break
            i += 1

        return es_composed[i] @ e_matrices[i] @ self.gs_home["ee_joint"] @ self.ee.getFrameToTipTransform()
        
    def getJacobian(self, thetas):
        """
        Compute 6xn Jacobian for current joint configuration.
        The 6 element output is of the form V^s_{st}, which is the column vector
        [v_s; ω_s] where v_s = linear velocities and ω_s = rotational velocities
        of the center of the tool frame in spatial frame coords.
        
        Args:
            thetas (list[float]): n element list of angles for each joint

        Returns:
            np.array: 6xn spatial manipulator Jacobian J^s_{st}
        """
        # Update exponential twist compositions for each joint so we can convert each xi to xi' (xip (prime))
        e_matrices = [self.xiThetaExponential(self.xis[joint_name], theta) for joint_name, theta in zip(self.xis, thetas)]
        
        # Make sure robot has at least one joint
        if len(self.xis) == 0:
            print(f"ERROR: No joints in arm")
            return np.zeros((6,0))
        
        # Create transformation matrices for each joint's new twist location in spatial frame
        es_composed = [0]*len(self.xis)
        
        # First exponential twist matrix is identity
        es_composed[0] = np.eye(4)
        
        # Use previous transforms to compute next twist transforms
        for i in range(1, len(self.xis)):
            es_composed[i] = es_composed[i-1] @ e_matrices[i-1]

        # self.compute_all_joint_poses(thetas)
        
        # Compute spatial manipulator Jacobian
        xi_ps = [0]*len(self.xis)
        for i, joint_name in enumerate(self.xis):
            xi_ps[i] = LU.adjMatrix(es_composed[i]) @ self.xis[joint_name]
        
        Js = np.vstack(xi_ps).T
        return Js
    
    """
    The following three functions offer support for important
    linear algebra subroutines used in the screw theory formulation
    """
    def xiThetaExponential(self, xi, theta):
        v, w = xi[0:3], xi[3:6]
        if w[0] == 0 and w[1] == 0 and w[2] == 0:
            res = np.eye(4)
            res[:-1,-1] = theta*v
            return res
        else:
            w_ss = LU.asSkewSymmetric(w)
            e_w_ss_theta = np.eye(3) + w_ss * np.sin(theta) + w_ss @ w_ss * (1 - np.cos(theta))
            top_right = (np.eye(3) - e_w_ss_theta) @ (w_ss @ v) + w.reshape((3,1)) @ w.reshape((1,3)) @ v * theta
            return np.vstack([np.hstack([e_w_ss_theta, top_right.reshape(3,1)]), [0,0,0,1]])
    
    def computeXiBar(self, w, q):
        return np.append(-np.cross(w, q), w)

    def computeXiBarPrismatic(self, axis):
        return np.append(axis, [0, 0, 0])
    
    """
    Use urdf_parser_py to create an internal representation of
    the arm's joint locations, dimensions, and 3D model file paths
    """
    def createArmFromURDF(self, urdf_file_path, const_offset_g=np.eye(4), verbosity=0):
        # const_offset_g is a constant transformation difference between URDF
        # frame and desired model frame in meshcat and is defined by the user
        
        # Get robot info from URDF
        self.robot_urdf = URDF.from_xml_file(urdf_file_path)
        
        # Create gs_home for base link
        self.gs_home[self.robot_urdf.get_root()] = const_offset_g
        
        # Convert URDF robot info to screw formulation
        for i, joint_name in enumerate(self.robot_urdf.joint_map):
            axis = [0.0, 0.0, 0.0]
            if self.robot_urdf.joint_map[joint_name].axis:
                axis = const_offset_g[:3,:3] @ self.robot_urdf.joint_map[joint_name].axis
            origin = self.robot_urdf.joint_map[joint_name].origin
            # Build transformation matrix between prev and current joints
            g = LU.homoTransMatFromDH(origin, const_offset_g)
            # Aggregate previous transformations for joints in the same chain
            # Make sure to only transform on joints from the same chain in the robot tree
            if i > 0:
                parent_link_name = self.robot_urdf.joint_map[joint_name].parent
                # [0][1] means taking first parent joint/link pair name in list and then taking name of the parent joint
                parent_joint_name = self.robot_urdf.parent_map[parent_link_name][0][0]
                self.gs_home[joint_name] = self.gs_home[parent_joint_name] @ g
            else:
                self.gs_home[joint_name] = g
            
            # Compute xi_bar using joint axis and homogeneous
            # transformation matrix information
            # Get ith joint's axis of rotation in base frame
            w = self.gs_home[joint_name][:-1,:-1] @ axis
            # Get ith joint's origin
            q = self.gs_home[joint_name][:-1,-1]
            if self.robot_urdf.joint_map[joint_name].type == "prismatic":
                if verbosity > 0: print("found prismatic joint")
                self.xis[joint_name] = self.computeXiBarPrismatic(w)
            elif self.robot_urdf.joint_map[joint_name].type in ["revolute", "fixed"]:
                if verbosity > 0: print("found revolute joint")
                self.xis[joint_name] = self.computeXiBar(w, q)
                # print(f"self.xis[{joint_name}]: {self.xis[joint_name]}, type: {self.robot_urdf.joint_map[joint_name].type}")
            else:
                print("Error: unknown joint type")
        
        # Copy home state transforms to current arm state transforms
        for g_home in self.gs_home:
            self.gs[g_home] = self.gs_home[g_home]
        
        # Create map between link and joint names
        for link_name in self.robot_urdf.child_map:
            if self.robot_urdf.child_map[link_name]:
                children_sets = self.robot_urdf.child_map[link_name]
                # Second child is link first child is joint
                for children in children_sets:
                    self.link_to_joint_name[children[1]] = children[0]
            
    """
    Functions for computing joint locations
    """
    def aggregateParentTransforms(self, mats, g_home):
        """
        Uses mats to transform between base and each joint frame while preserving
        relative transforms between branches of the robot (separate branches aren't
        transformed on each other)

        Args:
            mats (list): list of 4x4 homogeneous transformation matrices
        """
        pass
        
    def computeJointTransforms(self, thetas, ee_thetas=None):
        """Compute transforms gs for each joint

        Args:
            thetas (list[float]): joint angles (for revolute) or displacements (for prismatic)
        """
        
        if self.ee != None:
            if ee_thetas != None:
                finger_theta = self.ee.setOpenDist(ee_thetas)
                thetas = np.append(thetas, np.array([finger_theta, finger_theta]))
            else:
                print("Error in computeJointTransforms: End effector exists but no end effector parameters were provided")
        elif len(thetas) < len(self.robot_urdf.joints):
            print(f"Error: robot has {len(self.robot_urdf.joints)} joints but only {len(thetas)} were provided")
        
        theta_index = 0
        temp_gs = {}
        for i, joint in enumerate(self.robot_urdf.joints):
            # Compute e^𝜉θ (fixed joints don't have a corresponding theta)
            if joint.type == "fixed":
                e_mat = self.xiThetaExponential(self.xis[joint.name], 0)
            else:
                e_mat = self.xiThetaExponential(self.xis[joint.name], thetas[theta_index])
                theta_index += 1
            # Apply screw transforms for this joint (e^{xi_0*\theta_0}*e^{xi_1*\theta_1}*...*e^{xi_i*\theta_i}*gs_home)
            if i == 0:
                temp_gs[joint.name] = e_mat
                self.gs[joint.name] = e_mat @ self.gs_home[joint.name]
            else:
                parent_link_name = self.robot_urdf.joint_map[joint.name].parent
                # [0][0] means taking first parent joint/link pair name in list and then taking name of the parent joint
                parent_joint_name = self.robot_urdf.parent_map[parent_link_name][0][0]
                temp_gs[joint.name] = temp_gs[parent_joint_name] @ e_mat
                self.gs[joint.name] = temp_gs[joint.name] @ self.gs_home[joint.name]
    
    def __str__(self):
        np.set_printoptions(precision=3, suppress=True)
        print("gs_home:", file=self.logfile)
        for g in self.gs_home:
            print(g, file=self.logfile)
        print("joint names:", file=self.logfile)
        for joint_name in self.robot_urdf.joint_map:
            print(joint_name, file=self.logfile)
        print("link names:", file=self.logfile)
        for link_name in self.robot_urdf.link_map:
            print(link_name, file=self.logfile)
        return ""
    