import numpy as np

class LinalgUtils():
    """
    Some linear algebra functions for building transformation matrices
    """
    @classmethod
    def adjMatrix(cls, g):
        R = g[:3,:3]
        p = g[:3,-1]
        Adj = np.zeros((6,6))
        Adj[:3,:3] = R # Top left 3x3
        Adj[3:,3:] = R # Bottom right 3x3
        Adj[:3,3:] = cls.asSkewSymmetric(p) @ R # Top right 3x3
        return Adj
        
    @classmethod
    def asSkewSymmetric(cls, vector):
        a, b, c = vector
        return np.array([[0, -c, b],
                        [c, 0, -a],
                        [-b, a, 0]])
    
    @classmethod
    def rotX(cls, r):
        return np.array([[1, 0, 0],
                            [0, np.cos(r), -np.sin(r)],
                            [0, np.sin(r), np.cos(r)]])
    @classmethod
    def rotY(cls, r):
        return np.array([[np.cos(r), 0, np.sin(r)],
                            [0, 1, 0],
                            [-np.sin(r), 0, np.cos(r)]])
    @classmethod
    def rotZ(cls, r):
        return np.array([[np.cos(r), -np.sin(r), 0],
                            [np.sin(r), np.cos(r), 0],
                            [0, 0, 1]])
        
    @classmethod
    def rotToQuat(cls, R):
        # Returns quaternion in form (q_w, q_x, q_y, q_z)
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2])/2
        if qw == 0:
            return np.array([1, 0, 0, 0])
        else:
            qx = (R[2,1] - R[1,2]) / (4*qw)
            qy = (R[0,2] - R[2,0]) / (4*qw)
            qz = (R[1,0] - R[0,1]) / (4*qw)
            return np.array([qw, qx, qy, qz])

    @classmethod
    def rotToEuler(cls, R):
        return cls.quatToEuler(cls.rotToQuat(R))

    @classmethod
    def quatToEuler(cls, q):
        # Quaternion of the form (q_w, q_x, q_y, q_z)
        # Euler in form Body 3-2-1 (ϕ, θ, ψ)
        phi = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]**2 + q[2]**2))
        theta = -np.pi/2 + 2*np.arctan2(np.sqrt(1 + 2*(q[0]*q[2] - q[1]*q[3])), np.sqrt(1 - 2*(q[0]*q[2] - q[1]*q[3])))
        psi = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1-2*(q[2]**2 + q[3]**2))
        return np.array([phi, theta, psi])
    
    @classmethod
    def eulerToQuat(cls, e):
        # Euler in form Body 3-2-1 (ϕ, θ, ψ)
        # Quaternion of the form (q_w, q_x, q_y, q_z)
        return np.array([[np.cos(e[0]/2)*np.cos(e[1]/2)*np.cos(e[2]/2) + np.sin(e[0]/2)*np.sin(e[1]/2)*np.sin(e[2]/2)],
                         [np.sin(e[0]/2)*np.cos(e[1]/2)*np.cos(e[2]/2) - np.cos(e[0]/2)*np.sin(e[1]/2)*np.sin(e[2]/2)],
                         [np.cos(e[0]/2)*np.sin(e[1]/2)*np.cos(e[2]/2) + np.sin(e[0]/2)*np.cos(e[1]/2)*np.sin(e[2]/2)],
                         [np.cos(e[0]/2)*np.cos(e[1]/2)*np.sin(e[2]/2) - np.sin(e[0]/2)*np.sin(e[1]/2)*np.cos(e[2]/2)]])
        

    @classmethod
    def homoTransMatFromDH(cls, origin, const_offset_g=np.eye(4)):
        res = np.eye(4)
        res[:-1,:-1] = cls.rotX(origin.rpy[0]) @ cls.rotY(origin.rpy[1]) @ cls.rotZ(origin.rpy[2])
        res[:-1,-1] = origin.xyz
        return res @ const_offset_g

    @classmethod
    def invTransform(cls, g):
        R = g[:3,:3]
        p = g[:3,-1]
        inv_g = np.eye(4)
        inv_g[:3,:3] = R.T
        inv_g[:3,-1] = -R.T @ p
        return inv_g

    @classmethod
    def xyzQuatToTransform(cls, xyz_quat):
        # Quaternion of the form (q_w, q_i, q_j, q_k)
        xyz = xyz_quat[:3]
        q = [xyz_quat[4], xyz_quat[5], xyz_quat[6], xyz_quat[3]]
        transform = np.array([[(1 - 2*q[1]**2 - 2*q[2]**2), (2*q[0]*q[1] - 2*q[3]*q[2]), (2*q[0]*q[2] + 2*q[3]*q[1]), xyz[0]],
                            [(2*q[0]*q[1] + 2*q[3]*q[2]), (1 - 2*q[0]**2 - 2*q[2]**2), (2*q[1]*q[2] - 2*q[3]*q[0]), xyz[1]],
                            [(2*q[0]*q[2] - 2*q[3]*q[1]), (2*q[1]*q[2] + 2*q[3]*q[0]), (1 - 2*q[0]**2 - 2*q[1]**2), xyz[2]],
                            [0, 0, 0, 1]])
        return transform