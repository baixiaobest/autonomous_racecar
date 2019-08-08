import numpy as np
from pyquaternion import Quaternion
import numpy.linalg as LA
import pprint

pp = pprint.PrettyPrinter(indent=2)

class ErrorStateEKF:
    '''
    Initialize the filter.
    Initial pose consists of position, velocity and quaternion.
    Error covariance consists of error of position, velocity and angular velocity.
    '''
    def __init__(self, init_pose=None, error_cov=None, var_imu_f=1, var_imu_w=1):
        if init_pose is None:
            init_pose = np.zeros(10)
            init_pose[6] = 1

        if error_cov is None:
            error_cov = np.zeros((9, 9))

        self.pose = init_pose
        self.error_cov = error_cov
        self.var_imu_f = var_imu_f
        self.var_imu_w = var_imu_w
        self.motion_jac = np.zeros([9, 6])
        self.motion_jac[3:, :] = np.identity(6)

    '''Return a skew symmetric matrix of a 3-D vector.'''
    def _skew_symmetric(self, vec):
        return np.array([
            [0, -vec[2], vec[1]],
            [vec[2], 0, -vec[0]],
            [-vec[1], vec[0], 0]])

    '''
    Update the pose with imu input.
    Required linear acceleration and angular velocity.
    '''
    def imu_update(self, accel, ang_vel, delta_t):
        p_prev = self.pose[0:3]
        v_prev = self.pose[3:6]
        q_prev = self.pose[6:10]
        quaternion_prev = Quaternion(q_prev[0], q_prev[1], q_prev[2], q_prev[3])

        # 1. Update state with IMU inputs.
        p_check = p_prev + delta_t * v_prev\
                + (delta_t ** 2) / 2 * quaternion_prev.rotate(accel)

        v_check = v_prev + delta_t * quaternion_prev.rotate(accel)

        # Convert angular vector into angle axis. Then apply to quaternion.
        angle_vec = ang_vel * delta_t
        angle_vec_norm = LA.norm(angle_vec)
        quaternion_check = quaternion_prev
        if angle_vec_norm != 0.0:
            delta_quaternion = Quaternion(axis=angle_vec, angle=angle_vec_norm)
            quaternion_check = quaternion_check * delta_quaternion

        self.pose[0:3] = p_check
        self.pose[3:6] = v_check
        self.pose[6:10] = quaternion_check.elements

        # 2. Linearize motion model and compute Jacobians.
        F = np.identity(9)
        F[0:3, 3:6] = np.identity(3) * delta_t
        F[3:6, 6:9] = -1 * self._skew_symmetric(quaternion_prev.rotate(accel)) * delta_t

        # 3. Propagate uncertainty.
        Q = np.identity(6)
        Q[0:3, 0:3] = delta_t ** 2 * self.var_imu_f * np.identity(3)
        Q[3:6, 3:6] = delta_t ** 2 * self.var_imu_w * np.identity(3)
        error_cov = F @ self.error_cov @ F.T + self.motion_jac @ Q @ self.motion_jac.T

    def get_pose(self):
        return self.pose
        
