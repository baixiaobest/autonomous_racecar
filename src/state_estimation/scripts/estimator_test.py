from ErrorStateEKF import ErrorStateEKF as EKF
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion


if __name__=='__main__':
    init_pose = np.zeros(10)
    init_pose[6:10] = Quaternion(axis=np.array([0, 0, 1]), angle=0).elements
    init_pose[3] = 1.0
    ekf = EKF(init_pose=init_pose)

    N = 1000
    pose = np.zeros((N, 10))
    angle = np.zeros(N)

    delta_t = 0.1

    for i in range(N):
        ekf.imu_update(accel=np.array([0, 0.1, 0]), ang_vel=np.array([0, 0, 0.1]), delta_t=delta_t)
        pose[i] = ekf.get_pose()
        angle[i] = Quaternion(pose[i, 6:10]).angle

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pose[:, 0], pose[:, 1], pose[:, 2])
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-10, 10)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    fig2 = plt.figure()
    ax21 = fig2.add_subplot(211)
    ax21.plot(np.arange(0, N), angle)
    ax22 = fig2.add_subplot(212)
    ax22.plot(np.arange(0, N), pose[:, 3]** 2 + pose[:, 4] ** 2)
    plt.show()
