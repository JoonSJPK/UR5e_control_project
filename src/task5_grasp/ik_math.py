from __future__ import annotations

import math

import numpy as np


def calc_p(matrix):
    m = np.asarray(matrix)
    return np.array([m[0][3], m[1][3], m[2][3]])


def calc_z_axis(matrix):
    m = np.asarray(matrix)
    return np.array([m[0][2], m[1][2], m[2][2]])


def calc_r(matrix):
    m = np.asarray(matrix)
    return np.array(m[:3, :3])


def calc_jv_column(z, p):
    return np.cross(z, p)


def calc_e_mag(e):
    return math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])


def calc_error_o(R_d, R_e):
    R_e = calc_r(R_e)
    R_d = calc_r(R_d)
    R_err = np.array(R_d) @ np.array(R_e).T

    cos_phi = (np.trace(R_err) - 1.0) / 2.0
    cos_phi = np.clip(cos_phi, -1.0, 1.0)
    phi = np.arccos(cos_phi)

    axis = np.array([R_err[2, 1] - R_err[1, 2],
                     R_err[0, 2] - R_err[2, 0],
                     R_err[1, 0] - R_err[0, 1]])

    sin_phi = np.sin(phi)
    if abs(sin_phi) < 1e-9:
        return np.zeros(3)

    return (phi / (2.0 * sin_phi)) * axis


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _euler_xyz(rpy):
    return _Rx(rpy[0]) @ _Ry(rpy[1]) @ _Rz(rpy[2])


def _axis_rot(axis, theta):
    if axis[0]:
        return _Rx(theta * axis[0])
    if axis[1]:
        return _Ry(theta * axis[1])
    return _Rz(theta * axis[2])


class SOArm100Kinematics:
    LINK_PARAMS = [
        ([0.0, -0.0452, 0.0165], [1.57079, 0.0, 0.0], [0, 1, 0]),   # Rotation
        ([0.0,  0.1025,  0.0306], [1.57079, 0.0, 0.0], [1, 0, 0]),  # Pitch
        ([0.0,  0.11257, 0.028],  [-1.57079, 0.0, 0.0], [1, 0, 0]),  # Elbow
        ([0.0,  0.0052,  0.1349], [-1.57079, 0.0, 0.0], [1, 0, 0]),  # Wrist_Pitch
        ([0.0, -0.0601,  0.0],    [0.0, 1.57079, 0.0], [0, 1, 0]),  # Wrist_Roll
    ]

    def __init__(self, finger_offset):
        self.finger_offset = np.asarray(finger_offset, dtype=float)

    def link_elem_matrix(self, trans, rpy, axis, theta):
        R = _euler_xyz(rpy) @ _axis_rot(axis, theta)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = trans
        return T

    def forward_kinematics(self, q):
        T = np.eye(4)
        transforms, z, p = [], [], []
        for idx in range(5):
            trans, rpy, axis = self.LINK_PARAMS[idx]
            F = T.copy()
            F[:3, :3] = T[:3, :3] @ _euler_xyz(rpy)
            F[:3, 3] = T[:3, 3] + T[:3, :3] @ np.asarray(trans, dtype=float)
            z.append(F[:3, :3] @ np.asarray(axis, dtype=float))
            p.append(calc_p(F))
            T = T @ self.link_elem_matrix(trans, rpy, axis, q[idx])
            transforms.append(T)
        return T, transforms, z, p

    def grasp_pose(self, q):
        T, _, _, _ = self.forward_kinematics(q)
        R_jaw = calc_r(T)
        p_grasp = calc_p(T) + R_jaw @ self.finger_offset
        return p_grasp, R_jaw


def ik_step(kin, q, p_target, R_d, K_p=1.0, K_o=0.5, lambda_squared=0.08 ** 2):
    T, _transforms, z, p = kin.forward_kinematics(q)
    R_e = calc_r(T)
    p_grasp = calc_p(T) + R_e @ kin.finger_offset

    e_p = p_target - p_grasp
    e_o = calc_error_o(R_d, R_e)
    e_p_mag = calc_e_mag(e_p)
    e_o_mag = calc_e_mag(e_o)

    jv_transpose, jw_transpose = [], []
    for idx in range(5):
        jv_transpose.append(calc_jv_column(z[idx], p_grasp - p[idx]))
        jw_transpose.append(z[idx])

    jv = np.array(jv_transpose).T
    jw = np.array(jw_transpose).T
    j = np.vstack([jv, jw])  # 6x5
    j_transpose = j.T

    e = np.array([K_p * e_p[0], K_p * e_p[1], K_p * e_p[2],
                  K_o * e_o[0], K_o * e_o[1], K_o * e_o[2]])
    identity = np.eye(6)
    delta_theta = (j_transpose @ np.linalg.inv((j @ j_transpose) + (lambda_squared * identity))) @ e

    return delta_theta, e_p_mag, e_o_mag
