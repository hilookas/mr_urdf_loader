from __future__ import annotations
from urchin import URDF
import numpy as np
import modern_robotics as mr


def AnalyticJacobianBody(M, Blist, thetalist):
    """Computes the Analytic Jacobian for an open chain robot
    """
    Jb = mr.JacobianBody(Blist, thetalist)
    Tsb = mr.FKinBody(M, Blist, thetalist)
    Rsb = Tsb[0:3, 0:3]
    r = mr.so3ToVec(Rsb)
    norm_r = np.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    omega_r = mr.VecToso3(r)
    A = (
        np.eye(3)
        - (1 - np.cos(norm_r)) / (norm_r * norm_r) * omega_r
        + (norm_r - np.sin(norm_r)) / (norm_r**3) * omega_r @ omega_r
    )
    A_ = np.eye(6)
    A_[0:3, 0:3] = Rsb
    A_[3:6, 3:6] = Rsb
    Ja = A_ @ Jb

    return Ja


def loadURDF(urdf_name, eef_link_name=None, actuated_joint_names=None):
    robot = URDF.load(urdf_name)
    lfk = robot.link_fk()

    if actuated_joint_names is not None:
        actuated_joints = [robot.joint_map[name] for name in actuated_joint_names]
    else:
        actuated_joints = robot.actuated_joints

    Slist = []
    Glist = []
    Mlist = []
    last_CoM_M = np.eye(4)
    for joint in actuated_joints:
        child_link = robot.link_map[joint.child]

        child_M = lfk[child_link]
        child_M_R, child_M_p = mr.TransToRp(child_M)
        w = child_M_R @ np.array(joint.axis).T
        p = child_M_p
        v = -np.cross(w, p)
        if joint.joint_type == 'revolute':
            Slist.append([w[0], w[1], w[2], v[0], v[1], v[2]])
        elif joint.joint_type == 'prismatic':
            Slist.append([0, 0, 0, w[0], w[1], w[2]])
        else:
            raise Exception("Unsupport type of joint")

        # Dynamic part may be broken
        # See: https://blog.csdn.net/qq_42243147/article/details/132838572

        G = np.eye(6)
        G[0:3, 0:3] = child_link.inertial.inertia
        G[3:6, 3:6] = child_link.inertial.mass * np.eye(3)
        Glist.append(G)

        CoM_M = child_M @ child_link.inertial.origin
        Mlist.append(mr.TransInv(last_CoM_M) @ CoM_M)
        last_CoM_M = CoM_M

    if eef_link_name is not None:
        eef_link = robot.link_map[eef_link_name]
    else:
        eef_link = robot.end_links[0]
    M = lfk[eef_link]

    Slist = np.transpose(Slist)
    Blist = mr.Adjoint(mr.TransInv(M)) @ Slist
    Glist = np.array(Glist)
    Mlist.append(mr.TransInv(last_CoM_M) @ M)
    Mlist = np.array(Mlist)

    return M, Slist, Blist, Mlist, Glist, robot
