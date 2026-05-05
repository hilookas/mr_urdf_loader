"""Task-Space Impedance Control with Inertia Shaping — Space Frame

Desired dynamics:  M_d·ẍ + D·ẋ + K·e = 0

Implementation:
  1. F_s = K·e_s - D·V_s           (spring-damper wrench, space frame)
  2. a_cmd = M_d⁻¹ · F_s           (desired geometric acceleration)
  3. q̈_cmd = J_g⁺ · (a_cmd - J̇_g·q̇)  (map to joint acceleration)
  4. τ = InverseDynamics(q, q̇, q̈_cmd)  (computed torque)

Set Md = None to disable inertia shaping (falls back to passive K-D control).
"""

import pybullet as p
import time
import pybullet_data
import numpy as np
import modern_robotics as mr
from mr_urdf_loader import loadURDF

np.set_printoptions(precision=6, suppress=True)


def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def orientation_error_space(R_d, R):
    """Orientation error in the space frame: R_d = exp([e])·R."""
    return mr.so3ToVec(mr.MatrixLog3(R_d @ R.T))


def spatial_to_geometric_jacobian(Js, p_ee):
    Jg = np.zeros_like(Js)
    Jg[0:3, :] = Js[0:3, :]
    Jg[3:6, :] = Js[3:6, :] - skew(p_ee) @ Js[0:3, :]
    return Jg


def compute_Jdot_qdot_geo(Slist, M_home, q, q_dot, Jg, eps=1e-6):
    """Numerically compute J̇_g·q̇ via finite difference."""
    q_pert = q + eps * q_dot
    T_pert = mr.FKinSpace(M_home, Slist, q_pert)
    Js_pert = mr.JacobianSpace(Slist, q_pert)
    Jg_pert = spatial_to_geometric_jacobian(Js_pert, T_pert[0:3, 3])
    return (Jg_pert @ q_dot - Jg @ q_dot) / eps


def pos_orn_to_T(pos, orn):
    T = np.eye(4)
    T[0:3, 3] = np.array(pos)
    T[0:3, 0:3] = np.reshape(p.getMatrixFromQuaternion(orn), (3, 3))
    return T


# ===================== Robot Setup =====================
urdf_name = "3DoF.urdf"
M_home, Slist, Blist, Mlist, Glist, robot = loadURDF(urdf_name)

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robotID = p.loadURDF(urdf_name, [0, 0, 0], [0, 0, 0, 1], useFixedBase=1,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)

numJoints = p.getNumJoints(robotID)
for i in range(numJoints):
    p.setJointMotorControl2(robotID, i, p.VELOCITY_CONTROL,
                            targetVelocity=0, force=0)

timeStep = 1 / 240.0
p.setTimeStep(timeStep)
p.setRealTimeSimulation(False)

# ===================== Initial Configuration =====================
q_init = [0.1, 0.1, 0.0]
for i, qi in enumerate(q_init):
    p.resetJointState(robotID, i + 1, qi)

# ===================== Desired Pose =====================
q_target = np.array([0.3, 0.5, -0.3])
T_desired = mr.FKinSpace(M_home, Slist, q_target)
p_desired = T_desired[0:3, 3].copy()
R_desired = T_desired[0:3, 0:3].copy()

print(f"Desired EE position : {p_desired}")
print(f"(from joint config  : {q_target})")
p.addUserDebugPoints([p_desired.tolist()], [[1, 0, 0]], pointSize=15)

# ===================== Impedance Parameters (space frame) =====================
Kp = np.diag([1000.0, 500.0, 500.0])
Ko = np.diag([80.0, 50.0, 50.0])
Dp = np.diag([120.0, 100.0, 100.0])
Do = np.diag([12.0, 10.0, 10.0])

K_s = np.block([[Ko, np.zeros((3, 3))],
                [np.zeros((3, 3)), Kp]])
D_s = np.block([[Do, np.zeros((3, 3))],
                [np.zeros((3, 3)), Dp]])

# Desired inertia (space frame). Set to None for passive K-D control.
Md = np.diag([2.0, 2.0, 2.0, 5.0, 5.0, 5.0])
Md_inv = np.linalg.inv(Md) if Md is not None else None

g = np.array([0, 0, -9.8])

# ===================== Control Loop =====================
step = 0

while p.isConnected():
    jointStates = p.getJointStates(robotID, [1, 2, 3])
    q = np.array([js[0] for js in jointStates])
    q_dot = np.array([js[1] for js in jointStates])

    # ===========================================================
    #  Common: space-frame FK, errors, Jacobian
    # ===========================================================
    mr_T = mr.FKinSpace(M_home, Slist, q)
    R = mr_T[0:3, 0:3]
    pos = mr_T[0:3, 3]

    e_s = np.concatenate([
        orientation_error_space(R_desired, R),
        p_desired - pos,
    ])

    mr_Js = mr.JacobianSpace(Slist, q)
    mr_Jg = spatial_to_geometric_jacobian(mr_Js, pos)
    Vg = mr_Jg @ q_dot

    F_s = K_s @ e_s - D_s @ Vg                             # wrench (space)

    # ===========================================================
    #  MR: compute joint torques
    # ===========================================================
    if Md_inv is not None:
        a_cmd = Md_inv @ F_s
        Jdot_qdot = compute_Jdot_qdot_geo(Slist, M_home, q, q_dot, mr_Jg)
        J_pinv = np.linalg.pinv(mr_Jg)
        qdd_cmd = J_pinv @ (a_cmd - Jdot_qdot)
        mr_tau = mr.InverseDynamics(q, q_dot, qdd_cmd, g, np.zeros(6),
                                    Mlist, Glist, Slist)
    else:
        R_block = np.zeros((6, 6))
        R_block[0:3, 0:3] = R
        R_block[3:6, 3:6] = R
        Ftip = R_block.T @ F_s
        mr_tau = mr.InverseDynamics(q, q_dot, np.zeros(3), g, Ftip,
                                    Mlist, Glist, Slist)

    # ===========================================================
    #  PB: same algorithm, pybullet functions
    # ===========================================================
    pb_Jl, pb_Ja = p.calculateJacobian(
        robotID, 4, [0, 0, 0],
        q.tolist(), q_dot.tolist(), [0, 0, 0],
    )
    pb_Jg = np.vstack([np.array(pb_Ja), np.array(pb_Jl)])

    if Md_inv is not None:
        pb_Vg = pb_Jg @ q_dot
        pb_Fs = K_s @ e_s - D_s @ pb_Vg
        pb_a_cmd = Md_inv @ pb_Fs

        eps = 1e-6
        q_pert = q + eps * q_dot
        pb_Jl2, pb_Ja2 = p.calculateJacobian(
            robotID, 4, [0, 0, 0],
            q_pert.tolist(), q_dot.tolist(), [0, 0, 0],
        )
        pb_Jg2 = np.vstack([np.array(pb_Ja2), np.array(pb_Jl2)])
        pb_Jdot_qdot = (pb_Jg2 @ q_dot - pb_Jg @ q_dot) / eps

        pb_J_pinv = np.linalg.pinv(pb_Jg)
        pb_qdd_cmd = pb_J_pinv @ (pb_a_cmd - pb_Jdot_qdot)
        pb_tau = np.array(p.calculateInverseDynamics(
            robotID, q.tolist(), q_dot.tolist(), pb_qdd_cmd.tolist(),
        ))
    else:
        pb_Vg = pb_Jg @ q_dot
        pb_Fs = K_s @ e_s - D_s @ pb_Vg
        pb_tau_imp = pb_Jg.T @ pb_Fs
        pb_tau_dyn = np.array(p.calculateInverseDynamics(
            robotID, q.tolist(), q_dot.tolist(), [0, 0, 0],
        ))
        pb_tau = pb_tau_imp + pb_tau_dyn

    # ===========================================================
    #  Print comparison every 1 second
    # ===========================================================
    if step % 240 == 0:
        mode = "M_d shaping" if Md_inv is not None else "passive K-D"
        print(f"\n--- t = {step * timeStep:.1f}s  [{mode}] ---")
        print(f"  |pos_err| = {np.linalg.norm(e_s[3:6]):.6f} m    "
              f"|ori_err| = {np.linalg.norm(e_s[0:3]):.6f} rad")
        print(f"  mr_tau = {mr_tau}")
        print(f"  pb_tau = {pb_tau}")
        print(f"  diff   = {mr_tau - pb_tau}")

    for i in range(len(robot.actuated_joints)):
        p.setJointMotorControl2(robotID, i + 1, p.TORQUE_CONTROL,
                                force=mr_tau[i])

    p.stepSimulation()
    time.sleep(timeStep)
    step += 1
