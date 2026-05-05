"""Task-Space Impedance Control with Inertia Shaping — Body Frame

Desired dynamics:  M_d·ẍ + D·ẋ + K·e = 0

Implementation:
  1. F_b = K·e_b - D·V_b           (spring-damper wrench, body frame)
  2. a_cmd = M_d⁻¹ · F_b           (desired twist acceleration)
  3. q̈_cmd = J_b⁺ · (a_cmd - J̇_b·q̇)  (map to joint acceleration)
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


def orientation_error_body(R_d, R):
    """Orientation error in the body frame: R_d = R·exp([e_b])."""
    return mr.so3ToVec(mr.MatrixLog3(R.T @ R_d))


def pos_orn_to_T(pos, orn):
    T = np.eye(4)
    T[0:3, 3] = np.array(pos)
    T[0:3, 0:3] = np.reshape(p.getMatrixFromQuaternion(orn), (3, 3))
    return T


def compute_Jdot_qdot_body(Blist, q, q_dot, Jb, eps=1e-6):
    """Numerically compute J̇_b·q̇ via finite difference."""
    q_pert = q + eps * q_dot
    Jb_pert = mr.JacobianBody(Blist, q_pert)
    return (Jb_pert @ q_dot - Jb @ q_dot) / eps


# ===================== Robot Setup =====================
urdf_name = "3DoF.urdf"
M, Slist, Blist, Mlist, Glist, robot = loadURDF(urdf_name)

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
T_desired = mr.FKinSpace(M, Slist, q_target)
p_desired = T_desired[0:3, 3].copy()
R_desired = T_desired[0:3, 0:3].copy()

print(f"Desired EE position : {p_desired}")
print(f"(from joint config  : {q_target})")
p.addUserDebugPoints([p_desired.tolist()], [[1, 0, 0]], pointSize=15)

# ===================== Impedance Parameters (body frame) =====================
Kp = np.diag([1000.0, 500.0, 500.0])
Ko = np.diag([80.0, 50.0, 50.0])
Dp = np.diag([120.0, 100.0, 100.0])
Do = np.diag([12.0, 10.0, 10.0])

K_b = np.block([[Ko, np.zeros((3, 3))],
                [np.zeros((3, 3)), Kp]])       # 6×6 stiffness
D_b = np.block([[Do, np.zeros((3, 3))],
                [np.zeros((3, 3)), Dp]])       # 6×6 damping

# Desired inertia (body frame). Set to None for passive K-D control.
Md = np.diag([2.0, 2.0, 2.0, 5.0, 5.0, 5.0])  # [rot(3); trans(3)]
Md_inv = np.linalg.inv(Md) if Md is not None else None

g = np.array([0, 0, -9.8])

# ===================== Control Loop =====================
step = 0

while p.isConnected():
    jointStates = p.getJointStates(robotID, [1, 2, 3])
    q = np.array([js[0] for js in jointStates])
    q_dot = np.array([js[1] for js in jointStates])

    # ===========================================================
    #  Common: body-frame FK, errors, Jacobian
    # ===========================================================
    mr_T = mr.FKinSpace(M, Slist, q)
    R = mr_T[0:3, 0:3]
    pos = mr_T[0:3, 3]

    e_b = np.concatenate([
        orientation_error_body(R_desired, R),
        R.T @ (p_desired - pos),
    ])

    mr_Jb = mr.JacobianBody(Blist, q)
    Vb = mr_Jb @ q_dot

    F_b = K_b @ e_b - D_b @ Vb                             # wrench

    # ===========================================================
    #  MR: compute joint torques
    # ===========================================================
    if Md_inv is not None:
        a_cmd = Md_inv @ F_b                                # desired twist accel
        Jdot_qdot = compute_Jdot_qdot_body(Blist, q, q_dot, mr_Jb)
        J_pinv = np.linalg.pinv(mr_Jb)
        qdd_cmd = J_pinv @ (a_cmd - Jdot_qdot)
        mr_tau = mr.InverseDynamics(q, q_dot, qdd_cmd, g, np.zeros(6),
                                    Mlist, Glist, Slist)
    else:
        mr_tau = mr.InverseDynamics(q, q_dot, np.zeros(3), g, F_b,
                                    Mlist, Glist, Slist)

    # ===========================================================
    #  PB: same algorithm, pybullet functions
    # ===========================================================
    pb_Jl, pb_Ja = p.calculateJacobian(
        robotID, 4, [0, 0, 0],
        q.tolist(), q_dot.tolist(), [0, 0, 0],
    )
    pb_Jg = np.vstack([np.array(pb_Ja), np.array(pb_Jl)])
    R_block = np.zeros((6, 6))
    R_block[0:3, 0:3] = R
    R_block[3:6, 3:6] = R
    pb_Jb = R_block.T @ pb_Jg

    if Md_inv is not None:
        pb_Vb = pb_Jb @ q_dot
        pb_Fb = K_b @ e_b - D_b @ pb_Vb
        pb_a_cmd = Md_inv @ pb_Fb

        eps = 1e-6
        q_pert = q + eps * q_dot
        pb_Jl2, pb_Ja2 = p.calculateJacobian(
            robotID, 4, [0, 0, 0],
            q_pert.tolist(), q_dot.tolist(), [0, 0, 0],
        )
        pb_Jg2 = np.vstack([np.array(pb_Ja2), np.array(pb_Jl2)])
        mr_T2 = mr.FKinSpace(M, Slist, q_pert)
        R2 = mr_T2[0:3, 0:3]
        R_block2 = np.zeros((6, 6))
        R_block2[0:3, 0:3] = R2
        R_block2[3:6, 3:6] = R2
        pb_Jb2 = R_block2.T @ pb_Jg2
        pb_Jdot_qdot = (pb_Jb2 @ q_dot - pb_Jb @ q_dot) / eps

        pb_J_pinv = np.linalg.pinv(pb_Jb)
        pb_qdd_cmd = pb_J_pinv @ (pb_a_cmd - pb_Jdot_qdot)
        pb_tau = np.array(p.calculateInverseDynamics(
            robotID, q.tolist(), q_dot.tolist(), pb_qdd_cmd.tolist(),
        ))
    else:
        pb_Vb = pb_Jb @ q_dot
        pb_Fb = K_b @ e_b - D_b @ pb_Vb
        pb_tau_imp = pb_Jb.T @ pb_Fb
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
        print(f"  |pos_err| = {np.linalg.norm(e_b[3:6]):.6f} m    "
              f"|ori_err| = {np.linalg.norm(e_b[0:3]):.6f} rad")
        print(f"  mr_tau = {mr_tau}")
        print(f"  pb_tau = {pb_tau}")
        print(f"  diff   = {mr_tau - pb_tau}")

    for i in range(len(robot.actuated_joints)):
        p.setJointMotorControl2(robotID, i + 1, p.TORQUE_CONTROL,
                                force=mr_tau[i])

    p.stepSimulation()
    time.sleep(timeStep)
    step += 1
