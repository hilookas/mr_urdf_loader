"""Task-Space Impedance Control for 3DoF Robot

Two implementation approaches compared:

  MR approach  — compute impedance wrench in the body frame,
                 pass it as Ftip to InverseDynamics.
                 Newton-Euler internally handles J_b^T·Ftip + dynamics.

  PB approach  — compute impedance wrench in the space frame,
                 map to joint torques via J_geo^T, then add dynamics.

Both yield identical joint torques.
"""

import pybullet as p
import time
import pybullet_data
import numpy as np
import modern_robotics as mr
from mr_urdf_loader import loadURDF

np.set_printoptions(precision=6, suppress=True)


# ===================== Utility Functions =====================

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def orientation_error_space(R_d, R):
    """Orientation error in the space frame: R_d = exp([e])·R."""
    return mr.so3ToVec(mr.MatrixLog3(R_d @ R.T))


def orientation_error_body(R_d, R):
    """Orientation error in the body frame: R_d = R·exp([e_b])."""
    return mr.so3ToVec(mr.MatrixLog3(R.T @ R_d))


def spatial_to_geometric_jacobian(Js, p_ee):
    """Convert MR spatial Jacobian to geometric Jacobian.

    Spatial:   V_s = [ω; v_s]  where v_s = ṗ - ω×p
    Geometric: V_g = [ω; ṗ]   actual EE linear velocity

    ṗ = v_s + ω×p = v_s - [p]×ω  →  J_g[3:6] = J_s[3:6] - [p]× J_s[0:3]
    """
    Jg = np.zeros_like(Js)
    Jg[0:3, :] = Js[0:3, :]
    Jg[3:6, :] = Js[3:6, :] - skew(p_ee) @ Js[0:3, :]
    return Jg


def pos_orn_to_T(pos, orn):
    T = np.eye(4)
    T[0:3, 3] = np.array(pos)
    T[0:3, 0:3] = np.reshape(p.getMatrixFromQuaternion(orn), (3, 3))
    return T


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

# Kp 被改成了各向异性（1000, 500, 500），不再是标量乘以单位阵了。 这就是两边结果不同的根本原因。

# 当 Kp 各向异性时，body frame 和 space frame 下的阻抗控制本质上就是不同的控制器：

# Body frame（MR 方式）：刚度轴随末端执行器旋转。"x 方向更硬"是相对于工具坐标系。
# Space frame（PB 方式）：刚度轴固定在世界坐标系。"x 方向更硬"是相对于世界坐标系。
# 数学上，各向同性时 Kp @ R^T = R^T @ Kp（因为 kI 和任何矩阵可交换），所以两边等价。但各向异性时 Kp @ R^T ≠ R^T @ Kp，变换不可交换，两个控制器就不等价了。

# ===================== Impedance Parameters =====================
Kp = np.diag([1000.0, 500.0, 500.0])    # translational stiffness  [N/m]
Ko = np.diag([50.0, 50.0, 50.0])       # rotational stiffness     [Nm/rad]
Dp = np.diag([100.0, 100.0, 100.0])    # translational damping    [Ns/m]
Do = np.diag([10.0, 10.0, 10.0])       # rotational damping       [Nms/rad]

g = np.array([0, 0, -9.8])

# ===================== Control Loop =====================
step = 0

while p.isConnected():
    jointStates = p.getJointStates(robotID, [1, 2, 3])
    q = np.array([js[0] for js in jointStates])
    q_dot = np.array([js[1] for js in jointStates])

    # ===========================================================
    #  MR approach: body-frame impedance via Ftip
    #
    #  Compute errors & velocity in the body frame {n+1},
    #  build the impedance wrench, and pass it as Ftip to
    #  InverseDynamics. The Newton-Euler backward pass naturally
    #  performs  τ = M·q̈ + c + g + J_b^T·Ftip  in one call.
    # ===========================================================

    # FK
    mr_T = mr.FKinSpace(M, Slist, q)
    mr_R = mr_T[0:3, 0:3]
    mr_pos = mr_T[0:3, 3]

    # Errors in body frame
    e_pos_b = mr_R.T @ (p_desired - mr_pos)
    e_ori_b = orientation_error_body(R_desired, mr_R)

    # 等价：np.linalg.inv(mr_T) @ T_desired

    # Body Jacobian → body-frame velocity  [ω_b; v_b]
    mr_Jb = mr.JacobianBody(Blist, q)
    mr_Vb = mr_Jb @ q_dot

    # Impedance wrench in body frame  [moment; force]
    Ftip_imp = np.concatenate([
        Ko @ e_ori_b - Do @ mr_Vb[0:3],
        Kp @ e_pos_b - Dp @ mr_Vb[3:6],
    ])

    # One-shot: dynamics + Ftip
    mr_tau = mr.InverseDynamics(q, q_dot, np.zeros(3), g, Ftip_imp,
                                Mlist, Glist, Slist)

    # ===========================================================
    #  PB approach: space-frame impedance via J_geo^T
    #
    #  pybullet's calculateInverseDynamics has no Ftip parameter,
    #  so we map the impedance wrench through J_geo^T manually.
    # ===========================================================

    # FK
    linkState = p.getLinkState(robotID, 4, 1, 1)
    pb_T = pos_orn_to_T(linkState[4], linkState[5])
    pb_pos = pb_T[0:3, 3]
    pb_R = pb_T[0:3, 0:3]

    # Errors in space frame
    e_pos_s = p_desired - pb_pos
    e_ori_s = orientation_error_space(R_desired, pb_R)

    # Geometric Jacobian  (pybullet returns [J_linear; J_angular])
    pb_Jl, pb_Ja = p.calculateJacobian(
        robotID, 4, [0, 0, 0],
        q.tolist(), q_dot.tolist(), [0, 0, 0],
    )
    pb_Jg = np.vstack([np.array(pb_Ja), np.array(pb_Jl)])  # → [ω; v]

    # EE velocity in space frame
    pb_V = pb_Jg @ q_dot

    # Impedance wrench in space frame  [moment; force]
    pb_F = np.concatenate([
        Ko @ e_ori_s - Do @ pb_V[0:3],
        Kp @ e_pos_s - Dp @ pb_V[3:6],
    ])

    # τ = J_geo^T·F_impedance  +  dynamics(q, q̇, q̈=0)
    pb_tau_imp = pb_Jg.T @ pb_F
    pb_tau_dyn = np.array(p.calculateInverseDynamics(
        robotID, q.tolist(), q_dot.tolist(), [0, 0, 0],
    ))
    pb_tau = pb_tau_imp + pb_tau_dyn

    # ===========================================================
    #  Print comparison every 1 second
    # ===========================================================
    if step % 240 == 0:
        print(f"\n--- t = {step * timeStep:.1f}s ---")
        print(f"  |pos_err| = {np.linalg.norm(e_pos_b):.6f} m    "
              f"|ori_err| = {np.linalg.norm(e_ori_b):.6f} rad")
        print(f"  mr_tau = {mr_tau}")
        print(f"  pb_tau = {pb_tau}")
        print(f"  diff   = {mr_tau - pb_tau}")

    # ===========================================================
    #  Apply torques (MR-computed)
    # ===========================================================
    for i in range(len(robot.actuated_joints)):
        p.setJointMotorControl2(robotID, i + 1, p.TORQUE_CONTROL,
                                force=mr_tau[i])

    p.stepSimulation()
    time.sleep(timeStep)
    step += 1
