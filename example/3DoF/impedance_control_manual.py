"""Task-Space Impedance Control for 3DoF Robot

Comparing PyBullet and Modern Robotics implementations.

The end-effector behaves as a virtual spring-damper:
    F = K * (x_d - x) - D * ẋ
    τ = J^T * F + τ_dynamics(q, q̇)
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


def orientation_error(R_d, R):
    """Compute orientation error as a 3D rotation vector in the space frame.

    Returns the axis-angle vector such that R_d = exp([e])*R.
    """
    R_err = R_d @ R.T
    so3mat = mr.MatrixLog3(R_err)
    return mr.so3ToVec(so3mat)


def spatial_to_geometric_jacobian(Js, p_ee):
    """Convert MR spatial Jacobian to geometric Jacobian.

    Spatial twist:   V_s = [ω; v_s]   where v_s = ṗ - ω×p
    Geometric twist: V_g = [ω; ṗ]     actual EE linear velocity

    Relationship: ṗ = v_s + ω×p = v_s - [p]×ω
    So: J_g = [[I, 0], [-[p]×, I]] @ J_s
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

# ===================== Impedance Parameters =====================
Kp = np.diag([500.0, 500.0, 500.0])    # translational stiffness  [N/m]
Ko = np.diag([50.0, 50.0, 50.0])       # rotational stiffness     [Nm/rad]
Dp = np.diag([100.0, 100.0, 100.0])    # translational damping    [Ns/m]
Do = np.diag([10.0, 10.0, 10.0])       # rotational damping       [Nms/rad]

g = np.array([0, 0, -9.8])
Ftip = np.zeros(6)

# ===================== Control Loop =====================
step = 0

while p.isConnected():
    jointStates = p.getJointStates(robotID, [1, 2, 3])
    q = np.array([js[0] for js in jointStates])
    q_dot = np.array([js[1] for js in jointStates])

    # ===========================================================
    #  Modern Robotics impedance control
    # ===========================================================

    # FK
    mr_T = mr.FKinSpace(M, Slist, q)
    mr_pos = mr_T[0:3, 3]
    mr_R = mr_T[0:3, 0:3]

    # Task-space error
    e_pos = p_desired - mr_pos
    e_ori = orientation_error(R_desired, mr_R)

    # Geometric Jacobian (maps q̇ → [ω; ṗ] in space frame)
    mr_Js = mr.JacobianSpace(Slist, q)
    mr_Jg = spatial_to_geometric_jacobian(mr_Js, mr_pos)

    # EE velocity
    mr_V = mr_Jg @ q_dot   # [ω(3); v_linear(3)]

    # Impedance wrench  [moment(3); force(3)]  (MR convention)
    mr_F = np.concatenate([
        Ko @ e_ori - Do @ mr_V[0:3],
        Kp @ e_pos - Dp @ mr_V[3:6],
    ])

    # τ = J^T·F_impedance + τ_dynamics(q, q̇, q̈=0)
    mr_tau_imp = mr_Jg.T @ mr_F
    mr_tau_dyn = mr.InverseDynamics(q, q_dot, np.zeros(3), g, Ftip,
                                    Mlist, Glist, Slist)
    mr_tau = mr_tau_imp + mr_tau_dyn

    # ===========================================================
    #  PyBullet impedance control
    # ===========================================================

    # FK
    linkState = p.getLinkState(robotID, 4, 1, 1)
    pb_T = pos_orn_to_T(linkState[4], linkState[5])
    pb_pos = pb_T[0:3, 3]
    pb_R = pb_T[0:3, 0:3]

    # Task-space error
    e_pos_pb = p_desired - pb_pos
    e_ori_pb = orientation_error(R_desired, pb_R)

    # Geometric Jacobian  (pybullet returns [J_linear; J_angular] directly)
    pb_Jl, pb_Ja = p.calculateJacobian(
        robotID, 4, [0, 0, 0],
        q.tolist(), q_dot.tolist(), [0, 0, 0],
    )
    pb_Jg = np.vstack([np.array(pb_Ja), np.array(pb_Jl)])  # reorder → [ω; v]

    # EE velocity
    pb_V = pb_Jg @ q_dot

    # Impedance wrench
    pb_F = np.concatenate([
        Ko @ e_ori_pb - Do @ pb_V[0:3],
        Kp @ e_pos_pb - Dp @ pb_V[3:6],
    ])

    # τ = J^T·F_impedance + τ_dynamics(q, q̇, q̈=0)
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
        print(f"  |pos_err| = {np.linalg.norm(e_pos):.6f} m    "
              f"|ori_err| = {np.linalg.norm(e_ori):.6f} rad")
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
