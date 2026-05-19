"""Franka Panda Task-Space Impedance Control.

Desired dynamics:   M_d · ẍ + D · ẋ + K · e  =  0
                    (with --inertia-shaping; otherwise the M_d term is
                     dropped and the controller is the passive K-D form)

Options
-------
  --frame {body, tcp}        body : impedance expressed in the end-effector
                                    body frame (rotates with the EE)
                             tcp  : impedance expressed in the world frame,
                                    at the TCP (geometric Jacobian)
  --inertia-shaping          enable desired-inertia (M_d) shaping
  --method {mr, pybullet}    backend used to drive the robot.  The other
                             backend is still computed every step and
                             printed for comparison.

Examples
--------
    python impedance_control.py --frame body
    python impedance_control.py --frame tcp  --inertia-shaping
    python impedance_control.py --frame body --inertia-shaping --method pybullet
"""

import argparse
import os
import time

import numpy as np
import pybullet as p
import pybullet_data
import modern_robotics as mr
from mr_urdf_loader import loadURDF

np.set_printoptions(precision=4, suppress=True)


# ===================== CLI =====================
def parse_args():
    parser = argparse.ArgumentParser(
        description="Franka Panda task-space impedance control",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--frame', choices=['body', 'tcp'], default='body',
                        help="reference frame for the impedance")
    parser.add_argument('--inertia-shaping', action='store_true',
                        help="enable desired-inertia (M_d) shaping")
    parser.add_argument('--method', choices=['mr', 'pybullet'], default='mr',
                        help="backend used to compute the applied torques")
    parser.add_argument('--no-gui', action='store_true',
                        help="run pybullet in DIRECT mode (no GUI)")
    parser.add_argument('--print-every', type=float, default=1.0,
                        help="comparison print interval, seconds")
    parser.add_argument('--max-steps', type=int, default=0,
                        help="stop after this many simulation steps (0 = run forever)")
    return parser.parse_args()


# ===================== Math helpers =====================
def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def orientation_error_body(R_d, R):
    """e_b such that R_d = R · exp([e_b])."""
    return mr.so3ToVec(mr.MatrixLog3(R.T @ R_d))


def orientation_error_space(R_d, R):
    """e_s such that R_d = exp([e_s]) · R."""
    return mr.so3ToVec(mr.MatrixLog3(R_d @ R.T))


def spatial_to_geometric_jacobian(Js, p_ee):
    """MR spatial Jacobian → geometric Jacobian [ω; ṗ]."""
    Jg = np.zeros_like(Js)
    Jg[0:3, :] = Js[0:3, :]
    Jg[3:6, :] = Js[3:6, :] - skew(p_ee) @ Js[0:3, :]
    return Jg


def pos_orn_to_T(pos, orn):
    T = np.eye(4)
    T[0:3, 3] = np.array(pos)
    T[0:3, 0:3] = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    return T


def blkdiag_R(R):
    """blockdiag(R, R) — rotates the [orient; trans] parts of a wrench/twist
    that share the same coordinate frame.  Not a spatial adjoint."""
    out = np.zeros((6, 6))
    out[0:3, 0:3] = R
    out[3:6, 3:6] = R
    return out


# ===================== Per-frame error / Jacobian (MR) =====================
def mr_eJV(frame, M, Slist, Blist, q, q_dot, p_des, R_des):
    """Returns (e, J, V, T, Jdot_qdot_func) computed via modern_robotics.

    body : J = J_body,   V = [ω_b; v_b]
    tcp  : J = J_geo,    V = [ω_s; ṗ]
    """
    T = mr.FKinSpace(M, Slist, q)
    R = T[0:3, 0:3]
    pos = T[0:3, 3]

    if frame == 'body':
        e = np.concatenate([
            orientation_error_body(R_des, R),
            R.T @ (p_des - pos),
        ])
        J = mr.JacobianBody(Blist, q)
        V = J @ q_dot

        def Jdot_qdot(eps=1e-6):
            J2 = mr.JacobianBody(Blist, q + eps * q_dot)
            return (J2 @ q_dot - V) / eps
    else:  # tcp
        e = np.concatenate([
            orientation_error_space(R_des, R),
            p_des - pos,
        ])
        Js = mr.JacobianSpace(Slist, q)
        J = spatial_to_geometric_jacobian(Js, pos)
        V = J @ q_dot

        def Jdot_qdot(eps=1e-6):
            q2 = q + eps * q_dot
            T2 = mr.FKinSpace(M, Slist, q2)
            Js2 = mr.JacobianSpace(Slist, q2)
            J2 = spatial_to_geometric_jacobian(Js2, T2[0:3, 3])
            return (J2 @ q_dot - V) / eps

    return e, J, V, T, Jdot_qdot


# ===================== Per-frame error / Jacobian (PyBullet) =====================
def pb_eJV(frame, robotID, tcp_link_index, n_arm,
           full_q, full_qdot, p_des, R_des, M_mr, Slist):
    """Returns (e, J, V, T, Jdot_qdot_func) computed via pybullet.

    Only the first n_arm columns of the Jacobian are kept (finger joints do
    not move the TCP, so those columns are zero anyway).
    """
    full_qddot_zero = [0.0] * len(full_q)

    ls = p.getLinkState(robotID, tcp_link_index, 1, 1)
    T = pos_orn_to_T(ls[4], ls[5])
    R = T[0:3, 0:3]
    pos = T[0:3, 3]

    Jl, Ja = p.calculateJacobian(
        robotID, tcp_link_index, [0, 0, 0],
        list(full_q), list(full_qdot), full_qddot_zero,
    )
    Jg_full = np.vstack([np.array(Ja), np.array(Jl)])  # [ω; v]_world

    if frame == 'body':
        J_full = blkdiag_R(R.T) @ Jg_full
        e = np.concatenate([
            orientation_error_body(R_des, R),
            R.T @ (p_des - pos),
        ])
    else:
        J_full = Jg_full
        e = np.concatenate([
            orientation_error_space(R_des, R),
            p_des - pos,
        ])

    J = J_full[:, :n_arm]
    V = J @ full_qdot[:n_arm]

    def Jdot_qdot(eps=1e-6):
        q_pert_full = list(full_q)
        for i in range(n_arm):
            q_pert_full[i] = full_q[i] + eps * full_qdot[i]
        Jl2, Ja2 = p.calculateJacobian(
            robotID, tcp_link_index, [0, 0, 0],
            q_pert_full, list(full_qdot), full_qddot_zero,
        )
        Jg2 = np.vstack([np.array(Ja2), np.array(Jl2)])
        if frame == 'body':
            # need R(q + ε q̇) — use MR FK on the arm joints
            q_arm_pert = np.array(full_q[:n_arm]) + eps * np.array(full_qdot[:n_arm])
            R2 = mr.FKinSpace(M_mr, Slist, q_arm_pert)[0:3, 0:3]
            J2_full = blkdiag_R(R2.T) @ Jg2
        else:
            J2_full = Jg2
        J2 = J2_full[:, :n_arm]
        return (J2 @ full_qdot[:n_arm] - V) / eps

    return e, J, V, T, Jdot_qdot


# ===================== Torque assembly =====================
def assemble_tau_mr(e, J, V, Jdot_qdot, K, D, Md_inv,
                    q, q_dot, n_arm, Mlist, Glist, Slist, g):
    """τ from MR.  M_d shaping vs. passive K-D depending on Md_inv."""
    F = K @ e - D @ V
    if Md_inv is not None:
        a_cmd = Md_inv @ F
        qdd = np.linalg.pinv(J) @ (a_cmd - Jdot_qdot())
        return mr.InverseDynamics(q, q_dot, qdd, g, np.zeros(6),
                                  Mlist, Glist, Slist)
    # passive K-D:   τ = J^T F  +  ID(q, q̇, 0)
    tau_imp = J.T @ F
    tau_dyn = mr.InverseDynamics(q, q_dot, np.zeros(n_arm), g, np.zeros(6),
                                 Mlist, Glist, Slist)
    return tau_imp + tau_dyn


def assemble_tau_pb(e, J, V, Jdot_qdot, K, D, Md_inv,
                    robotID, full_q, full_qdot, n_arm):
    """τ from PyBullet.  M_d shaping vs. passive K-D depending on Md_inv."""
    F = K @ e - D @ V
    full_qddot = [0.0] * len(full_q)
    if Md_inv is not None:
        a_cmd = Md_inv @ F
        qdd_arm = np.linalg.pinv(J) @ (a_cmd - Jdot_qdot())
        for i in range(n_arm):
            full_qddot[i] = float(qdd_arm[i])
        tau_full = np.array(p.calculateInverseDynamics(
            robotID, list(full_q), list(full_qdot), full_qddot,
        ))
        return tau_full[:n_arm]
    # passive K-D:   τ = J^T F  +  ID(q, q̇, 0)
    tau_imp = J.T @ F
    tau_dyn_full = np.array(p.calculateInverseDynamics(
        robotID, list(full_q), list(full_qdot), full_qddot,
    ))
    return tau_imp + tau_dyn_full[:n_arm]


# ===================== Robot bring-up =====================
def setup_robot(args):
    # Use the stripped 7-DoF arm URDF shipped next to this script so that
    # modern_robotics and pybullet see the same rigid-body tree (no gripper).
    urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "panda_arm.urdf")

    arm_joint_names = [f'panda_joint{i}' for i in range(1, 8)]
    tcp_link_name = 'panda_tcp'

    # ---- MR side ----
    M, Slist, Blist, Mlist, Glist, robot = loadURDF(
        urdf_path,
        eef_link_name=tcp_link_name,
        actuated_joint_names=arm_joint_names,
    )

    # ---- PyBullet side ----
    p.connect(p.DIRECT if args.no_gui else p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robotID = p.loadURDF(urdf_path, [0, 0, 0], [0, 0, 0, 1],
                         useFixedBase=1, flags=p.URDF_USE_INERTIA_FROM_FILE)

    arm_idx, tcp_link_idx, all_dof_idx = [], -1, []
    for j in range(p.getNumJoints(robotID)):
        info = p.getJointInfo(robotID, j)
        jname = info[1].decode()
        link_name = info[12].decode()
        if info[2] != p.JOINT_FIXED:
            all_dof_idx.append(j)
        if jname in arm_joint_names:
            arm_idx.append(j)
        if link_name == tcp_link_name:
            tcp_link_idx = j
    assert tcp_link_idx >= 0, "TCP link not found"
    assert len(arm_idx) == 7, "expected 7 arm joints"

    for j in arm_idx:
        p.setJointMotorControl2(robotID, j, p.VELOCITY_CONTROL,
                                targetVelocity=0, force=0)

    timeStep = 1.0 / 240.0
    p.setTimeStep(timeStep)
    p.setRealTimeSimulation(False)

    return dict(
        M=M, Slist=Slist, Blist=Blist, Mlist=Mlist, Glist=Glist, robot=robot,
        robotID=robotID,
        arm_idx=arm_idx, tcp_link_idx=tcp_link_idx, all_dof_idx=all_dof_idx,
        timeStep=timeStep,
    )


# ===================== Main =====================
def main():
    args = parse_args()
    ctx = setup_robot(args)

    M, Slist, Blist = ctx['M'], ctx['Slist'], ctx['Blist']
    Mlist, Glist = ctx['Mlist'], ctx['Glist']
    robotID = ctx['robotID']
    arm_idx, all_dof_idx = ctx['arm_idx'], ctx['all_dof_idx']
    tcp_link_idx = ctx['tcp_link_idx']
    timeStep = ctx['timeStep']
    n_arm = len(arm_idx)

    # Franka home-like initial configuration
    q_init = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.6, 0.785])
    for j, qi in zip(arm_idx, q_init):
        p.resetJointState(robotID, j, qi)

    # Desired pose: offset from current EE
    T_init = mr.FKinSpace(M, Slist, q_init)
    p_desired = T_init[0:3, 3] + np.array([0.05, 0.10, 0.05])
    R_desired = T_init[0:3, 0:3].copy()

    print(f"Initial EE position : {T_init[0:3, 3]}")
    print(f"Desired EE position : {p_desired}")
    if not args.no_gui:
        p.addUserDebugPoints([p_desired.tolist()], [[1, 0, 0]], pointSize=15)

    # Impedance gains  (Kp/Dp for translation, Ko/Do for orientation)
    Kp = np.diag([400.0, 400.0, 400.0])
    Ko = np.diag([40.0, 40.0, 40.0])
    Dp = np.diag([40.0, 40.0, 40.0])
    Do = np.diag([8.0, 8.0, 8.0])
    K = np.block([[Ko, np.zeros((3, 3))],
                  [np.zeros((3, 3)), Kp]])
    D = np.block([[Do, np.zeros((3, 3))],
                  [np.zeros((3, 3)), Dp]])

    # Desired inertia (block-diagonal, [rot(3); trans(3)])
    if args.inertia_shaping:
        Md = np.diag([2.0, 2.0, 2.0, 5.0, 5.0, 5.0])
        Md_inv = np.linalg.inv(Md)
    else:
        Md_inv = None

    g = np.array([0, 0, -9.8])

    print(f"frame={args.frame}  inertia_shaping={args.inertia_shaping}  "
          f"method={args.method}")

    print_every = max(1, int(round(args.print_every / timeStep)))
    step = 0

    while p.isConnected():
        # --- read state ---
        arm_states = p.getJointStates(robotID, arm_idx)
        q = np.array([js[0] for js in arm_states])
        q_dot = np.array([js[1] for js in arm_states])

        all_states = p.getJointStates(robotID, all_dof_idx)
        full_q = [js[0] for js in all_states]
        full_qdot = [js[1] for js in all_states]

        # =====================================================
        #  MR-side torque
        # =====================================================
        e_mr, J_mr, V_mr, _T_mr, Jdq_mr = mr_eJV(
            args.frame, M, Slist, Blist, q, q_dot, p_desired, R_desired)
        mr_tau = assemble_tau_mr(
            e_mr, J_mr, V_mr, Jdq_mr,
            K, D, Md_inv, q, q_dot, n_arm, Mlist, Glist, Slist, g)

        # =====================================================
        #  PB-side torque
        # =====================================================
        e_pb, J_pb, V_pb, _T_pb, Jdq_pb = pb_eJV(
            args.frame, robotID, tcp_link_idx, n_arm,
            full_q, full_qdot, p_desired, R_desired, M, Slist)
        pb_tau = assemble_tau_pb(
            e_pb, J_pb, V_pb, Jdq_pb,
            K, D, Md_inv, robotID, full_q, full_qdot, n_arm)

        # =====================================================
        #  Choose & apply
        # =====================================================
        tau = mr_tau if args.method == 'mr' else pb_tau
        for j, ti in zip(arm_idx, tau):
            p.setJointMotorControl2(robotID, j, p.TORQUE_CONTROL,
                                    force=float(ti))

        if step % print_every == 0:
            mode = "M_d shaping" if Md_inv is not None else "passive K-D"
            t = step * timeStep
            print(f"\n--- t={t:5.2f}s  [{args.frame} | {mode} | drive={args.method}] ---")
            print(f"  |pos_err| = {np.linalg.norm(e_mr[3:6]):.6f} m    "
                  f"|ori_err| = {np.linalg.norm(e_mr[0:3]):.6f} rad")
            print(f"  mr_tau = {mr_tau}")
            print(f"  pb_tau = {pb_tau}")
            print(f"  |diff| = {np.linalg.norm(mr_tau - pb_tau):.4e}")

        p.stepSimulation()
        if not args.no_gui:
            time.sleep(timeStep)
        step += 1
        if args.max_steps and step >= args.max_steps:
            break


if __name__ == "__main__":
    main()
