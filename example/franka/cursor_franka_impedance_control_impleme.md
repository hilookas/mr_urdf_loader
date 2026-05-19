# Franka impedance control implementation
_Exported on 2026/5/23 at GMT+8 22:57:19 from Cursor (3.5.17)_

---

**User**

参考@example/3DoF 这里的案例，帮我写一个franka的阻抗控制，放到example/franka/文件夹中

要求：同时支持body frame和tcp frame控制，使用选项进行区分；支持通过选项来设置是否支持改变自然惯量；提供pybullet和modern_robotics两种算tau的方法

---

**Cursor**

Now let me look at how to handle the URDF with urchin and understand the panda structure:

The issue is that urchin tries to resolve `package://meshes/...` and fails. Let me try with lazy mesh loading:

I'll add a `lazy_load_meshes` parameter to `loadURDF` since the Franka URDF uses `package://` paths that urchin can't resolve. Then I'll write the Franka impedance control example.

Now let me verify it works:

Now I have a complete understanding. Let me create the Franka example. First, let me set up the directory and write the main impedance control file:

Now let me test it in a few configurations using no-GUI mode to verify the math is correct:

Let me add a `--max-steps` option for headless testing and re-test:

There's a ~6 Nm torque difference between MR and PB. Let me investigate by checking the simpler 3DoF case and the gravity-only torques on Franka:

The torque discrepancy comes from pybullet seeing extra mass (hand + fingers ~1kg) that the MR loader doesn't include (the loader only registers links of actuated joints, skipping fixed children). To get matching dynamics for the demo, let me create a simplified URDF with only the 7 arm joints:

Now update the example to use this simplified URDF:

Now retest with the simplified arm URDF — torques should match:

Excellent — torques now match to machine precision and convergence improved. Let me verify all four other combinations:

Let me also verify that driving via pybullet works:

All 8 configurations (frame × inertia-shaping × method) work and the MR vs PB torques agree to machine precision. Let me check there are no lint issues and check the final file:

Now let me add a small README so the example folder follows the repo's style:

Let me take a final look at the example file to ensure everything is clean:

Small cleanup: `frame` and `T` are unused in the torque assembly functions. Let me drop them:

Quick re-test to make sure nothing broke:

Everything works. Here's a summary of what I created in `example/franka/`:

**Files**
- `example/franka/impedance_control.py` — single-file impedance controller with CLI options
- `example/franka/panda_arm.urdf` — stripped 7-DoF Panda arm (no gripper) so MR and PyBullet see the same rigid-body tree
- `src/mr_urdf_loader/core.py` — added an optional `lazy_load_meshes` flag to `loadURDF` (urchin chokes on the `package://` mesh paths in Franka URDFs otherwise; backward-compatible since it defaults to `False`)

**Options**
```bash
python impedance_control.py --frame body                                       # body-frame impedance, passive K-D, MR drives
python impedance_control.py --frame tcp                                        # world-frame at TCP (geometric Jacobian)
python impedance_control.py --frame body --inertia-shaping                     # M_d shaping (computed-torque)
python impedance_control.py --frame tcp  --inertia-shaping --method pybullet   # use pybullet's calculateInverseDynamics
python impedance_control.py --no-gui --max-steps 480 --print-every 0.5         # headless smoke test
```

**Design choices, following the `3DoF/` examples**
- `--frame {body, tcp}` selects whether the impedance is expressed in the EE body frame (Jacobian = `JacobianBody`) or in the world frame at the TCP (Jacobian = geometric Jacobian). All matrix dimensions and math closures change behind a single switch in `mr_eJV` / `pb_eJV`.
- `--inertia-shaping` toggles `M_d`. When off, the controller is the passive form `τ = Jᵀ(K·e − D·V) + ID(q, q̇, 0)`. When on, the desired closed-loop dynamics is `M_d ẍ + D ẋ + K e = 0`, implemented as `q̈_cmd = J⁺(M_d⁻¹ F − J̇·q̇)` followed by `τ = M(q)·q̈_cmd + C·q̇ + G`.
- `--method {mr, pybullet}` picks which backend's torques are actually sent to the robot. The other backend is still computed every step and printed for cross-checking — once the inertia trees match, the two agree to ~1e-6 Nm.

**Verification**
All eight combinations were tested headlessly; pos-error converges from 0.122 m → ~0.001 m in ~1 s and the `|mr_tau − pb_tau|` diff stays at machine precision (`~1e-6`), except briefly during fast transients with body-frame `M_d` shaping (`~3e-2`) because the `J̇·q̇` finite-difference uses MR FK for the body-frame `R(q+εq̇)`; this washes out as the velocity decays.

