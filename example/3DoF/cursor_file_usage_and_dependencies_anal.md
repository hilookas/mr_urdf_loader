# File usage and dependencies analysis
_Exported on 2026/5/15 at GMT+8 23:46:45 from Cursor (3.4.17)_

---

**User**

@5_phone_d455_extrinsic.py @6_eval_phone.sh 帮我看一下这两个文件，确定一下这两个文件的用法，以及缺少什么数据/文件让这两个程序运行起来

---

**Cursor**

Now let me check what files already exist in the workspace and what dependencies these scripts rely on.

Let me also check if the missing scripts might be inside the `code.zip`.

好的，现在我已经完整分析了这两个文件。以下是详细的说明：

---

## 1. `5_phone_d455_extrinsic.py` — 用途

这是一个**手机-D455 外参标定**的端到端管线，用于计算手机 ORBSLAM 轨迹和 D455 相机之间的刚体变换 (AX=XB 问题)。

**流程步骤：**
1. 从 ROS2 bag (`.db3`) 中提取 `/D455/pose` 话题 → 生成 `D455_pose.txt`
2. 从手机 HHMMSS 子文件夹中读取 ORBSLAM TUM 文件，将相对时间戳转为绝对 Unix 时间戳 → `*_abs.txt`
3. 对每个 session 搜索最优时间偏移（粗搜 + 细搜）
4. 汇总所有 session 的运动对，用 AX=XB 求解全局外参
5. 逐 session 验证，导出 `*_on_D455_pose.txt` 和 `time_offset.txt`

**用法：**
```bash
python 5_phone_d455_extrinsic.py --root /path/to/bench_single/phone_loose
```

## 2. `6_eval_phone.sh` — 用途

这是一个**评估脚本**，对步骤 5 输出的 `*_abs_on_D455_pose.txt` 文件进行精度评估。它使用 `evo` 工具包计算 ATE/ARE/RTE/RRE 指标，并按轨迹类型分组打印汇总表。

**用法：**
```bash
./6_eval_phone.sh --root /path/to/bench_single/phone_loose
```

---

## 3. 缺少的文件/数据

### (A) 缺少的关键 Python 脚本（**必须有，否则无法运行**）

| 文件 | 被引用位置 | 作用 |
|---|---|---|
| `2_clculate_extrinsic_d455.py` | 第 41 行 `importlib` 导入 | 提供核心工具函数：`read_tum`, `write_tum`, `interpolate_poses`, `solve_ax_xb`, `refine_ax_xb`, `compute_world_alignment`, `compute_errors` 等 |
| `4_pico_head_d455_extrinsic.py` | 第 64 行 `importlib` 导入 | 提供 `search_best_offset` 函数（时间偏移搜索） |

这两个脚本必须放在和 `5_phone_d455_extrinsic.py` **同目录**下。它们不在 `code.zip` 或 `data.0514.tgz` 中。

### (B) 缺少的 Python 依赖包

| 包名 | 用于 | 脚本 |
|---|---|---|
| `numpy` | 数值计算 | 两者都需要 |
| `scipy` | `Rotation` 四元数/旋转矩阵转换 | `5_phone_d455_extrinsic.py` |
| `rosbags` | 反序列化 ROS2 bag 中的消息 | `5_phone_d455_extrinsic.py` |
| `evo` | `evo_ape`, `evo_rpe`, `evo_traj` 命令行工具 | `6_eval_phone.sh` |

当前 `pyproject.toml` 的 `dependencies` 为空，需要补上。

### (C) 缺少的数据 — 目录结构不匹配

脚本期望的 `--root` 目录结构如下：

```
phone_loose/                    ← --root 指向这里
├── session_1/
│   ├── rosbag2_xxx_0.db3       ← ROS2 bag，含 /D455/pose
│   └── 172013_xxx/             ← HHMMSS 开头的子文件夹（手机录制开始时间）
│       └── ORBSLAM_VI.txt      ← 手机 ORBSLAM 轨迹（TUM 格式，相对时间戳）
├── session_2/
│   ├── rosbag2_yyy_0.db3
│   └── 172130_yyy/
│       └── ORBSLAM_VI.txt
└── ...
```

**当前仓库中的 `.db3` 文件**位于 `launch/rosbag2_*/` 目录下，**确实包含 `/D455/pose` 话题**（已验证），但缺少：

1. **HHMMSS 开头的手机子文件夹** — 每个 session 目录下需要有一个名称前 6 位是 `HHMMSS`（如 `172013`）的子文件夹
2. **手机 ORBSLAM 轨迹文件** — 子文件夹中的 `ORBSLAM_VI.txt`（TUM 格式：`timestamp tx ty tz qx qy qz qw`）

---

## 4. 总结：要跑起来需要准备

1. **获取** `2_clculate_extrinsic_d455.py` 和 `4_pico_head_d455_extrinsic.py` 放到项目根目录
2. **安装 Python 依赖**：`pip install numpy scipy rosbags evo`
3. **组织数据目录**：在一个根目录下，每个 session 子文件夹内放置 `.db3` + HHMMSS 手机子文件夹（含 `ORBSLAM_VI.txt`）
4. 先运行 `5_phone_d455_extrinsic.py --root <data_root>` 完成标定
5. 再运行 `6_eval_phone.sh --root <data_root>` 进行精度评估

---

**User**

@2_clculate_extrinsic_d455.py @4_pico_head_d455_extrinsic.py 看一下现在是否好了

---

**Cursor**

两个依赖脚本现在都已经存在了。来做一个完整的状态检查。

好的，现在情况很清楚了。**代码文件已齐全**，但还缺依赖包和数据。以下是更新后的状态：

---

## 代码依赖 — 已解决

- `2_clculate_extrinsic_d455.py` — 已存在
- `4_pico_head_d455_extrinsic.py` — 已存在

三个脚本的调用链：`5` 导入 `2` 和 `4`，`6` 调用 `evo` 工具。

---

## 还差什么

### 1. Python 依赖包（全部未安装）

```bash
pip install numpy scipy rosbags evo
```

| 包 | 用途 |
|---|---|
| `numpy` | 数值计算（所有脚本） |
| `scipy` | 旋转插值、LM 优化 |
| `rosbags` | 反序列化 ROS2 bag 消息 |
| `evo` | `6_eval_phone.sh` 需要的 `evo_ape` / `evo_rpe` / `evo_traj` 命令行工具 |

### 2. 数据目录结构不匹配

`5_phone_d455_extrinsic.py` 的 `find_sessions()` 期望每个 session 目录同时包含：
- 一个 `.db3` 文件
- 一个以 **HHMMSS 开头**的子文件夹（里面有手机 ORBSLAM 轨迹）

当前 `.db3` 文件在 `launch/rosbag2_*/` 下，但这些目录里**没有 HHMMSS 子文件夹和手机轨迹**。需要的结构是：

```
<root>/
├── session_A/
│   ├── *.db3                    ← 已有（含 /D455/pose topic，已验证）
│   └── 171813_xxx/              ← 缺少：HHMMSS 开头的子文件夹
│       └── ORBSLAM_VI.txt       ← 缺少：手机 ORBSLAM 轨迹（TUM 格式）
├── session_B/
│   ├── *.db3
│   └── 172102_xxx/
│       └── ORBSLAM_VI.txt
└── ...
```

### 总结

| 项目 | 状态 |
|---|---|
| `2_clculate_extrinsic_d455.py` | 已有 |
| `4_pico_head_d455_extrinsic.py` | 已有 |
| `5_phone_d455_extrinsic.py` | 已有 |
| `6_eval_phone.sh` | 已有 |
| Python 包 (`numpy`, `scipy`, `rosbags`, `evo`) | **未安装** |
| `.db3` ROS2 bag 数据 | 已有（`launch/rosbag2_*/` 下，含 `/D455/pose`） |
| 手机 ORBSLAM 轨迹文件 (`ORBSLAM_VI.txt`) | **缺少** |
| HHMMSS 子文件夹 | **缺少** |

需要我帮你安装依赖包，或者帮你构造/整理数据目录结构吗？

---

**User**

为了准确对齐时间，这个脚本都做了什么处理，另外要求tum格式的输入数据满足什么要求？

---

**Cursor**

## 时间对齐的处理流程

`5_phone_d455_extrinsic.py` 做了**两层**时间对齐：

### 第一层：相对时间 → 绝对 Unix 时间戳

手机 ORBSLAM 输出的 TUM 文件时间戳是**相对时间**（从录制开始算起的秒数），而 D455 的 ROS2 bag 里是 Unix 绝对时间戳。脚本通过以下方式转换：

```174:189:5_phone_d455_extrinsic.py
def compute_time_base(db3_path, folder_name):
    """Compute the Unix epoch time base for the phone recording.

    Uses the calendar date from the bag and HHMMSS from the folder name.
    Returns time_base (float seconds) or None on failure.
    """
    hms = parse_hhmmss(folder_name)
    if hms is None:
        return None
    hh, mm, ss = hms

    bag_min_ns = get_bag_min_timestamp_ns(db3_path)
    bag_date = datetime.fromtimestamp(bag_min_ns / 1e9).date()

    dt_start = datetime.combine(bag_date, dt_time(hh, mm, ss))
    return dt_start.timestamp()
```

逻辑：
1. 从子文件夹名前 6 位解析出 `HHMMSS`（手机录制开始的本地时间）
2. 从 ROS2 bag 最早消息时间戳提取**日期**
3. 组合成 `datetime(日期, HH:MM:SS)` → 转为 Unix 时间戳作为 `time_base`
4. 最终：`ts_abs = ts_relative + time_base`

### 第二层：搜索最优时间偏移（补偿亚秒级误差）

由于 HHMMSS 只精确到秒，且手机和 D455 没有硬件同步，还需搜索一个精细偏移。调用 `search_best_offset`（来自 `4_pico_head_d455_extrinsic.py`）：

```161:206:4_pico_head_d455_extrinsic.py
def search_best_offset(est_ts, est_pos, est_quat,
                       gt_ts, gt_pos, gt_quat,
                       search_range, coarse_step, fine_step):
    // ...
    # Coarse search
    offsets = np.arange(-search_range, search_range + coarse_step * 0.5,
                        coarse_step)
    for off in offsets:
        r = _try_offset(off, est_ts, est_pos, est_quat,
                        gt_ts, gt_pos, gt_quat)
        if r is not None and r[0] < best_cost:
            best_cost = r[0]
            best_offset = off
            best_result = r
    // ...
    # Fine search around coarse best
    if fine_step < coarse_step:
        fine_lo = best_offset - coarse_step
        fine_hi = best_offset + coarse_step
        fine_offsets = np.arange(fine_lo, fine_hi + fine_step * 0.5, fine_step)
        // ...
```

策略：
1. **粗搜索**：在 `[-5s, +5s]` 范围内，步长 0.1s，遍历所有候选偏移
2. **细搜索**：在粗搜最优值 ±0.1s 范围内，步长 0.01s 精细化
3. **评价标准**：对每个偏移值，将 est 时间戳平移后与 GT 做插值对齐，用 AX=XB 残差作为 cost，选最小的

时间插值使用的方法：

```93:100:2_clculate_extrinsic_d455.py
def interpolate_poses(query_ts, ref_ts, ref_pos, ref_quat):
    mask = (query_ts >= ref_ts[0]) & (query_ts <= ref_ts[-1])
    ts = query_ts[mask]
    if len(ts) < 4:
        return mask, np.empty((0, 3)), np.empty((0, 3, 3))
    pos = np.column_stack([np.interp(ts, ref_ts, ref_pos[:, i]) for i in range(3)])
    rot = Slerp(ref_ts, R.from_quat(ref_quat))(ts).as_matrix()
    return mask, pos, rot
```

- 位置：逐轴线性插值
- 旋转：Slerp 球面线性插值
- 只保留在 GT 时间范围内的点

---

## TUM 格式输入数据的要求

### 基本格式

每行 8 列，空格分隔：

```
timestamp tx ty tz qx qy qz qw
```

| 列 | 含义 |
|---|---|
| `timestamp` | 时间戳（浮点数，单位：秒） |
| `tx ty tz` | 平移（米） |
| `qx qy qz qw` | 四元数（Hamilton 约定，w 在最后） |

### 具体约束

1. **时间戳必须单调递增** — `Slerp` 和 `np.interp` 都要求参考序列严格递增，否则会报错

2. **至少 10 个 pose** — `_try_offset` 中有 `if n_aligned < 10: return None` 的检查；`load_and_align` 也要求 `len(...) >= 10`

3. **不能有 NaN/Inf 或非法四元数** — `np.loadtxt` 解析，四元数会被传给 `R.from_quat()`

4. **不能有注释行或空行** — `np.loadtxt` 默认 `#` 为注释可以跳过，但空行会出错

5. **手机轨迹时间戳可以是相对的**（从 0 开始的秒数），脚本会自动加上 `time_base`

6. **D455 GT 时间戳必须是绝对 Unix 时间**（从 bag 提取后天然满足）

7. **运动要有足够的旋转分量** — `solve_rotation_park_martin` 中跳过旋转向量模长 < 1e-4 的运动对，如果整段轨迹几乎纯平移会导致标定失败

---

**User**

@tum_data @data/launch 这五条数据分别按照时间顺序对应这5个rosbag，帮我评估一下轨迹精度

---

**Cursor**

好的，我来确认时间对应关系并进行评估。TUM 文件的时间戳已经是 Unix 绝对时间，可以直接和 bag 对齐。让我写一个评估脚本来完成这个任务。

---

**User**

@tum_data @data/launch 这五条数据分别按照时间顺序对应这5个rosbag，帮我评估一下轨迹精度

记得使用uv管理的venv环境 source .venv/bin/activate

请尽量使用现存脚本，进行最小修改

---

**Cursor**

时间对应关系确认无误。现在创建评估用的目录结构，提取 D455 GT 轨迹，然后用现有脚本评估。

现在提取 D455 GT 轨迹，然后用 `4_pico_head_d455_extrinsic.py` 做带时间偏移搜索的评估。

GT 提取完成。现在用 `4_pico_head_d455_extrinsic.py` 运行完整的外参标定+精度评估流程：

The process timed out during the global solve phase. The initial per-session results show that the time offsets are inconsistent and session 3 has very high errors (likely a wrong minimum). Let me try a more direct approach — since both TUM data and D455 GT already use absolute Unix timestamps, I can first verify the direct alignment without time offset search, then use `2_clculate_extrinsic_d455.py`.

时间重叠很好（约 90%+ 的 est 点落在 GT 时间范围内），可以直接对齐。用 `2_clculate_extrinsic_d455.py` 来计算外参并评估：

结果出来了，但有几个问题需要进一步分析。Session 2 是明显异常值（113° 旋转误差），且各 session 的外参估计不一致。让我用带时间偏移搜索的方式重新评估（增大搜索范围），并排除异常 session：

结果非常好！扩大搜索范围后，所有 5 个 session 都找到了一致的时间偏移 **+3.94s**，精度显著提升。

---

## 评估结果汇总

### 时间对齐

| 参数 | 值 |
|---|---|
| 时间偏移 | **+3.940s**（所有 session 一致，std=0） |
| 含义 | `gripper_right时间戳 + 3.94s = D455时间戳` |

### 逐 Session 轨迹精度

| Session | Poses | 平移误差 (median) | 旋转误差 (median) | AX=XB cost |
|---|---|---|---|---|
| session_1 | 933 | **0.0017 m** | **0.26°** | 6.38e-02 |
| session_2 | 1012 | **0.0026 m** | **0.31°** | 8.96e-02 |
| session_3 | 862 | **0.0016 m** | **0.28°** | 7.35e-02 |
| session_4 | 1157 | **0.0023 m** | **0.28°** | 8.51e-02 |
| session_5 | 916 | **0.0017 m** | **0.32°** | 1.25e-01 |

### 全局精度统计（4880 个 pose）

| 指标 | Mean | Median | Max | Std |
|---|---|---|---|---|
| 平移误差 [m] | 0.0022 | **0.0016** | 0.0177 | 0.0019 |
| 旋转误差 [°] | 0.361 | **0.279** | 3.126 | 0.278 |

### 外参（gripper_right → D455）

| 参数 | 值 |
|---|---|
| 平移 ||t|| | 0.0634 m (~6.3cm) |
| 旋转 Euler xyz | [+3.7°, -31.9°, -16.7°] |

### 各 session 外参一致性

5 个 session 独立求解的外参高度一致（euler 变化 < 1°，||t|| 在 0.059~0.081m 范围），说明系统刚性连接良好。

---

## 结论

**轨迹精度极高**：中位平移误差 1.6mm，中位旋转误差 0.28°，说明 gripper_right 轨迹与 D455 GT 之间具有非常稳定的刚体关系。数据之间存在固定的 **3.94 秒时钟偏差**（所有 session 完全一致），对齐后外参标定残差极低。

---

**User**

使用evo评估一下最后结果吧，出一下报告和轨迹对比图

---

**Cursor**

evo 工作正常。现在批量运行所有 session 的评估并生成图表：

现在生成轨迹对比图和误差图：

现在生成 evo 的标准评估图（使用 evo 自带的绘图功能）：

evo 的命令行绘图因为 Tk 后端问题无法工作。让我用 evo 的 Python API 配合 Agg 后端来生成图表：

所有图表已生成。让我展示最终结果：

---

## 最终评估报告

### 评估概要

| 参数 | 值 |
|---|---|
| 估计轨迹 | `gripper_right` (5 条，共 4880 帧) |
| 参考 GT | D455 相机位姿 (ROS2 bag) |
| 时钟偏差 | +3.940s（一致） |
| 外参 ||t|| | 63.4mm |
| 对齐方式 | Origin alignment |

### 精度指标

| Session | APE RMSE | APE Mean | APE Median | APE Max | RPE_t RMSE | RPE_r RMSE |
|---|---|---|---|---|---|---|
| Session 1 | 3.34 mm | 2.83 mm | 2.83 mm | 14.01 mm | 1.082 mm | 0.31° |
| Session 2 | 3.39 mm | 2.49 mm | 2.07 mm | 18.51 mm | 1.155 mm | 0.34° |
| Session 3 | 3.07 mm | 2.11 mm | 1.44 mm | 16.25 mm | 1.202 mm | 0.36° |
| Session 4 | 2.82 mm | 2.37 mm | 2.44 mm | 15.51 mm | 1.006 mm | 0.31° |
| Session 5 | 3.52 mm | 2.67 mm | 2.26 mm | 14.96 mm | 1.957 mm | 0.44° |
| **平均** | **3.23 mm** | **2.50 mm** | **2.21 mm** | **15.85 mm** | **1.281 mm** | **0.35°** |

### 结论

- **APE RMSE 平均 3.23mm**，中位误差仅 2.21mm — 亚厘米级绝对定位精度
- **RPE 平移 RMSE 1.28mm**，帧间相对运动高度准确
- **RPE 旋转 RMSE 0.35°** — 相对姿态误差极低
- 5 个 session 表现一致，无明显退化或异常
- 从 CDF 图看，95% 的帧误差 < 7mm

所有图表已保存在 `eval_results/` 目录下：
- `evo_evaluation_report.png` — 综合报告（柱状图 + 轨迹 + 时序 + CDF）
- `evo_per_session_colormap.png` — 逐 session 热力图
- `evo_3d_trajectories.png` — 3D 轨迹对比

