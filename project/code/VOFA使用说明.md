# VOFA+ 调试（LS2K0300）

支持两种传输方式（编译时二选一）：

| 模式 | CMake | 上位机设置 |
|------|--------|------------|
| **串口**（默认） | 不加 `VOFA_USE_TCP` | 串口 115200 + JustFloat |
| **TCP** | `-DVOFA_USE_TCP=ON` | **TCP 服务端** + JustFloat |

逐飞助手仍用 **TCP 8086**，与 VOFA **1347** 互不冲突。

---

## TCP 模式（推荐与网口调试一致）

### 1. 编译

```bash
cd /home/lq/LS2K0300_Library/LS2K300_Library/Seekfree_LS2K0300_Opensource_Library/project/user/build

rm -f CMakeCache.txt

cmake .. \
  -DENABLE_VOFA=ON \
  -DVOFA_USE_TCP=ON \
  -DVOFA_TCP_IP=192.168.137.1 \
  -DVOFA_TCP_PORT=1347

make -j
```

确认 cmake 输出含：`VOFA transport: TCP 192.168.137.1:1347`

### 2. PC 端 VOFA+

1. 先打开 VOFA+，**数据接口 → TCP 服务端**，端口 `1347`（与 `VOFA_TCP_PORT` 一致）
2. **数据引擎 → JustFloat**
3. 再运行板子程序（板子作为 TCP Client 连上来）
4. 串口日志应出现 `[vofa] init ok, tcp=...`

### 3. 网络

- PC 网卡 IP 需与 `VOFA_TCP_IP` 一致（默认与逐飞助手相同 `192.168.137.1`）
- 防火墙放行 TCP 1347

---

## 串口模式

```bash
cmake .. -DVOFA_SERIAL_DEVICE=/dev/ttyACM0
make -j
```

VOFA+ 选串口 115200 + JustFloat。

---

## 波形 / 调参通道

### 发送（波形，勾选用于观察）

| TX | 变量 | 说明 |
|----|------|------|
| 0 | `img_err` | 图像偏差（像素） |
| 1 | `g_target_speed` | 目标速度 |
| 2 | `g_u_yaw` | 角速度环输出（轮速差） |
| 3 | `g_target_yaw_spd` | 角度外环输出（角速度给定） |
| 4 | `g_yaw_speed` | 陀螺仪角速度（deg/s） |
| 5 | `g_speed_l` | 左轮速度 |
| 6 | `g_speed_r` | 右轮速度 |
| 7 | `g_speed` | 平均速度 |

**速度环调参建议勾选**：TX1、TX5、TX6、TX7。  
**角速度环调参建议勾选**：TX2、TX3、TX4。

### 接收（滑块 / 输入，Hex 命令）

协议：`A6 <通道十六进制> %% <通道十六进制>`

| RX | 板端变量（`YAW_SPD_TUNE_MODE=0` 三串环） | Hex 命令 | 初值建议 | 范围建议 |
|----|----------|----------|----------|----------|
| 0 | `g_target_speed` | `A6 00 %% 00` | 0.5~1.0 | 0 ~ 2 |
| 1 | `pd_yaw.P1` | `A6 01 %% 01` | 按角度外环 | 按角度外环 |
| 2 | `pid_speed_l/r.P`（左右同步） | `A6 02 %% 02` | 5 | 0 ~ 200 |
| 3 | `pid_yaw_spd.P` | `A6 03 %% 03` | 0.015 | 按角速度环 |
| 4 | `pd_yaw.D1` | `A6 04 %% 04` | 按角度外环 | 按角度外环 |
| 5 | `pid_speed_l/r.I`（左右同步） | `A6 05 %% 05` | 15 | 0 ~ 100 |
| 6 | `pid_yaw_spd.D` | `A6 06 %% 06` | 0.006 | 按角速度环 |

**角速度环独立调参**（`app_config.h` 中 `YAW_SPD_TUNE_MODE=1`）时 RX 映射变化：

| RX | 板端变量 | Hex 命令 | 初值建议 | 范围建议 |
|----|----------|----------|----------|----------|
| 0 | `g_target_speed` | `A6 00 %% 00` | 0~0.3（原地转可 0） | 0 ~ 2 |
| 1 | **`g_target_yaw_spd`（deg/s）** | `A6 01 %% 01` | ±10 阶跃起步 | -180 ~ 180 |
| 2 | `pid_speed_l/r.P` | `A6 02 %% 02` | 已调好速度环 P | 0 ~ 200 |
| 3 | `pid_yaw_spd.P` | `A6 03 %% 03` | 0.015 | 0 ~ 1 |
| 4 | （忽略，勿用） | — | — | — |
| 5 | `pid_speed_l/r.I` | `A6 05 %% 05` | 已调好速度环 I | 0 ~ 100 |
| 6 | `pid_yaw_spd.D` | `A6 06 %% 06` | 0.006 | 0 ~ 0.05 |

速度环 **D 固定为 0**（`motor.cpp` 中 `PID_INC_INIT` 第三项），不经 VOFA 调节。

修改 RX2、RX5 后固件会自动调用 `speed_reset()`，清除增量式 PID 历史，避免突变。  
修改 RX3、RX6 后会 `PID_Pos_Reset(&pid_yaw_spd)`，避免角速度环微分历史突变。

**Windows 导入 json 后**：若发送内容显示 `EF BF BD` 而非 `A6`，请手改为 `A6 xx %% xx`（编码乱码会导致板子无法解析）。

命令组示例可导入：

- 全通道：`libraries/zf_components/vofa_plus/examples/vofa+.cmds.json`
- **速度环专用**：`libraries/zf_components/vofa_plus/examples/vofa_speed_tune.cmds.json`（RX1/RX4 为角度外环 P/D，纯速度环时置 0）
- **角速度环专用（含目标角速度）**：`libraries/zf_components/vofa_plus/examples/vofa_yaw_spd_tune.cmds.json`（VOFA+ 命令面板 → 导入）

当前工程 `app_config.h` 默认 **`YAW_SPD_TUNE_MODE=1`**，角速度环独立调参时应导入 **yaw** 版命令组，勿使用 speed 版（否则 RX1 显示为「角度P」，实际板端会当作 `g_target_yaw_spd`）。

---

## 速度环 VOFA 调参流程

1. 确认 `main.cpp` 中 `ENABLE_MOTOR_CLOSED_LOOP` 为 `1`。
2. **架空或垫高车轮**；RX0 目标速度从 **0.3~0.5** 起步。
3. **纯速度环调试**：RX1（角度 P）、RX4（角度 D）滑块设为 **0**。
4. 波形页勾选 **TX1、TX3、TX4、TX5**，观察跟踪与振荡。
5. 调参顺序：**P（RX2）→ I（RX5）**；左右轮自动相同，无需分通道。
6. 满意后将 P、I 写回 `motor.cpp` 中 `PID_INC_INIT(P, I, 0)`（D 保持 0），左右轮各一行相同数值：

```cpp
PID_Inc_Datatypedef pid_speed_l = PID_INC_INIT(P, I, 0.0f);
PID_Inc_Datatypedef pid_speed_r = PID_INC_INIT(P, I, 0.0f);
```

上位机速度滑块：目标速度(RX0)、速度P(RX2)、速度I(RX5)。角速度环：RX3(P)、RX6(D)。角度外环：RX1(P1)、RX4(D1)（`pd_yaw` 双 PD 其余项在 `motor.cpp` 写死）。

注意：控制为 **三串环分频**：角度 **6ms**（`pit_callback_yaw`）→ `g_target_yaw_spd`；角速度 **3ms**（`pit_callback_yaw_spd`）→ `g_u_yaw`；速度 **2ms**（`pit_callback_speed`）左右轮 `g_target_speed ± g_u_yaw`。

---

## 角速度环独立调参流程

适用：角度外环未接、`YAW_SPD_TUNE_MODE=1`（`main.cpp` 不注册 6ms `img_timer`），仅 **速度 2ms + 角速度 3ms** 串环。

1. 确认 `ENABLE_MOTOR_CLOSED_LOOP=1`，`app_config.h` 中 **`YAW_SPD_TUNE_MODE=1`**。
2. VOFA+ 命令面板导入 **`vofa_yaw_spd_tune.cmds.json`**（含 **RX1 目标角速度** 滑块）。
3. **架空或垫高车轮**；等待 IMU 零漂完成（`g_imu_ready`）后再给非零 RX1。
4. VOFA 波形勾选 **TX2、TX3、TX4**（`g_u_yaw`、`g_target_yaw_spd`、`g_yaw_speed`）；需要看轮速可加 TX5~TX7。
5. **初值**：
   - RX0 线速度：**0 ~ 0.3 m/s**（原地转弯可 0）
   - RX1 目标角速度：从 **±10 deg/s** 阶跃，再试 ±30、±60
   - RX3 P：从 **0.005** 扫（当前默认 0.015）
   - RX6 D：从 **0.002** 扫（当前默认 0.006）
6. **调参顺序**：固定 RX1 阶跃（如 +30 deg/s）→ 调 **P（RX3）** 至 `g_yaw_speed` 较快跟上且无明显持续振荡 → 再调 **D（RX6）** 抑制过冲。
7. **现象对照**：

| 现象 | 可能原因 | 调整 |
|------|----------|------|
| 给定角速度但车几乎不转 | P 过小 / `g_u_yaw` 限幅 ±5 | 增 P；查 `g_imu_ready` |
| 剧烈左右抖、波形毛刺大 | P 过大或 D 不足 | 减 P 或增 D |
| 能跟上但明显过冲再回摆 | D 不足 | 增 D |
| 响应很慢、无超调 | P 过小或 D 过大 | 增 P / 减 D |
| 正反转不对称 | IMU 双向增益 | 调 `imu0.hpp` 中 `IMU_GYRO_GAIN_POS/NEG` |

8. 满意后将参数写回 `motor.cpp`：

```cpp
PID_Pos_Datatypedef pid_yaw_spd = PID_POS_INIT(P, 0.0f, D, 0.0f);
```

9. **恢复整车三串环**：`app_config.h` 设 **`YAW_SPD_TUNE_MODE=0`**，重新编译；RX1/RX4 恢复为角度外环 `pd_yaw.P1/D1`，改导入 `vofa_speed_tune.cmds.json`。

---

## 关闭 VOFA

```bash
cmake .. -DENABLE_VOFA=OFF
```
