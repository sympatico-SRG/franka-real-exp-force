# Franka FR3 + GELLO 双机数采启动说明（含可选关节力矩）

本文对应服务器仓库 `/home/pair1/srg/real-exp-force` 的当前代码。系统由两台机器组成：

- **机器人端 client**：连接 FR3、GELLO 和末端执行器，运行 ROS 2 控制节点。
- **数据端 server**：连接 RealSense，相机图像、机器人状态和动作经 ROS 2 汇合后，由 ZMQ bridge 交给 LeRobot recorder。

本次新增功能只从现有 `franka_msgs/msg/FrankaRobotState` 消息读取状态，不新建 FCI/libfranka 连接，不修改控制律、控制频率、动作命令或安全参数。力矩记录默认关闭。

已对照 2026-10-13 上传的实际 client 快照：client 为 Ubuntu 22.04、ROS 2 Humble、Python 3.10，使用 `rmw_cyclonedds_cpp` 和 `ROS_DOMAIN_ID=0`；server 为 ROS 2 Jazzy。client 主仓库基线 `a85aa1c`、GELLO `abe9fcd`、LeRobot `4eecbad` 与本分支的修改基线一致。

> 当前仅完成无机器人软件测试。第一次连接真实 FR3 时必须由熟悉系统的人员在场，并按实验室安全流程执行。

## 1. 数据内容

开启后，每一帧新增两个 `float32` 字段，单位均为 `N.m`：

- `observation.joint_torque`：测量关节力矩，对应 `RobotState.measured_joint_state.effort`（libfranka `tau_J`）。
- `observation.external_joint_torque`：滤波后的外部关节力矩估计，对应 `RobotState.tau_ext_hat_filtered.effort`。

单臂 shape 为 `(7,)`；双臂 shape 为 `(14,)`，顺序固定为左臂 7 维、右臂 7 维。它们不会覆盖 `observation.state`。末端外力 `O_F_ext_hat_K` 本次不写入数据集，避免把关节力矩和笛卡尔外力混为一谈。

关闭 `record_torque` 时不会注册上述 feature，也不会改变旧 observation、数据格式或录制行为。

## 2. 启动前必须确认

1. 两台机器物理急停、FR3 Desk、FCI、GELLO 零位和工作空间均已按原流程检查。
2. client 与 server 使用同一局域网、相同 `ROS_DOMAIN_ID`（当前默认 `0`），DDS 可以跨机发现 Topic。
3. 两台机器必须用 chrony/NTP 对时。力矩时间戳来自机器人端，图像时间戳来自数据端；时钟不同步会被同步检查拒绝，不能用调大阈值掩盖。
4. 当前 server 上另有 `/home/pair1/gzy/real-exp` 任务可能占用 RealSense、8080/8081 或 GPU。启动前先与使用者确认并停止冲突任务；不要直接结束他人的进程。
5. client 必须已经发布完整的 Franka robot-state Topic。新增力矩不要求额外启动机器人连接。

两台机器分别检查时间：

```bash
timedatectl status
chronyc tracking
```

server 上检查冲突端口和相机进程：

```bash
ss -ltnp | grep -E ':5555|:8080|:8081'
ps -ef | grep -E 'realsense|start_data_collection' | grep -v grep
```

## 3. 首次构建

### 3.1 在 client 构建个人副本

不要覆盖 `/home/landau/real-exp`。在个人副本中执行：

```bash
cd /home/landau/srg/franka-real-exp-force/gello_software/ros2
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/local_setup.bash
colcon build --packages-select \
  franka_fr3_arm_controllers \
  franka_gello_state_publisher \
  franka_gripper_manager \
  --symlink-install
source install/local_setup.bash
```

该步骤只编译个人目录中的代码，不启动 ROS 控制节点，也不会连接机器人。

运行无运动静态预检：

```bash
cd /home/landau/srg/franka-real-exp-force
./scripts/check_data_collection_client.sh
```

预期显示 `Static Franka torque interface check: PASS`。该脚本用 Python 检查消息定义，因为实际 client 没有安装 `ros2 interface` 命令扩展。

### 3.2 在 server 构建 bridge

在 server 执行：

```bash
cd /home/pair1/srg/real-exp-force/gello_software/ros2
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/local_setup.bash
colcon build --packages-select franka_lerobot_data_bridge --symlink-install
source install/local_setup.bash
```

只修改 YAML 开关时一般不需要重新编译 Python，但每次新终端都要 source ROS、Franka 和本仓库 overlay。

## 4. 开启或关闭力矩记录

推荐通过 server 启动参数开启，不需要修改 YAML：

```bash
./scripts/start_data_collection_server.sh --duo --gripper --record-torque
```

不传 `--record-torque` 即为关闭，启动脚本会显式覆盖为 `false`，旧数据集 schema 保持不变。YAML 中继续保留安全默认值和同步阈值：

```yaml
record_torque: false
max_torque_age_sec: 0.1
max_torque_sample_skew_sec: 0.05
```

`max_torque_age_sec` 限制状态的新鲜度；`max_torque_sample_skew_sec` 限制力矩与关节状态、动作、末端位姿和所选相机帧的时间差。缺失、过期、来自未来或超出时间差的样本会等待下一帧，而不是沿用旧力矩。

## 5. 启动顺序

### 5.1 先在 client 启动控制链路

下面以双臂二指夹爪为例；单臂用 `--left` 或 `--right`，只采机械臂可使用 `--arm`。路径以 client 实际部署路径为准：

```bash
cd /home/landau/srg/franka-real-exp-force
./scripts/start_data_collection_client.sh --duo --gripper
```

此命令会连接真实机器人。操作者必须留在机器人旁，确保急停可立即触达。不要为了读力矩另开 libfranka/FCI 会话。

client 启动成功后，可在另一个 client 终端做只读 Topic 预检：

```bash
cd /home/landau/srg/franka-real-exp-force
./scripts/check_data_collection_client.sh --duo
```

此命令只订阅并验证左右各一条真实 `FrankaRobotState`，不发布命令。单臂分别使用 `--left` 或 `--right`。

### 5.2 在 server 检查 RobotState（只读，不启动机械臂）

```bash
cd /home/pair1/srg/real-exp-force
# 离线检查：系统 Python、ROS、Franka 消息类型支持与序列化
bash scripts/check_data_collection_server.sh
# 仅订阅已经存在的左右状态发布者，最多等待 10 秒
bash scripts/check_data_collection_server.sh --duo --timeout 10
# 需要查看原始 Cyclone DDS 警告时添加 --verbose
```

服务器默认 Jazzy；可通过 `DATA_SERVER_ROS_DISTRO` 指定已安装的其他发行版。
检查脚本和数采启动脚本默认使用与已审计 Humble client 一致的
`rmw_cyclonedds_cpp`，保留显式 `RMW_IMPLEMENTATION` 覆盖。
检查使用系统 Python，不依赖 Conda，不修改调用者环境或 `.bashrc`。

单臂使用 `--left` 或 `--right`。只有真正收到消息，且两组 effort 都包含
7 个有限值、header 时间戳非零且有效，才报告 live PASS；这不替代录制时的新鲜度和同步检查。
只有订阅者而没有发布者时会明确失败（退出码 2），不会启动或恢复机器人。

Humble client / Jazzy server 的 DDS 发现可能出现 `Failed to parse type hash`、
`Topic type hash: INVALID` 和未知节点名。缺少哈希不能单独判定数据损坏，
发现发布者也不能证明已收到数据。检查脚本默认仅提高 Cyclone DDS logger 的门槛至 ERROR，
减少发现警告；`--verbose` 保留原始警告。此设置仅影响本次检查，不修复缺失的类型哈希，
不保证跨发行版兼容，也不改变普通 `ros2 topic info` 的行为。

如果有发布者但收不到样本，需核对两端完整 `franka_msgs` 及其依赖消息定义、DDS 和网络。
从根本上消除跨发行版差异应统一双方 ROS/RMW 版本；本次受限于只修改服务器 `srg/`，
没有改动系统 ROS、其他工作空间或机器人端环境。

### 5.3 在 server 启动相机、bridge 和 recorder

```bash
cd /home/pair1/srg/real-exp-force
./scripts/start_data_collection_server.sh \
  --duo --gripper --record-torque --record \
  --local-dir /home/pair1/srg/datasets/fr3_torque_test \
  --repo-id local/fr3_torque_test \
  --bridge-host 192.168.50.13 \
  --ros-distro jazzy
```

launcher 会等待需要的 ROS Topic。client 应先启动，否则等待超时后应排查 Topic，而不是反复启动机器人控制节点。

recorder 终端按键沿用项目原逻辑：

- `s`：开始/结束一个 episode；
- `e`：保存当前 episode；
- `d`：丢弃当前 episode；
- `q`：退出。

先录一个很短的静止测试 episode，确认字段、时间戳和磁盘写入都正确，再进行正式示教。

## 6. 检查实时同步状态

bridge 日志若持续出现以下等待原因，不应继续正式采集：

- torque sample missing：尚未收到力矩；
- stale：力矩已过期；
- future timestamp：两台机器时钟不同步；
- joint/action/camera/end-effector skew：力矩与其他模态时间差超限。

首先修复 Topic 频率、网络或系统对时。只有确认硬件频率本来就需要更宽容的阈值后，才在实验负责人认可下修改 YAML。

可分别查看频率：

```bash
ros2 topic hz /left/franka_robot_state_broadcaster/robot_state
ros2 topic hz /left/gello/accepted_joint_states
```

## 7. 录制后验证数据

力矩开启的数据集会额外生成：

```text
meta/real_exp_torque_config.json
```

该文件记录 source、单位、手臂顺序和 feature shape。可在 server 的 `lerobot` 环境检查数据集：

```bash
cd /home/pair1/srg/real-exp-force
conda run -n lerobot python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = "/home/pair1/srg/datasets/fr3_torque_test"
ds = LeRobotDataset("local/fr3_torque_test", root=root)
print(ds.features["observation.joint_torque"])
print(ds.features["observation.external_joint_torque"])
sample = ds[0]
print("tau_J:", sample["observation.joint_torque"], sample["observation.joint_torque"].shape)
print("tau_ext:", sample["observation.external_joint_torque"], sample["observation.external_joint_torque"].shape)
PY
```

预期 dtype 为 `torch.float32`，单臂 7 维、双臂 14 维，单位在元数据中为 `N.m`。数值是否符合真实 FR3 物理状态仍需连接硬件后由操作者判断。

## 8. 停止顺序

1. 在 recorder 中保存或丢弃当前 episode，然后按 `q`。
2. `Ctrl-C` 停止 server launcher，确认相机和 bridge 已退出。
3. 最后按原实验流程停止 client 控制链路并退出 FCI。
4. 检查数据目录和 `meta/real_exp_torque_config.json`，再开始下一次实验。

## 9. 兼容、回滚与待验证项

- 最快回滚：启动时移除 `--record-torque`；YAML 默认值保持为 `false`。
- Git 回滚：对本次提交执行 `git revert <commit>`，不要在多人共享仓库使用 `reset --hard`。
- 旧数据集没有 torque feature 和 torque metadata，仍按旧 schema 读取。
- 旧模型不应请求新增字段，关闭开关时推理和训练输入保持不变。
- 部署配置固定关闭力矩记录，防止训练外的推理链路意外改变。

仍需在真实系统验证：两台主机 ROS 时钟偏差、RobotState 实际发布频率、双臂字段顺序、长时间丢帧率、不同负载下 `tau_J`/`tau_ext_hat_filtered` 的物理合理性，以及紧急停止和正常退出流程。本机软件测试不能替代这些硬件验收。


### 只读 CLI：过滤缺失哈希警告

```bash
bash scripts/ros2_topic_readonly.sh info /left/franka_robot_state_broadcaster/robot_state -v
bash scripts/ros2_topic_readonly.sh echo /left/franka_robot_state_broadcaster/robot_state --field measured_joint_state.effort
```

此入口只允许 topic info/echo，强制 --no-daemon，echo 默认 --once，并有 12 秒超时。
仅过滤 Cyclone DDS 的 Failed to parse type hash / USER_DATA '(null)' 警告行，保留其他警告、错误和消息正文。
添加 --raw-warnings 可显示原始警告。这不修复类型哈希；INVALID / 未知节点名可能仍显示，普通 ros2 命令不受影响。
检查入口在未设置 CYCLONEDDS_URI 时读取已有的 ~/.config/cyclonedds.xml（不修改该文件），以使用服务器已配置的 DDS 网卡。
