# uav_audio

ROS1 Noetic / C++14 最简 WAV 播放服务，使用 `aplay`，一次只播放一个文件；新的有效请求会抢占当前播放。
音频目录包含之前验证过的 `audio/sample-6s.wav`，以及从 `drone-voice-cn.zip`
导入的 15 条 `cn-*.wav` 中文提示音；文件名与文案见 [audio/phrases.csv](audio/phrases.csv)。

## 安装与声卡

```bash
sudo apt-get install alsa-utils
aplay -l       # 硬件声卡
aplay -L       # 可用于 -D 的 ALSA 设备名
```

`~audio_device` 默认 `default`，对应 **ALSA 默认设备**。
本机 `/etc/asound.conf` 将它设为 Jetson APE 内置声卡；桌面的 USB 默认输出
由 PulseAudio 管理，因此本机 USB 喇叭建议用 `audio_device:=pulse`。
也可使用 `audio_device:=plughw:CARD=Device,DEV=0` 直接访问 USB 声卡，
但直接访问时设备不能被 PulseAudio 或其他程序独占。
此包不修改系统音频配置。

## 编译

```bash
cd ~/devices_ws
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1 -j4
source devel/setup.bash
```

## 启动和调用

```bash
source ~/devices_ws/setup_devices.bash
roslaunch uav_audio audio.launch audio_device:=pulse
```

如 ALSA 默认设备已经正确配置，可直接 `roslaunch uav_audio audio.launch`。
另一个终端：

```bash
source ~/devices_ws/setup_devices.bash
rosservice call /audio/play "filename: 'sample-6s.wav'"
rosservice call /audio/play "filename: 'cn-ready.wav'"
rosservice call /audio/play "filename: 'missing.wav'"
rosservice call /audio/play "filename: '../sample-6s.wav'"
```

服务类型：`uav_audio/PlayAudio`，请求 `string filename`，
响应 `bool success` 和 `string message`。

- 成功启动子进程：`true / Playback started`，立即返回，不等待播放完成。
- 当前正在播放：新的有效请求会立即终止并回收旧播放器，再启动新播放，返回 `true / Playback started`；不排队，旧音频不会恢复。
- 文件名或文件检查失败的请求返回 `false`，不会中断当前播放。
- 文件不存在、不可读、非普通文件或进程无法启动：`false`，附具体错误并记录 ROS 日志。
- 仅接受以小写 `.wav` 结尾的纯文件名；拒绝 `/`、`\`、`..`、NUL 和符号链接。
- 文件从包内 `audio/` 打开并通过描述符交给 `aplay`，不调用 Shell。
- `success=true` 仅表示 `aplay` 已成功启动。设备打开失败或 WAV 内容损坏等
  后续错误由 `aplay` 和节点日志报告，退出后恢复空闲；不保证实际发声或播放完成。
- 节点关闭时终止并回收其播放子进程，等待线程退出。

添加自己的 WAV 时放到本包 `audio/` 目录，然后以其文件名调用服务。

## 抢占播放自测（2026-11-01）

在 WSL Ubuntu 20.04 / ROS Noetic 的独立 Master 上编译并验证：

- 使用可控播放器验证连续 100 次抢占、8 个并发请求、同文件重新播放、旧进程回收及关闭清理；无效请求保持原播放。
- 使用真实 `aplay` 经 ALSA `null` 成功播放全部 17 个 WAV；经 WSLg PulseAudio 验证播放中切换，新请求响应约 28.5 ms，替换音频正常结束。
- 缺少播放器、播放器异常退出、无效音频设备后的错误处理和后续播放恢复均通过。

本次验证不包含飞机喇叭的声学测量。以下 2026-10-27 记录对应修改前的拒绝忙碌请求行为。

## 实机自测（2026-10-27）

在 Jetson Orin NX / ROS Noetic 上，使用独立 Master（端口 11329）完成
37 项检查，测试进程及临时文件均已清理：

- 整个 `devices_ws` 的 `catkin_make -DROS_EDITION=ROS1 -j4` 成功（退出码 0），
  新节点使用 C++14 编译，自定义 Service 正常生成。
- 服务代码生成、launch 私有参数、默认参数、空设备参数拒绝。
- 真实 `sample-6s.wav` 经 `aplay -D pulse` 路由到 USB 输出，播放正常退出（0）；
  首次服务响应约 10 ms，播放期间拒绝新请求。
- 8 个并发请求仅 1 个成功、7 个返回 busy；播放结束后通过 `rosservice call` 再次播放成功。
- 缺失文件、无读取权限、非法路径、NUL、扩展名、目录、符号链接和 FIFO 均被拒绝。
- 找不到 `aplay` 时返回失败；连续 50 次启动失败后无文件描述符或子进程泄漏。
- 不存在的 ALSA 设备导致 `aplay` 非零退出时记录错误，并自动恢复空闲。
- 播放时关闭 launch，以及只给节点发送 SIGINT，均正确回收子进程；
  后者约 0.22 秒退出，退出码 0。

播放验证依据为 USB 输出路由和 `aplay` 退出状态，未使用麦克风测量实际声压。

同日导入 `drone-voice-cn.zip` 后，15 条中文 WAV 均通过 `/audio/play` 逐条实机测试
（`audio_device:=pulse`）：服务响应约 5–7 ms，播放中重复请求均返回 busy，
每条 `aplay` 均正常退出（0），随后可继续播放下一条。
音频及原始说明、文案表已逐字节核对，与压缩包一致。
