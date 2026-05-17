# ROS Point Cloud Reliable UDP Compression

A cross-version ROS point cloud transmission project based on **ROS 1 Noetic** and **ROS 2**, using **reliable UDP packetization** and **point cloud compression** for lightweight transmission.

This repository is designed for experiments on point cloud data transmission, compression ratio, latency, throughput, packet loss handling, and cross-ROS communication.

---

## 1. Project Overview

Large-scale point cloud data usually has high bandwidth requirements. Direct transmission of raw `sensor_msgs/PointCloud2` data may cause high latency, unstable throughput, and packet loss, especially in cross-machine or cross-version ROS environments.

This project implements a custom reliable UDP transmission framework for point cloud data.

Main workflow:

1. The ROS 1 sender subscribes to point cloud messages.
2. Point cloud data is serialized and optionally compressed.
3. The serialized frame is split into UDP packets.
4. The receiver collects packets and reconstructs complete frames.
5. The ROS 2 receiver validates and processes the reconstructed point cloud.
6. Benchmark scripts can be used to evaluate transmission performance.

Main goals:

- Support point cloud transmission from ROS 1 to ROS 2.
- Reduce transmission data size through compression.
- Improve robustness by frame-based packet reconstruction.
- Provide benchmark tools for latency, throughput, compression ratio, and packet statistics.

---

## 2. Repository Structure

```text
.
├── ros1_ws/
│   └── src/
│       └── udp_reliable_sender/
│           ├── include/
│           │   └── udp_reliable_common/
│           │       ├── chunker.hpp
│           │       ├── compression.hpp
│           │       ├── crc32.hpp
│           │       ├── protocol.hpp
│           │       └── udp_socket.hpp
│           ├── src/
│           │   ├── chunker.cpp
│           │   ├── compression.cpp
│           │   ├── crc32.cpp
│           │   ├── udp_reliable_sender_node.cpp
│           │   └── udp_socket.cpp
│           ├── CMakeLists.txt
│           └── package.xml
│
├── ros2_ws/
│   └── src/
│       └── udp_reliable_receiver/
│           ├── include/
│           │   └── udp_reliable_common/
│           │       ├── compression.hpp
│           │       ├── crc32.hpp
│           │       ├── frame_buffer.hpp
│           │       ├── protocol.hpp
│           │       ├── stats.hpp
│           │       └── udp_socket.hpp
│           ├── src/
│           │   ├── compression.cpp
│           │   ├── crc32.cpp
│           │   ├── frame_buffer.cpp
│           │   ├── stats.cpp
│           │   ├── udp_reliable_receiver_node.cpp
│           │   └── udp_socket.cpp
│           ├── CMakeLists.txt
│           └── package.xml
│
├── scripts/
│   ├── analyze_udp_codec_benchmark.py
│   ├── run_receiver_benchmark_once.sh
│   ├── run_sender_benchmark_once.sh
│   └── tune_udp_buffer.sh
│
├── README.md
├── LICENSE
└── .gitignore
```

---

## 3. Main Features

### 3.1 Reliable UDP Transmission

This project uses a custom UDP frame protocol instead of sending a large point cloud message directly.

The transmission design includes:

- Frame ID
- Packet ID
- Total packet count
- Payload size
- CRC32 checksum
- Receiver-side frame reconstruction
- Packet-level validation

This design is more suitable for large point cloud frames than simple one-shot UDP transmission.

---

### 3.2 Point Cloud Compression

The sender side supports point cloud data compression before UDP packetization.

Compression-related files are located at:

```text
ros1_ws/src/udp_reliable_sender/include/udp_reliable_common/compression.hpp
ros1_ws/src/udp_reliable_sender/src/compression.cpp
ros2_ws/src/udp_reliable_receiver/include/udp_reliable_common/compression.hpp
ros2_ws/src/udp_reliable_receiver/src/compression.cpp
```

The purpose of compression is to reduce transmitted bytes and compare performance against uncompressed transmission.

---

### 3.3 Benchmark Support

Benchmark scripts are provided under the `scripts/` directory:

```text
scripts/run_sender_benchmark_once.sh
scripts/run_receiver_benchmark_once.sh
scripts/analyze_udp_codec_benchmark.py
scripts/tune_udp_buffer.sh
```

The analysis script searches CSV files in:

```text
benchmark_results/
logs/
results/
current directory
```

Run:

```bash
python3 scripts/analyze_udp_codec_benchmark.py
```

If no CSV file exists, the script will output:

```text
No CSV files found.
Checked: benchmark_results, logs, results, current directory
```

---

## 4. Environment

This project is intended for a cross-ROS setup.

Example environment:

| Side | System | ROS Version | Role |
|---|---|---|---|
| Sender | Ubuntu 20.04 / WSL | ROS 1 Noetic | Point cloud sender |
| Receiver | Ubuntu 22.04 / Ubuntu 24.04 | ROS 2 Humble / Jazzy | Point cloud receiver |

Recommended dependencies:

### ROS 1 side

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

### ROS 2 side

For ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

For ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 5. Build Instructions

### 5.1 Build ROS 1 Sender

```bash
cd ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

### 5.2 Build ROS 2 Receiver

For ROS 2 Humble:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

For ROS 2 Jazzy:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 6. Run Instructions

### 6.1 Start the ROS 2 Receiver

On the receiver machine:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run udp_reliable_receiver udp_reliable_receiver_node
```

If using ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run udp_reliable_receiver udp_reliable_receiver_node
```

---

### 6.2 Start the ROS 1 Sender

On the sender machine:

```bash
cd ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun udp_reliable_sender udp_reliable_sender_node
```

Make sure that a point cloud topic is already being published on the ROS 1 side.

Typical point cloud topic:

```text
/dense_pointcloud
```

You can check available topics with:

```bash
rostopic list
```

and inspect the message type with:

```bash
rostopic info /dense_pointcloud
```

---

## 7. UDP Buffer Tuning

For large point cloud frames, the default system UDP buffer may be too small.

Run:

```bash
sudo scripts/tune_udp_buffer.sh
```

or manually set larger buffer values:

```bash
sudo sysctl -w net.core.rmem_max=268435456
sudo sysctl -w net.core.wmem_max=268435456
sudo sysctl -w net.core.rmem_default=268435456
sudo sysctl -w net.core.wmem_default=268435456
```

---

## 8. Benchmark Workflow

A typical benchmark workflow is shown below.

### Terminal 1: Start receiver benchmark

```bash
cd ros-pointcloud-udp-reliable-compression
source ros2_ws/install/setup.bash
bash scripts/run_receiver_benchmark_once.sh
```

### Terminal 2: Start sender benchmark

```bash
cd ros-pointcloud-udp-reliable-compression
source ros1_ws/devel/setup.bash
bash scripts/run_sender_benchmark_once.sh
```

### Analyze results

```bash
python3 scripts/analyze_udp_codec_benchmark.py
```

The benchmark analysis reports statistics for CSV files, including:

- frame count
- average latency
- minimum latency
- maximum latency
- throughput
- compression ratio
- packet statistics

The exact output depends on the generated CSV columns.

---

## 9. Experimental Background

This repository is part of a point cloud lightweight transmission experiment.

Previous baseline experiments used TCP transmission for cross-version ROS point cloud communication. This project focuses on evaluating reliable UDP transmission with compression.

Important experimental indicators include:

| Metric | Description |
|---|---|
| Transmission latency | Time from sender frame generation to receiver reconstruction |
| Throughput | Effective data transmission rate |
| Compression ratio | Compressed size divided by original size |
| Packet loss handling | Whether incomplete frames can be detected and dropped |
| Frame reconstruction rate | Number of complete frames reconstructed by receiver |
| Point count change | Difference before and after point cloud compression |

---

## 10. Troubleshooting

If the sender reports that it is connected but no data is transmitted, check:

1. Whether the point cloud publisher is running.
2. Whether the topic name is correct.
3. Whether the topic type is `sensor_msgs/PointCloud2`.
4. Whether sender and receiver IP/port settings match.
5. Whether the firewall allows UDP traffic.
6. Whether the UDP buffer is large enough.

If the receiver cannot reconstruct frames, check:

1. Packet size setting.
2. Frame timeout setting.
3. CRC32 validation.
4. Network packet loss.
5. MTU limitation.

---

## 11. Future Work

Planned improvements:

- Add launch files for both ROS 1 and ROS 2.
- Add configurable parameters for IP, port, packet size, and topic name.
- Add more compression modes.
- Add CSV benchmark output directly in sender and receiver nodes.
- Add visualization support in RViz2.
- Compare reliable UDP with TCP baseline.
- Test different voxel resolutions and compression settings.
- Evaluate performance on larger point cloud datasets.

---

## 12. License

This project is licensed under the terms of the license file included in this repository.

