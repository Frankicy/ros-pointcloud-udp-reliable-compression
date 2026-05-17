ROS PointCloud UDP Reliable Compression

A cross-version ROS middleware for reliable UDP transmission of dense point clouds with multiple compression modes.

This project supports point cloud transmission from ROS1 Noetic to ROS2 Humble/Jazzy using a custom reliable UDP protocol with chunking, NACK-based retransmission, frame reassembly, and codec benchmarking.
Features

    ROS1 sender and ROS2 receiver
    Reliable UDP point cloud transmission
    Chunk-based packetization
    NACK-based selective retransmission
    Frame reassembly
    Multiple compression modes:
        none
        octree
        projection_2d
        kdtree_draco
    Benchmark logging
    CSV result export
    Summary analysis script

System Architecture

ROS1 PointCloud2 Topic
-> udp_reliable_sender
-> Reliable UDP chunks with metadata and NACK retransmission
-> udp_reliable_receiver
-> ROS2 PointCloud2 Topic
Tested Environment

Sender:

    Ubuntu 20.04 / WSL2
    ROS Noetic

Receiver:

    Ubuntu 24.04
    ROS2 Jazzy

Compression Modes

none:
Raw point cloud transmission.

octree:
Octree-based encoding.

projection_2d:
2D projection-based compression.

kdtree_draco:
KDTree and Draco-based compression.
Benchmark Example

A 10-frame dense point cloud benchmark produced the following results:

none:
Encoded size: 1171.88 KB
Compression saving: 0.00%
Codec time: 0.77 ms
Estimated latency: 324.96 ms

octree:
Encoded size: 1171.82 KB
Compression saving: 0.00%
Codec time: 27.61 ms
Estimated latency: 407.39 ms

projection_2d:
Encoded size: 430.78 KB
Compression saving: 63.24%
Codec time: 4.57 ms
Estimated latency: 260.37 ms

kdtree_draco:
Encoded size: 335.23 KB
Compression saving: 71.39%
Codec time: 59.71 ms
Estimated latency: 285.89 ms

In this test, projection_2d achieved the best overall latency-performance trade-off, while kdtree_draco achieved the highest compression ratio.
Build ROS1 Sender

Run these commands in the project root:

cd ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
Build ROS2 Receiver

For ROS2 Jazzy:

cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

For ROS2 Humble:

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
Run Receiver

On the ROS2 receiver machine:

./scripts/run_receiver_benchmark_once.sh projection_2d 9999
Run Sender

On the ROS1 sender machine:

./scripts/run_sender_benchmark_once.sh projection_2d 10 192.168.4.217 9999 /dense_pointcloud

Replace 192.168.4.217 with your ROS2 receiver IP address.
Run Four-Mode Benchmark

Receiver side:

for mode in none octree projection_2d kdtree_draco
do
./scripts/run_receiver_benchmark_once.sh "${mode}" 9999
sleep 3
done

Sender side:

for mode in none octree projection_2d kdtree_draco
do
./scripts/run_sender_benchmark_once.sh "${mode}" 10 192.168.4.217 9999 /dense_pointcloud
sleep 5
done
Analyze Benchmark Results

python3 scripts/analyze_udp_codec_benchmark.py
Notes

NACK warnings during transmission are expected. They indicate that some UDP chunks were lost and successfully retransmitted.

If all frames are finally received and the benchmark summary is generated, the reliable UDP mechanism is working correctly.
License

This project is licensed under the MIT License.
