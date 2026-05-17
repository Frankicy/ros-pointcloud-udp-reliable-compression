#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "udp_reliable_common/chunker.hpp"
#include "udp_reliable_common/compression.hpp"
#include "udp_reliable_common/crc32.hpp"
#include "udp_reliable_common/protocol.hpp"
#include "udp_reliable_common/udp_socket.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace udp_reliable;

static std::string normalizeCompressionModeString(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  for (char &c : mode) {
    if (c == '-') {
      c = '_';
    }
  }

  return mode;
}

static CompressionMode parseCompressionModeFromStringRobust(const std::string &input) {
  const std::string mode = normalizeCompressionModeString(input);

  if (mode == "0" || mode == "none" || mode == "no" || mode == "raw") {
    return COMP_NONE;
  }

  if (mode == "1" || mode == "octree" || mode == "oct") {
    return COMP_OCTREE;
  }

  if (mode == "2" || mode == "projection_2d" || mode == "projection" || mode == "proj2d") {
    return COMP_PROJECTION_2D;
  }

  if (mode == "3" || mode == "kdtree_draco" || mode == "draco" || mode == "kd_draco") {
    return COMP_KDTREE_DRACO;
  }

  ROS_WARN("Unknown compression_mode='%s', fallback to none.", input.c_str());
  return COMP_NONE;
}

static std::string getCompressionModeParamRobust() {
  std::string mode = "none";

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 1. ROS1 private parameter: _compression_mode:=octree
  if (pnh.getParam("compression_mode", mode)) {
    return mode;
  }

  // 2. Alternative private parameter names
  if (pnh.getParam("codec_mode", mode)) {
    return mode;
  }

  if (pnh.getParam("mode", mode)) {
    return mode;
  }

  // 3. Global parameter fallback
  if (nh.getParam("compression_mode", mode)) {
    return mode;
  }

  if (nh.getParam("codec_mode", mode)) {
    return mode;
  }
  return mode;
}



static constexpr int DEFAULT_MAX_FRAMES = 10;

class UdpReliableSender {
public:
  UdpReliableSender() : nh_(), pnh_("~") {
    pnh_.param<std::string>("server_ip", server_ip_, "192.168.4.217");
    pnh_.param<int>("server_port", server_port_, 9999);
    pnh_.param<int>("chunk_size", chunk_size_, 1200);
    pnh_.param<int>("max_retries", max_retries_, 5);
    pnh_.param<int>("ack_timeout_ms", ack_timeout_ms_, 100);
    pnh_.param<int>("max_frames", max_frames_, DEFAULT_MAX_FRAMES);
    pnh_.param<int>("duration_sec", duration_sec_, 0);
    pnh_.param<std::string>("input_topic", input_topic_, "/dense_pointcloud");

    if (!sock_.openSocket()) {
      ROS_FATAL("Failed to open UDP socket.");
      ros::shutdown();
      return;
    }

    sock_.setReceiveTimeoutMs(ack_timeout_ms_);

    if (!sock_.makeAddress(server_ip_, server_port_, server_addr_)) {
      ROS_FATAL("Invalid server address: %s:%d", server_ip_.c_str(), server_port_);
      ros::shutdown();
      return;
    }

    start_time_ = std::chrono::steady_clock::now();

    sub_ = nh_.subscribe(input_topic_, 1, &UdpReliableSender::cloudCallback, this);

    ROS_INFO("UDP reliable sender started.");
    ROS_INFO("Server: %s:%d", server_ip_.c_str(), server_port_);
    ROS_INFO("Input topic: %s", input_topic_.c_str());

    // Robustly read compression mode from ROS1 private/global params and argv.

    const std::string compression_mode_param = getCompressionModeParamRobust();

    compression_mode_ = parseCompressionModeFromStringRobust(compression_mode_param);


    ROS_INFO("Requested compression_mode param: %s", compression_mode_param.c_str());


    ROS_INFO("Compression mode: %s (%u)", compressionModeName(compression_mode_).c_str(), static_cast<unsigned>(compression_mode_));
    ROS_INFO("Chunk size: %d bytes", chunk_size_);
    ROS_INFO("Max retries: %d", max_retries_);
    ROS_INFO("ACK timeout: %d ms", ack_timeout_ms_);
    if (max_frames_ > 0) {
      ROS_INFO("Max frames: %d", max_frames_);
    } else {
      ROS_INFO("Max frames: unlimited");
    }

    if (duration_sec_ > 0) {
      ROS_INFO("Duration limit: %d sec", duration_sec_);
    } else {
      ROS_INFO("Duration limit: disabled");
    }
  }

  void shutdownGracefully() {
    if (started_ && !ended_) {
      ROS_INFO("Graceful shutdown requested. Sending END packet.");
      sendControlPacket(PKT_END);
      ended_ = true;
    }
  }

private:
  bool shouldStop() const {
    if (max_frames_ > 0 && frame_id_ >= static_cast<uint64_t>(max_frames_)) {
      return true;
    }

    if (duration_sec_ > 0) {
      auto now = std::chrono::steady_clock::now();
      double elapsed_sec = std::chrono::duration<double>(now - start_time_).count();

      if (elapsed_sec >= duration_sec_) {
        return true;
      }
    }

    return false;
  }

  bool getFieldOffset(const sensor_msgs::PointCloud2ConstPtr &msg, const std::string &name, uint32_t &offset) {
    for (const auto &f : msg->fields) {
      if (f.name == name) {
        offset = f.offset;
        return true;
      }
    }
    return false;
  }

  bool pointCloudToXYZPayload(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    std::vector<uint8_t> &payload,
    uint32_t &point_count
  ) {
    EncodedPointCloud encoded;
    if (!encodePointCloud(msg, compression_mode_, encoded)) {
      ROS_ERROR("Failed to encode point cloud with compression mode: %s",
                compressionModeName(compression_mode_).c_str());
      return false;
    }

    payload.swap(encoded.payload);
    point_count = encoded.encoded_point_count;

    ROS_INFO("Encoded frame | mode=%s | raw_points=%u | encoded_points=%u | original=%.2f KB | encoded=%.2f KB | compress=%.2f ms",
             compressionModeName(encoded.mode).c_str(),
             encoded.raw_point_count,
             encoded.encoded_point_count,
             encoded.original_size_bytes / 1024.0,
             encoded.encoded_size_bytes / 1024.0,
             encoded.compress_time_ms);

    return true;
  }

  void sendControlPacket(uint16_t type) {
    PacketHeader h{};
    h.magic = MAGIC;
    h.version = VERSION;
    h.type = type;
    h.frame_id = frame_id_;
    h.chunk_id = 0;
    h.total_chunks = 0;
    h.payload_size = 0;
    h.chunk_crc32 = 0;
    h.frame_crc32 = 0;
    h.point_count = 0;
    h.stamp_ns = 0;
    h.flags = static_cast<uint32_t>(compression_mode_);

    for (int i = 0; i < 3; ++i) {
      sock_.sendTo(&h, sizeof(h), server_addr_);
      usleep(20000);
    }

    if (type == PKT_START) {
      ROS_INFO("Sent START packet.");
    } else if (type == PKT_END) {
      ROS_INFO("Sent END packet.");
    }
  }

  void sendChunk(
    uint64_t frame_id,
    const std::vector<uint8_t> &payload,
    const ChunkInfo &chunk,
    uint32_t total_chunks,
    uint32_t frame_crc,
    uint32_t point_count,
    uint64_t stamp_ns
  ) {
    PacketHeader h{};
    h.magic = MAGIC;
    h.version = VERSION;
    h.type = PKT_DATA;
    h.frame_id = frame_id;
    h.chunk_id = chunk.chunk_id;
    h.total_chunks = total_chunks;
    h.payload_size = static_cast<uint32_t>(chunk.size);
    h.chunk_crc32 = crc32_compute(payload.data() + chunk.offset, chunk.size);
    h.frame_crc32 = frame_crc;
    h.point_count = point_count;
    h.stamp_ns = stamp_ns;
    h.flags = static_cast<uint32_t>(compression_mode_);

    std::vector<uint8_t> packet(sizeof(PacketHeader) + chunk.size);
    std::memcpy(packet.data(), &h, sizeof(PacketHeader));
    std::memcpy(packet.data() + sizeof(PacketHeader), payload.data() + chunk.offset, chunk.size);

    sock_.sendTo(packet.data(), packet.size(), server_addr_);
  }

  bool waitAckOrHandleNack(
    uint64_t frame_id,
    const std::vector<uint8_t> &payload,
    const std::vector<ChunkInfo> &chunks,
    uint32_t frame_crc,
    uint32_t point_count,
    uint64_t stamp_ns
  ) {
    std::vector<uint8_t> buffer(65536);

    for (int retry = 0; retry <= max_retries_; ++retry) {
      sockaddr_in from{};
      socklen_t from_len = sizeof(from);

      ssize_t n = sock_.recvFrom(buffer.data(), buffer.size(), from, from_len);

      if (n >= static_cast<ssize_t>(sizeof(PacketHeader))) {
        PacketHeader h{};
        std::memcpy(&h, buffer.data(), sizeof(PacketHeader));

        if (h.magic != MAGIC || h.version != VERSION || h.frame_id != frame_id) {
          continue;
        }

        if (h.type == PKT_ACK) {
          return true;
        }

        if (h.type == PKT_NACK) {
          size_t payload_bytes = static_cast<size_t>(n) - sizeof(PacketHeader);
          size_t count = payload_bytes / sizeof(uint32_t);

          const uint32_t *ids = reinterpret_cast<const uint32_t *>(buffer.data() + sizeof(PacketHeader));

          ROS_WARN("Frame %lu received NACK, missing chunks: %zu", frame_id, count);

          for (size_t i = 0; i < count; ++i) {
            uint32_t id = ids[i];
            if (id < chunks.size()) {
              sendChunk(frame_id, payload, chunks[id], static_cast<uint32_t>(chunks.size()), frame_crc, point_count, stamp_ns);
            }
          }

          continue;
        }
      }

      ROS_WARN("Frame %lu ACK timeout, retransmit all chunks. retry=%d/%d", frame_id, retry + 1, max_retries_);

      for (const auto &c : chunks) {
        sendChunk(frame_id, payload, c, static_cast<uint32_t>(chunks.size()), frame_crc, point_count, stamp_ns);
      }
    }

    return false;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (busy_) {
      return;
    }

    if (shouldStop()) {
      if (!ended_) {
        ROS_INFO("Stop condition reached before sending next frame.");
        sendControlPacket(PKT_END);
        ended_ = true;
      }
      ros::shutdown();
      return;
    }

    busy_ = true;

    if (!started_) {
      sendControlPacket(PKT_START);
      started_ = true;
    }

    auto begin = std::chrono::steady_clock::now();

    std::vector<uint8_t> payload;
    uint32_t point_count = 0;

    if (!pointCloudToXYZPayload(msg, payload, point_count)) {
      busy_ = false;
      return;
    }

    uint64_t current_frame_id = frame_id_;
    uint32_t frame_crc = crc32_compute(payload.data(), payload.size());
    uint64_t stamp_ns = msg->header.stamp.toNSec();

    auto chunks = make_chunks(payload.size(), static_cast<size_t>(chunk_size_));

    for (const auto &c : chunks) {
      sendChunk(
        current_frame_id,
        payload,
        c,
        static_cast<uint32_t>(chunks.size()),
        frame_crc,
        point_count,
        stamp_ns
      );
    }

    bool ok = waitAckOrHandleNack(
      current_frame_id,
      payload,
      chunks,
      frame_crc,
      point_count,
      stamp_ns
    );

    auto end = std::chrono::steady_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end - begin).count();

    if (ok) {
      ++frame_id_;
      ROS_INFO(
        "Sent frame %lu/%d | points=%u | size=%.2f KB | chunks=%zu | total=%.2f ms",
        frame_id_,
        max_frames_,
        point_count,
        payload.size() / 1024.0,
        chunks.size(),
        total_ms
      );
    } else {
      ROS_ERROR("Frame %lu failed after retries.", current_frame_id);
      ++frame_id_;
    }

    if (shouldStop()) {
      if (max_frames_ > 0 && frame_id_ >= static_cast<uint64_t>(max_frames_)) {
        ROS_INFO("Finished %d frames.", max_frames_);
      }

      if (duration_sec_ > 0) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - start_time_).count();
        ROS_INFO("Duration condition reached. elapsed=%.2f sec", elapsed_sec);
      }

      if (!ended_) {
        sendControlPacket(PKT_END);
        ended_ = true;
      }

      ros::shutdown();
    }

    busy_ = false;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;

  UdpSocket sock_;
  sockaddr_in server_addr_{};

  std::string server_ip_;
  std::string input_topic_;
  std::string compression_mode_name_;
  CompressionMode compression_mode_ = COMP_NONE;
  int server_port_ = 9999;
  int chunk_size_ = 1200;
  int max_retries_ = 5;
  int ack_timeout_ms_ = 100;
  int max_frames_ = DEFAULT_MAX_FRAMES;
  int duration_sec_ = 0;

  std::chrono::steady_clock::time_point start_time_;

  std::atomic<bool> busy_{false};
  uint64_t frame_id_ = 0;

  bool started_ = false;
  bool ended_ = false;
};

static UdpReliableSender *g_sender_node = nullptr;

void sigintHandler(int) {
  if (g_sender_node) {
    g_sender_node->shutdownGracefully();
  }

  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_reliable_sender_node", ros::init_options::NoSigintHandler);

  UdpReliableSender node;
  g_sender_node = &node;

  std::signal(SIGINT, sigintHandler);

  ros::spin();

  node.shutdownGracefully();
  g_sender_node = nullptr;

  return 0;
}
