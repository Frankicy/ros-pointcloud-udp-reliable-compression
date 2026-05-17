#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "udp_reliable_common/crc32.hpp"
#include "udp_reliable_common/compression.hpp"
#include "udp_reliable_common/frame_buffer.hpp"
#include "udp_reliable_common/protocol.hpp"
#include "udp_reliable_common/stats.hpp"
#include "udp_reliable_common/udp_socket.hpp"

#include <chrono>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace udp_reliable;

class UdpReliableReceiver : public rclcpp::Node {
public:
  UdpReliableReceiver() : Node("udp_reliable_receiver_node") {
    this->declare_parameter<int>("listen_port", 9999);
    this->declare_parameter<int>("recv_timeout_ms", 20);
    this->declare_parameter<int>("nack_interval_ms", 50);
    this->declare_parameter<std::string>("output_topic", "/udp_reliable_pointcloud");
    this->declare_parameter<std::string>("output_csv", "/home/aiseon/udp_reliable_ros2_ws/udp_reliable_no_compression_raw_data.csv");

    this->get_parameter("listen_port", listen_port_);
    this->get_parameter("recv_timeout_ms", recv_timeout_ms_);
    this->get_parameter("nack_interval_ms", nack_interval_ms_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_csv", output_csv_);

    stats_.setOutputCsvPath(output_csv_);

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    if (!sock_.openSocket()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open UDP socket.");
      rclcpp::shutdown();
      return;
    }

    if (!sock_.bindPort(listen_port_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP port %d.", listen_port_);
      rclcpp::shutdown();
      return;
    }

    sock_.setReceiveTimeoutMs(recv_timeout_ms_);

    RCLCPP_INFO(this->get_logger(), "UDP reliable receiver listening on 0.0.0.0:%d", listen_port_);
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output CSV: %s", output_csv_.c_str());

    recv_thread_ = std::thread(&UdpReliableReceiver::recvLoop, this);
  }

  ~UdpReliableReceiver() override {
    running_ = false;

    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }

    if (!summary_saved_ && stats_.frameCount() > 0) {
      saveAndPrintSummary("Node shutdown");
    }
  }

private:

  sensor_msgs::msg::PointCloud2 makeCloudFromDecoded(const DecodedPointCloud &decoded) {
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    const uint32_t points = static_cast<uint32_t>(decoded.xyz.size() / 3);

    msg.height = 1;
    msg.width = points;
    msg.is_bigendian = false;
    msg.is_dense = true;

    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<size_t>(msg.row_step));

    if (!decoded.xyz.empty()) {
      std::memcpy(msg.data.data(), decoded.xyz.data(), msg.data.size());
    }

    return msg;
  }

  void saveAndPrintSummary(const std::string &reason) {
    if (summary_saved_) {
      return;
    }

    summary_saved_ = true;

    RCLCPP_INFO(this->get_logger(), "Saving experiment result. Reason: %s", reason.c_str());

    bool csv_ok = stats_.saveCsv();
    stats_.printSummary();

    if (csv_ok) {
      RCLCPP_INFO(this->get_logger(), "CSV saved successfully: %s", stats_.outputCsvPath().c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save CSV: %s", stats_.outputCsvPath().c_str());
    }
  }

  void resetStats() {
    frames_.clear();
    stats_.reset();
    summary_saved_ = false;
    receiving_ = true;

    RCLCPP_INFO(this->get_logger(), "Received START. Reset stats and start receiving.");
  }

  void sendAck(const FrameBuffer &fb, const sockaddr_in &addr) {
    PacketHeader h{};
    h.magic = MAGIC;
    h.version = VERSION;
    h.type = PKT_ACK;
    h.frame_id = fb.frameId();
    h.chunk_id = 0;
    h.total_chunks = fb.totalChunks();
    h.payload_size = 0;
    h.chunk_crc32 = 0;
    h.frame_crc32 = fb.frameCrc32();
    h.point_count = fb.pointCount();
    h.stamp_ns = fb.stampNs();
    h.flags = 0;

    sock_.sendTo(&h, sizeof(h), addr);
  }

  void sendNack(const FrameBuffer &fb, const sockaddr_in &addr) {
    auto missing = fb.missingChunks();

    if (missing.empty()) {
      return;
    }

    PacketHeader h{};
    h.magic = MAGIC;
    h.version = VERSION;
    h.type = PKT_NACK;
    h.frame_id = fb.frameId();
    h.chunk_id = 0;
    h.total_chunks = fb.totalChunks();
    h.payload_size = static_cast<uint32_t>(missing.size() * sizeof(uint32_t));
    h.chunk_crc32 = 0;
    h.frame_crc32 = fb.frameCrc32();
    h.point_count = fb.pointCount();
    h.stamp_ns = fb.stampNs();
    h.flags = 0;

    std::vector<uint8_t> packet(sizeof(PacketHeader) + h.payload_size);
    std::memcpy(packet.data(), &h, sizeof(PacketHeader));
    std::memcpy(packet.data() + sizeof(PacketHeader), missing.data(), h.payload_size);

    sock_.sendTo(packet.data(), packet.size(), addr);
  }

  sensor_msgs::msg::PointCloud2 makePointCloudMsg(const std::vector<uint8_t> &payload, uint32_t point_count) {
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.height = 1;
    msg.width = point_count;

    msg.fields.resize(3);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    msg.data = payload;

    return msg;
  }

  void publishFrame(FrameBuffer &fb, const sockaddr_in &sender_addr) {
    auto payload = fb.reassemble();

    DecodedPointCloud decoded;
    std::string decode_error;

    if (!decodePointCloudPayload(payload, decoded, decode_error)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to decode payload: %s",
        decode_error.c_str()
      );
      return;
    }


    stats_.setLastCodecInfo(
      compressionModeName(decoded.mode),
      decoded.raw_point_count,
      decoded.encoded_point_count,
      decoded.original_size_bytes,
      decoded.encoded_size_bytes,
      decoded.sender_compress_time_ms,
      decoded.decompress_time_ms
    );

    auto decoded_msg = makeCloudFromDecoded(decoded);
    pub_->publish(decoded_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Decoded frame | mode=%s | raw_points=%u | encoded_points=%u | original=%.2f KB | encoded=%.2f KB | sender_compress=%.2f ms | decompress=%.2f ms",
      compressionModeName(decoded.mode).c_str(),
      decoded.raw_point_count,
      decoded.encoded_point_count,
      decoded.original_size_bytes / 1024.0,
      decoded.encoded_size_bytes / 1024.0,
      decoded.sender_compress_time_ms,
      decoded.decompress_time_ms
    );


    uint32_t crc = crc32_compute(payload.data(), payload.size());

    if (crc != fb.frameCrc32()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Frame %lu CRC mismatch. expected=%u actual=%u",
        fb.frameId(),
        fb.frameCrc32(),
        crc
      );
      return;
    }

    double reassemble_ms = fb.reassembleTimeMs();

    auto recv_now = std::chrono::system_clock::now();
    int64_t recv_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      recv_now.time_since_epoch()
    ).count();

    double latency_ms = -1.0;
    if (fb.stampNs() > 0 && recv_timestamp_ns > static_cast<int64_t>(fb.stampNs())) {
      latency_ms = (recv_timestamp_ns - static_cast<int64_t>(fb.stampNs())) / 1000000.0;
    }

    auto msg = makePointCloudMsg(payload, fb.pointCount());
    pub_->publish(msg);

    stats_.addFrame(
      fb.frameId(),
      fb.pointCount(),
      payload.size(),
      fb.totalChunks(),
      reassemble_ms,
      latency_ms,
      recv_timestamp_ns
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Frame %d complete | frame_id=%lu | points=%u | chunks=%u | size=%.2f KB | reassemble=%.2f ms | latency=%.2f ms",
      stats_.frameCount(),
      fb.frameId(),
      fb.pointCount(),
      fb.totalChunks(),
      payload.size() / 1024.0,
      reassemble_ms,
      latency_ms
    );

    sendAck(fb, sender_addr);
  }

  void handleDataPacket(const uint8_t *data, size_t size, const sockaddr_in &sender_addr) {
    if (!receiving_) {
      return;
    }

    if (size < sizeof(PacketHeader)) {
      return;
    }

    PacketHeader h{};
    std::memcpy(&h, data, sizeof(PacketHeader));

    if (h.magic != MAGIC || h.version != VERSION || h.type != PKT_DATA) {
      return;
    }

    if (sizeof(PacketHeader) + h.payload_size != size) {
      return;
    }

    const uint8_t *payload = data + sizeof(PacketHeader);

    uint32_t chunk_crc = crc32_compute(payload, h.payload_size);
    if (chunk_crc != h.chunk_crc32) {
      RCLCPP_WARN(
        this->get_logger(),
        "Chunk CRC mismatch. frame=%lu chunk=%u",
        h.frame_id,
        h.chunk_id
      );
      return;
    }

    auto &fb = frames_[h.frame_id];

    if (!fb.initialized()) {
      fb.initFromHeader(h);
    }

    fb.addChunk(h.chunk_id, payload, h.payload_size);

    last_sender_addr_ = sender_addr;
    have_sender_ = true;

    if (fb.isComplete()) {
      publishFrame(fb, sender_addr);
      frames_.erase(h.frame_id);
    }
  }

  void checkTimeouts() {
    if (!have_sender_) {
      return;
    }

    auto now = std::chrono::steady_clock::now();

    if (std::chrono::duration<double, std::milli>(now - last_nack_time_).count() < nack_interval_ms_) {
      return;
    }

    last_nack_time_ = now;

    for (auto &kv : frames_) {
      FrameBuffer &fb = kv.second;

      if (fb.initialized() && !fb.isComplete()) {
        sendNack(fb, last_sender_addr_);
      }
    }
  }

  void recvLoop() {
    std::vector<uint8_t> buffer(65536);

    while (rclcpp::ok() && running_) {
      sockaddr_in sender_addr{};
      socklen_t sender_len = sizeof(sender_addr);

      ssize_t n = sock_.recvFrom(buffer.data(), buffer.size(), sender_addr, sender_len);

      if (n >= static_cast<ssize_t>(sizeof(PacketHeader))) {
        PacketHeader h{};
        std::memcpy(&h, buffer.data(), sizeof(PacketHeader));

        if (h.magic == MAGIC && h.version == VERSION) {
          if (h.type == PKT_START) {
            resetStats();
          } else if (h.type == PKT_END) {
            receiving_ = false;
            RCLCPP_INFO(this->get_logger(), "Received END. Stop receiving.");

            saveAndPrintSummary("Received END packet");

            rclcpp::shutdown();
            break;
          } else if (h.type == PKT_DATA) {
            handleDataPacket(buffer.data(), static_cast<size_t>(n), sender_addr);
          }
        }
      }

      if (receiving_) {
        checkTimeouts();
      }
    }
  }

private:
  int listen_port_ = 9999;
  int recv_timeout_ms_ = 20;
  int nack_interval_ms_ = 50;
  std::string output_topic_ = "/udp_reliable_pointcloud";
  std::string output_csv_ = "/home/aiseon/udp_reliable_ros2_ws/udp_reliable_no_compression_raw_data.csv";

  UdpSocket sock_;
  std::thread recv_thread_;
  bool running_ = true;
  bool receiving_ = false;
  bool summary_saved_ = false;

  std::map<uint64_t, FrameBuffer> frames_;
  StatsRecorder stats_;

  sockaddr_in last_sender_addr_{};
  bool have_sender_ = false;

  std::chrono::steady_clock::time_point last_nack_time_ = std::chrono::steady_clock::now();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UdpReliableReceiver>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
