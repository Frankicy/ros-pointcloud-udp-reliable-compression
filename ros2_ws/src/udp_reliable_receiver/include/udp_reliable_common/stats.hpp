#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace udp_reliable {

struct FrameStatRecord {
  uint64_t frame_id = 0;

  uint32_t point_count = 0;
  size_t frame_size_bytes = 0;
  uint32_t total_chunks = 0;

  double reassemble_time_ms = 0.0;
  double latency_ms = -1.0;
  int64_t recv_timestamp_ns = 0;

  std::string compression_mode = "unknown";

  uint32_t raw_point_count = 0;
  uint32_t encoded_point_count = 0;

  uint64_t original_size_bytes = 0;
  uint64_t encoded_size_bytes = 0;

  double compression_ratio_percent = 0.0;
  double compression_saving_percent = 0.0;

  double sender_compress_time_ms = 0.0;
  double receiver_decompress_time_ms = 0.0;
  double total_codec_time_ms = 0.0;
};

class StatsRecorder {
public:
  void setOutputCsvPath(const std::string &path);

  void reset();

  void setLastCodecInfo(
    const std::string &compression_mode,
    uint32_t raw_point_count,
    uint32_t encoded_point_count,
    uint64_t original_size_bytes,
    uint64_t encoded_size_bytes,
    double sender_compress_time_ms,
    double receiver_decompress_time_ms
  );

  void addFrame(
    uint64_t frame_id,
    uint32_t point_count,
    size_t frame_size_bytes,
    uint32_t total_chunks,
    double reassemble_ms,
    double latency_ms,
    int64_t recv_timestamp_ns
  );

  int frameCount() const;

  double averageReassembleMs() const;
  double averageLatencyMs() const;
  double averageFrameSizeKb() const;
  double totalReceivedMb() const;
  double averageThroughputKBps() const;

  double averageOriginalSizeKb() const;
  double averageEncodedSizeKb() const;
  double averageCompressionRatioPercent() const;
  double averageCompressionSavingPercent() const;
  double averageSenderCompressMs() const;
  double averageReceiverDecompressMs() const;
  double averageTotalCodecMs() const;

  bool saveCsv() const;
  const std::string &outputCsvPath() const;

  void printSummary() const;

private:
  static std::string makeTimestampedPath(const std::string &path);
  static std::string nowString();
  std::string dominantCompressionMode() const;

private:
  std::string output_csv_path_ = "/home/aiseon/udp_reliable_ros2_ws/udp_codec_test.csv";

  std::vector<FrameStatRecord> records_;

  int frames_ = 0;
  double total_reassemble_ms_ = 0.0;
  double total_latency_ms_ = 0.0;
  int valid_latency_count_ = 0;
  size_t total_bytes_ = 0;

  bool has_pending_codec_info_ = false;

  std::string pending_compression_mode_ = "unknown";
  uint32_t pending_raw_point_count_ = 0;
  uint32_t pending_encoded_point_count_ = 0;
  uint64_t pending_original_size_bytes_ = 0;
  uint64_t pending_encoded_size_bytes_ = 0;
  double pending_sender_compress_time_ms_ = 0.0;
  double pending_receiver_decompress_time_ms_ = 0.0;
};

}  // namespace udp_reliable
