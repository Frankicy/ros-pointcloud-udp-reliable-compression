#include "udp_reliable_common/stats.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace udp_reliable {

std::string StatsRecorder::nowString() {
  auto now = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(now);

  std::tm tm{};
  localtime_r(&tt, &tm);

  char buf[64];
  std::strftime(buf, sizeof(buf), "[%y:%m:%d][%H:%M:%S]", &tm);

  return std::string(buf);
}

std::string StatsRecorder::makeTimestampedPath(const std::string &path) {
  std::string base = path.empty()
    ? "/home/aiseon/udp_reliable_ros2_ws/udp_codec_test.csv"
    : path;

  std::string dir;
  std::string file = base;

  const auto slash = base.find_last_of('/');
  if (slash != std::string::npos) {
    dir = base.substr(0, slash + 1);
    file = base.substr(slash + 1);
  }

  std::string stem = file;
  std::string ext = ".csv";

  const auto dot = file.find_last_of('.');
  if (dot != std::string::npos) {
    stem = file.substr(0, dot);
    ext = file.substr(dot);
  }

  return dir + stem + "_" + nowString() + ext;
}

void StatsRecorder::setOutputCsvPath(const std::string &path) {
  output_csv_path_ = makeTimestampedPath(path);
}

void StatsRecorder::reset() {
  records_.clear();

  frames_ = 0;
  total_reassemble_ms_ = 0.0;
  total_latency_ms_ = 0.0;
  valid_latency_count_ = 0;
  total_bytes_ = 0;

  has_pending_codec_info_ = false;

  pending_compression_mode_ = "unknown";
  pending_raw_point_count_ = 0;
  pending_encoded_point_count_ = 0;
  pending_original_size_bytes_ = 0;
  pending_encoded_size_bytes_ = 0;
  pending_sender_compress_time_ms_ = 0.0;
  pending_receiver_decompress_time_ms_ = 0.0;
}

void StatsRecorder::setLastCodecInfo(
  const std::string &compression_mode,
  uint32_t raw_point_count,
  uint32_t encoded_point_count,
  uint64_t original_size_bytes,
  uint64_t encoded_size_bytes,
  double sender_compress_time_ms,
  double receiver_decompress_time_ms
) {
  has_pending_codec_info_ = true;

  pending_compression_mode_ = compression_mode;
  pending_raw_point_count_ = raw_point_count;
  pending_encoded_point_count_ = encoded_point_count;
  pending_original_size_bytes_ = original_size_bytes;
  pending_encoded_size_bytes_ = encoded_size_bytes;
  pending_sender_compress_time_ms_ = sender_compress_time_ms;
  pending_receiver_decompress_time_ms_ = receiver_decompress_time_ms;
}

void StatsRecorder::addFrame(
  uint64_t frame_id,
  uint32_t point_count,
  size_t frame_size_bytes,
  uint32_t total_chunks,
  double reassemble_ms,
  double latency_ms,
  int64_t recv_timestamp_ns
) {
  FrameStatRecord r;

  r.frame_id = frame_id;
  r.point_count = point_count;
  r.frame_size_bytes = frame_size_bytes;
  r.total_chunks = total_chunks;

  r.reassemble_time_ms = reassemble_ms;
  r.latency_ms = latency_ms;
  r.recv_timestamp_ns = recv_timestamp_ns;

  if (has_pending_codec_info_) {
    r.compression_mode = pending_compression_mode_;
    r.raw_point_count = pending_raw_point_count_;
    r.encoded_point_count = pending_encoded_point_count_;
    r.original_size_bytes = pending_original_size_bytes_;
    r.encoded_size_bytes = pending_encoded_size_bytes_;
    r.sender_compress_time_ms = pending_sender_compress_time_ms_;
    r.receiver_decompress_time_ms = pending_receiver_decompress_time_ms_;
  } else {
    r.compression_mode = "unknown";
    r.raw_point_count = point_count;
    r.encoded_point_count = point_count;
    r.original_size_bytes = frame_size_bytes;
    r.encoded_size_bytes = frame_size_bytes;
    r.sender_compress_time_ms = 0.0;
    r.receiver_decompress_time_ms = 0.0;
  }

  if (r.original_size_bytes > 0) {
    r.compression_ratio_percent =
      static_cast<double>(r.encoded_size_bytes) * 100.0 /
      static_cast<double>(r.original_size_bytes);

    r.compression_saving_percent = 100.0 - r.compression_ratio_percent;
  } else {
    r.compression_ratio_percent = 0.0;
    r.compression_saving_percent = 0.0;
  }

  r.total_codec_time_ms = r.sender_compress_time_ms + r.receiver_decompress_time_ms;

  records_.push_back(r);

  ++frames_;
  total_reassemble_ms_ += reassemble_ms;
  total_bytes_ += frame_size_bytes;

  if (latency_ms >= 0.0) {
    total_latency_ms_ += latency_ms;
    ++valid_latency_count_;
  }

  has_pending_codec_info_ = false;
}

int StatsRecorder::frameCount() const {
  return frames_;
}

double StatsRecorder::averageReassembleMs() const {
  if (frames_ <= 0) {
    return 0.0;
  }
  return total_reassemble_ms_ / static_cast<double>(frames_);
}

double StatsRecorder::averageLatencyMs() const {
  if (valid_latency_count_ <= 0) {
    return -1.0;
  }
  return total_latency_ms_ / static_cast<double>(valid_latency_count_);
}

double StatsRecorder::averageFrameSizeKb() const {
  if (frames_ <= 0) {
    return 0.0;
  }
  return static_cast<double>(total_bytes_) / 1024.0 / static_cast<double>(frames_);
}

double StatsRecorder::totalReceivedMb() const {
  return static_cast<double>(total_bytes_) / 1024.0 / 1024.0;
}

double StatsRecorder::averageThroughputKBps() const {
  if (total_reassemble_ms_ <= 0.0) {
    return 0.0;
  }

  const double total_kb = static_cast<double>(total_bytes_) / 1024.0;
  const double total_sec = total_reassemble_ms_ / 1000.0;

  return total_kb / total_sec;
}

double StatsRecorder::averageOriginalSizeKb() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += static_cast<double>(r.original_size_bytes) / 1024.0;
  }

  return total / static_cast<double>(records_.size());
}

double StatsRecorder::averageEncodedSizeKb() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += static_cast<double>(r.encoded_size_bytes) / 1024.0;
  }

  return total / static_cast<double>(records_.size());
}

double StatsRecorder::averageCompressionRatioPercent() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += r.compression_ratio_percent;
  }

  return total / static_cast<double>(records_.size());
}

double StatsRecorder::averageCompressionSavingPercent() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += r.compression_saving_percent;
  }

  return total / static_cast<double>(records_.size());
}

double StatsRecorder::averageSenderCompressMs() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += r.sender_compress_time_ms;
  }

  return total / static_cast<double>(records_.size());
}

double StatsRecorder::averageReceiverDecompressMs() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += r.receiver_decompress_time_ms;
  }

  return total / static_cast<double>(records_.size());
}

double StatsRecorder::averageTotalCodecMs() const {
  if (records_.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &r : records_) {
    total += r.total_codec_time_ms;
  }

  return total / static_cast<double>(records_.size());
}

const std::string &StatsRecorder::outputCsvPath() const {
  return output_csv_path_;
}

std::string StatsRecorder::dominantCompressionMode() const {
  if (records_.empty()) {
    return "unknown";
  }

  std::map<std::string, int> counts;

  for (const auto &r : records_) {
    counts[r.compression_mode]++;
  }

  std::string best = "unknown";
  int best_count = -1;

  for (const auto &kv : counts) {
    if (kv.second > best_count) {
      best = kv.first;
      best_count = kv.second;
    }
  }

  return best;
}

bool StatsRecorder::saveCsv() const {
  std::ofstream f(output_csv_path_);

  if (!f) {
    return false;
  }

  f << "frame_id,"
    << "compression_mode,"
    << "raw_point_count,"
    << "encoded_point_count,"
    << "original_size_bytes,"
    << "encoded_size_bytes,"
    << "original_size_kb,"
    << "encoded_size_kb,"
    << "compression_ratio_percent,"
    << "compression_saving_percent,"
    << "sender_compress_time_ms,"
    << "receiver_decompress_time_ms,"
    << "total_codec_time_ms,"
    << "frame_size_bytes,"
    << "frame_size_kb,"
    << "total_chunks,"
    << "reassemble_time_ms,"
    << "latency_ms,"
    << "recv_timestamp_ns"
    << "\n";

  f << std::fixed << std::setprecision(6);

  for (const auto &r : records_) {
    f << r.frame_id << ","
      << r.compression_mode << ","
      << r.raw_point_count << ","
      << r.encoded_point_count << ","
      << r.original_size_bytes << ","
      << r.encoded_size_bytes << ","
      << static_cast<double>(r.original_size_bytes) / 1024.0 << ","
      << static_cast<double>(r.encoded_size_bytes) / 1024.0 << ","
      << r.compression_ratio_percent << ","
      << r.compression_saving_percent << ","
      << r.sender_compress_time_ms << ","
      << r.receiver_decompress_time_ms << ","
      << r.total_codec_time_ms << ","
      << r.frame_size_bytes << ","
      << static_cast<double>(r.frame_size_bytes) / 1024.0 << ","
      << r.total_chunks << ","
      << r.reassemble_time_ms << ","
      << r.latency_ms << ","
      << r.recv_timestamp_ns
      << "\n";
  }

  return true;
}

void StatsRecorder::printSummary() const {
  const std::string mode = dominantCompressionMode();

  std::printf("\n");
  std::printf("=== UDP Reliable Codec Summary [%s] ===\n", mode.c_str());
  std::printf("Completed At: %s\n", nowString().c_str());

  std::printf("Total Frames: %d\n", frameCount());

  std::printf("Average Original Size: %.2f KB\n", averageOriginalSizeKb());
  std::printf("Average Encoded Size: %.2f KB\n", averageEncodedSizeKb());
  std::printf("Average Compression Ratio: %.2f %%\n", averageCompressionRatioPercent());
  std::printf("Average Compression Saving: %.2f %%\n", averageCompressionSavingPercent());

  std::printf("Average Sender Compress Time: %.2f ms\n", averageSenderCompressMs());
  std::printf("Average Receiver Decompress Time: %.2f ms\n", averageReceiverDecompressMs());
  std::printf("Average Total Codec Time: %.2f ms\n", averageTotalCodecMs());

  std::printf("Average Reassemble Time: %.2f ms\n", averageReassembleMs());
  std::printf("Average UDP Payload Size: %.2f KB\n", averageFrameSizeKb());
  std::printf("Total Received Data: %.2f MB\n", totalReceivedMb());
  std::printf("Average Reassemble Throughput: %.2f KB/s\n", averageThroughputKBps());

  if (averageLatencyMs() >= 0.0) {
    std::printf("Average Estimated End-to-End Latency: %.2f ms\n", averageLatencyMs());
  } else {
    std::printf("Average Estimated End-to-End Latency: N/A\n");
  }

  std::printf("Raw data saved to: %s\n", outputCsvPath().c_str());
}

}  // namespace udp_reliable
