#pragma once

#include "udp_reliable_common/protocol.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace udp_reliable {

class FrameBuffer {
public:
  FrameBuffer();

  void initFromHeader(const PacketHeader &h);
  bool initialized() const;

  bool addChunk(uint32_t chunk_id, const uint8_t *data, size_t size);

  bool isComplete() const;
  std::vector<uint32_t> missingChunks() const;
  std::vector<uint8_t> reassemble() const;

  uint64_t frameId() const;
  uint32_t totalChunks() const;
  uint32_t pointCount() const;
  uint32_t frameCrc32() const;
  uint64_t stampNs() const;
  size_t receivedBytes() const;

  double reassembleTimeMs() const;

private:
  bool initialized_ = false;

  uint64_t frame_id_ = 0;
  uint32_t total_chunks_ = 0;
  uint32_t point_count_ = 0;
  uint32_t frame_crc32_ = 0;
  uint64_t stamp_ns_ = 0;

  std::vector<std::vector<uint8_t>> chunks_;
  std::vector<bool> received_;

  size_t received_bytes_ = 0;

  std::chrono::steady_clock::time_point first_chunk_time_;
};

}  // namespace udp_reliable
