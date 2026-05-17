#include "udp_reliable_common/frame_buffer.hpp"

namespace udp_reliable {

FrameBuffer::FrameBuffer() {}

void FrameBuffer::initFromHeader(const PacketHeader &h) {
  initialized_ = true;

  frame_id_ = h.frame_id;
  total_chunks_ = h.total_chunks;
  point_count_ = h.point_count;
  frame_crc32_ = h.frame_crc32;
  stamp_ns_ = h.stamp_ns;

  chunks_.clear();
  received_.clear();

  chunks_.resize(total_chunks_);
  received_.resize(total_chunks_, false);

  received_bytes_ = 0;
  first_chunk_time_ = std::chrono::steady_clock::now();
}

bool FrameBuffer::initialized() const {
  return initialized_;
}

bool FrameBuffer::addChunk(uint32_t chunk_id, const uint8_t *data, size_t size) {
  if (!initialized_) {
    return false;
  }

  if (chunk_id >= total_chunks_) {
    return false;
  }

  if (!received_[chunk_id]) {
    chunks_[chunk_id].assign(data, data + size);
    received_[chunk_id] = true;
    received_bytes_ += size;
  }

  return true;
}

bool FrameBuffer::isComplete() const {
  if (!initialized_) {
    return false;
  }

  for (bool r : received_) {
    if (!r) {
      return false;
    }
  }

  return true;
}

std::vector<uint32_t> FrameBuffer::missingChunks() const {
  std::vector<uint32_t> missing;

  if (!initialized_) {
    return missing;
  }

  for (uint32_t i = 0; i < received_.size(); ++i) {
    if (!received_[i]) {
      missing.push_back(i);
    }
  }

  return missing;
}

std::vector<uint8_t> FrameBuffer::reassemble() const {
  std::vector<uint8_t> out;

  size_t total = 0;
  for (const auto &c : chunks_) {
    total += c.size();
  }

  out.reserve(total);

  for (const auto &c : chunks_) {
    out.insert(out.end(), c.begin(), c.end());
  }

  return out;
}

uint64_t FrameBuffer::frameId() const {
  return frame_id_;
}

uint32_t FrameBuffer::totalChunks() const {
  return total_chunks_;
}

uint32_t FrameBuffer::pointCount() const {
  return point_count_;
}

uint32_t FrameBuffer::frameCrc32() const {
  return frame_crc32_;
}

uint64_t FrameBuffer::stampNs() const {
  return stamp_ns_;
}

size_t FrameBuffer::receivedBytes() const {
  return received_bytes_;
}

double FrameBuffer::reassembleTimeMs() const {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(now - first_chunk_time_).count();
}

}  // namespace udp_reliable
