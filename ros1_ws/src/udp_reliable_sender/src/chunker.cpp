#include "udp_reliable_common/chunker.hpp"

namespace udp_reliable {

std::vector<ChunkInfo> make_chunks(size_t total_size, size_t chunk_size) {
  std::vector<ChunkInfo> chunks;

  if (chunk_size == 0) {
    return chunks;
  }

  size_t offset = 0;
  uint32_t id = 0;

  while (offset < total_size) {
    size_t remain = total_size - offset;
    size_t n = remain < chunk_size ? remain : chunk_size;

    chunks.push_back(ChunkInfo{id, offset, n});

    offset += n;
    ++id;
  }

  return chunks;
}

}  // namespace udp_reliable
