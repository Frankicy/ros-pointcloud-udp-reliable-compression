#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace udp_reliable {

struct ChunkInfo {
  uint32_t chunk_id;
  size_t offset;
  size_t size;
};

std::vector<ChunkInfo> make_chunks(size_t total_size, size_t chunk_size);

}  // namespace udp_reliable
