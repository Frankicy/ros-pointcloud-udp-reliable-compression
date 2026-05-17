#pragma once

#include <cstdint>

namespace udp_reliable {

static constexpr uint32_t MAGIC = 0x55445052;  // "UDPR"
static constexpr uint16_t VERSION = 1;

enum PacketType : uint16_t {
  PKT_DATA  = 1,
  PKT_ACK   = 2,
  PKT_NACK  = 3,
  PKT_START = 4,
  PKT_END   = 5
};

#pragma pack(push, 1)
struct PacketHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t type;

  uint64_t frame_id;
  uint32_t chunk_id;
  uint32_t total_chunks;

  uint32_t payload_size;
  uint32_t chunk_crc32;
  uint32_t frame_crc32;

  uint32_t point_count;
  uint64_t stamp_ns;
  uint32_t flags;
};
#pragma pack(pop)

}  // namespace udp_reliable
