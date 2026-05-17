#include "udp_reliable_common/crc32.hpp"

namespace udp_reliable {

uint32_t crc32_compute(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];

    for (int j = 0; j < 8; ++j) {
      if (crc & 1u) {
        crc = (crc >> 1u) ^ 0xEDB88320u;
      } else {
        crc >>= 1u;
      }
    }
  }

  return crc ^ 0xFFFFFFFFu;
}

}  // namespace udp_reliable
