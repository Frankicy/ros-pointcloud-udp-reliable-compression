#pragma once

#include <cstddef>
#include <cstdint>

namespace udp_reliable {

uint32_t crc32_compute(const uint8_t *data, size_t length);

}  // namespace udp_reliable
