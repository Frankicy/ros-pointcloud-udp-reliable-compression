#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

namespace udp_reliable {

enum CompressionMode : uint8_t {
  COMP_NONE = 0,
  COMP_OCTREE = 1,
  COMP_PROJECTION_2D = 2,
  COMP_KDTREE_DRACO = 3
};

#pragma pack(push, 1)
struct CodecPayloadHeader {
  uint32_t magic;              // 'UPCD'
  uint8_t compression_mode;    // CompressionMode
  uint8_t reserved0;
  uint16_t reserved1;

  uint64_t stamp_sec;
  uint64_t stamp_nsec;

  uint32_t raw_point_count;
  uint32_t encoded_point_count;

  uint64_t original_size_bytes;
  uint64_t encoded_size_bytes;

  double compress_time_ms;
};

struct ProjectionHeader {
  uint32_t cell_count;
  float min_x;
  float min_y;
  float resolution;
  uint32_t width;
  uint32_t height;
};

struct ProjectionCell {
  uint32_t u;
  uint32_t v;
  float z;
};

struct DracoMetaHeader {
  uint32_t draco_magic;        // 'DRAC'
  uint32_t raw_points;
  uint32_t kdtree_points;
  uint64_t raw_size;
  uint64_t compressed_size;
  double kdtree_time_ms;
  double draco_encode_time_ms;
};
#pragma pack(pop)

static constexpr uint32_t CODEC_MAGIC = 0x55504344u;   // UPCD
static constexpr uint32_t DRACO_MAGIC = 0x44524143u;   // DRAC

struct EncodedPointCloud {
  CompressionMode mode = COMP_NONE;
  uint32_t raw_point_count = 0;
  uint32_t encoded_point_count = 0;
  uint64_t original_size_bytes = 0;
  uint64_t encoded_size_bytes = 0;
  double compress_time_ms = 0.0;
  std::vector<uint8_t> payload;
};

CompressionMode parseCompressionMode(const std::string &name);
std::string compressionModeName(CompressionMode mode);

bool encodePointCloud(
  const sensor_msgs::PointCloud2ConstPtr &msg,
  CompressionMode mode,
  EncodedPointCloud &out
);

}  // namespace udp_reliable
