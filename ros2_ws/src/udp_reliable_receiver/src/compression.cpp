#include "udp_reliable_common/compression.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include <unistd.h>

namespace udp_reliable {

namespace {

template <typename T>
bool readStruct(const std::vector<uint8_t> &buf, size_t &off, T &out) {
  if (off + sizeof(T) > buf.size()) {
    return false;
  }

  std::memcpy(&out, buf.data() + off, sizeof(T));
  off += sizeof(T);
  return true;
}

std::string tempPath(const std::string &suffix) {
  char tmpl[] = "/tmp/udp_decode_XXXXXX";
  int fd = mkstemp(tmpl);
  if (fd >= 0) {
    close(fd);
    unlink(tmpl);
  }
  return std::string(tmpl) + suffix;
}

bool writeWholeFile(const std::string &path, const uint8_t *data, size_t size) {
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }

  if (size > 0) {
    f.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
  }

  return static_cast<bool>(f);
}

bool readPlyXYZ(const std::string &path, std::vector<float> &xyz, std::string &error) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    error = "failed to open decoded ply";
    return false;
  }

  std::string line;
  std::string fmt;
  size_t vertex_count = 0;

  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    if (line.rfind("format ", 0) == 0) {
      std::istringstream iss(line);
      std::string tmp;
      iss >> tmp >> fmt;
    } else if (line.rfind("element vertex ", 0) == 0) {
      std::istringstream iss(line);
      std::string a, b;
      iss >> a >> b >> vertex_count;
    } else if (line == "end_header") {
      break;
    }
  }

  if (vertex_count == 0) {
    xyz.clear();
    return true;
  }

  xyz.resize(vertex_count * 3);

  if (fmt == "binary_little_endian") {
    f.read(reinterpret_cast<char *>(xyz.data()), static_cast<std::streamsize>(vertex_count * 3 * sizeof(float)));
    if (!f) {
      error = "failed to read binary PLY vertex data";
      return false;
    }
    return true;
  }

  if (fmt == "ascii") {
    for (size_t i = 0; i < vertex_count; ++i) {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      f >> x >> y >> z;
      xyz[i * 3 + 0] = x;
      xyz[i * 3 + 1] = y;
      xyz[i * 3 + 2] = z;
    }
    return true;
  }

  error = "unsupported PLY format: " + fmt;
  return false;
}

bool decodeDracoPayload(
  const uint8_t *drc_data,
  size_t drc_size,
  std::vector<float> &xyz,
  std::string &error
) {
  std::string drc_path = tempPath(".drc");
  std::string ply_path = tempPath(".ply");

  if (!writeWholeFile(drc_path, drc_data, drc_size)) {
    error = "failed to write temp drc";
    return false;
  }

  std::string cmd =
    "draco_decoder -i " + drc_path +
    " -o " + ply_path +
    " >/tmp/udp_draco_decoder.log 2>&1";

  int ret = std::system(cmd.c_str());
  if (ret != 0) {
    error = "draco_decoder failed, see /tmp/udp_draco_decoder.log";
    std::remove(drc_path.c_str());
    std::remove(ply_path.c_str());
    return false;
  }

  bool ok = readPlyXYZ(ply_path, xyz, error);

  std::remove(drc_path.c_str());
  std::remove(ply_path.c_str());

  return ok;
}

}  // namespace

std::string compressionModeName(CompressionMode mode) {
  switch (mode) {
    case COMP_NONE: return "none";
    case COMP_OCTREE: return "octree";
    case COMP_PROJECTION_2D: return "projection_2d";
    case COMP_KDTREE_DRACO: return "kdtree_draco";
    default: return "unknown";
  }
}

bool decodePointCloudPayload(
  const std::vector<uint8_t> &payload,
  DecodedPointCloud &out,
  std::string &error
) {
  out = DecodedPointCloud{};

  if (payload.size() < sizeof(CodecPayloadHeader)) {
    error = "payload too small for CodecPayloadHeader";
    return false;
  }

  size_t off = 0;
  CodecPayloadHeader h{};
  if (!readStruct(payload, off, h)) {
    error = "failed to read CodecPayloadHeader";
    return false;
  }

  if (h.magic != CODEC_MAGIC) {
    error = "invalid codec magic";
    return false;
  }

  out.mode = static_cast<CompressionMode>(h.compression_mode);
  out.raw_point_count = h.raw_point_count;
  out.encoded_point_count = h.encoded_point_count;
  out.original_size_bytes = h.original_size_bytes;
  out.encoded_size_bytes = h.encoded_size_bytes;
  out.sender_compress_time_ms = h.compress_time_ms;
  out.stamp_sec = h.stamp_sec;
  out.stamp_nsec = h.stamp_nsec;

  auto t0 = std::chrono::steady_clock::now();

  if (out.mode == COMP_NONE || out.mode == COMP_OCTREE) {
    const size_t body_size = payload.size() - off;

    if (body_size % 12 != 0) {
      error = "XYZ body size is not divisible by 12";
      return false;
    }

    const size_t floats = body_size / sizeof(float);
    out.xyz.resize(floats);

    if (floats > 0) {
      std::memcpy(out.xyz.data(), payload.data() + off, body_size);
    }
  } else if (out.mode == COMP_PROJECTION_2D) {
    ProjectionHeader ph{};
    if (!readStruct(payload, off, ph)) {
      error = "failed to read ProjectionHeader";
      return false;
    }

    const size_t need = static_cast<size_t>(ph.cell_count) * sizeof(ProjectionCell);
    if (off + need > payload.size()) {
      error = "projection payload truncated";
      return false;
    }

    out.xyz.clear();
    out.xyz.reserve(static_cast<size_t>(ph.cell_count) * 3);

    for (uint32_t i = 0; i < ph.cell_count; ++i) {
      ProjectionCell c{};
      if (!readStruct(payload, off, c)) {
        error = "failed to read ProjectionCell";
        return false;
      }

      float x = ph.min_x + static_cast<float>(c.u) * ph.resolution;
      float y = ph.min_y + static_cast<float>(c.v) * ph.resolution;
      float z = c.z;

      out.xyz.push_back(x);
      out.xyz.push_back(y);
      out.xyz.push_back(z);
    }
  } else if (out.mode == COMP_KDTREE_DRACO) {
    DracoMetaHeader dh{};
    if (!readStruct(payload, off, dh)) {
      error = "failed to read DracoMetaHeader";
      return false;
    }

    if (dh.draco_magic != DRACO_MAGIC) {
      error = "invalid draco magic";
      return false;
    }

    if (off + static_cast<size_t>(dh.compressed_size) > payload.size()) {
      error = "draco payload truncated";
      return false;
    }

    out.sender_kdtree_time_ms = dh.kdtree_time_ms;
    out.sender_draco_encode_time_ms = dh.draco_encode_time_ms;

    if (!decodeDracoPayload(payload.data() + off, static_cast<size_t>(dh.compressed_size), out.xyz, error)) {
      return false;
    }
  } else {
    error = "unknown compression mode";
    return false;
  }

  auto t1 = std::chrono::steady_clock::now();
  out.decompress_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  return true;
}

}  // namespace udp_reliable
