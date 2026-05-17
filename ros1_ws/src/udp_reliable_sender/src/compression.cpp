#include "udp_reliable_common/compression.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <unordered_map>

#include <unistd.h>

namespace udp_reliable {

namespace {

struct XYZ {
  float x;
  float y;
  float z;
};

struct GridKey {
  int32_t x;
  int32_t y;
  int32_t z;

  bool operator==(const GridKey &o) const {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct GridKeyHash {
  std::size_t operator()(const GridKey &k) const {
    std::size_t h1 = std::hash<int32_t>()(k.x);
    std::size_t h2 = std::hash<int32_t>()(k.y);
    std::size_t h3 = std::hash<int32_t>()(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct ProjKey {
  uint32_t u;
  uint32_t v;

  bool operator==(const ProjKey &o) const {
    return u == o.u && v == o.v;
  }
};

struct ProjKeyHash {
  std::size_t operator()(const ProjKey &k) const {
    return (static_cast<std::size_t>(k.u) << 32) ^ static_cast<std::size_t>(k.v);
  }
};

template <typename T>
void appendStruct(std::vector<uint8_t> &out, const T &v) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
  out.insert(out.end(), p, p + sizeof(T));
}

void appendBytes(std::vector<uint8_t> &out, const uint8_t *p, size_t n) {
  out.insert(out.end(), p, p + n);
}

bool getFieldOffset(const sensor_msgs::PointCloud2ConstPtr &msg, const std::string &name, uint32_t &offset) {
  for (const auto &f : msg->fields) {
    if (f.name == name) {
      offset = f.offset;
      return true;
    }
  }
  return false;
}

bool extractXYZ(const sensor_msgs::PointCloud2ConstPtr &msg, std::vector<XYZ> &points) {
  uint32_t x_off = 0;
  uint32_t y_off = 0;
  uint32_t z_off = 0;

  if (!getFieldOffset(msg, "x", x_off) ||
      !getFieldOffset(msg, "y", y_off) ||
      !getFieldOffset(msg, "z", z_off)) {
    return false;
  }

  const uint32_t count = msg->width * msg->height;
  points.clear();
  points.reserve(count);

  for (uint32_t i = 0; i < count; ++i) {
    const uint8_t *src = &msg->data[static_cast<size_t>(i) * msg->point_step];

    XYZ p{};
    std::memcpy(&p.x, src + x_off, 4);
    std::memcpy(&p.y, src + y_off, 4);
    std::memcpy(&p.z, src + z_off, 4);

    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      points.push_back(p);
    }
  }

  return true;
}

std::vector<uint8_t> xyzToBytes(const std::vector<XYZ> &points) {
  std::vector<uint8_t> out;
  out.resize(points.size() * sizeof(XYZ));
  if (!points.empty()) {
    std::memcpy(out.data(), points.data(), out.size());
  }
  return out;
}

std::vector<XYZ> voxelDownsample(const std::vector<XYZ> &points, float voxel_size) {
  std::unordered_map<GridKey, size_t, GridKeyHash> first_index;
  first_index.reserve(points.size());

  std::vector<XYZ> out;
  out.reserve(points.size());

  for (const auto &p : points) {
    GridKey key{
      static_cast<int32_t>(std::floor(p.x / voxel_size)),
      static_cast<int32_t>(std::floor(p.y / voxel_size)),
      static_cast<int32_t>(std::floor(p.z / voxel_size))
    };

    if (first_index.find(key) == first_index.end()) {
      first_index.emplace(key, out.size());
      out.push_back(p);
    }
  }

  return out;
}

void kdtreeReorderRecursive(
  const std::vector<XYZ> &points,
  std::vector<size_t> &idx,
  size_t begin,
  size_t end,
  std::vector<XYZ> &out,
  size_t leaf_size
) {
  const size_t n = end - begin;

  if (n <= leaf_size) {
    for (size_t i = begin; i < end; ++i) {
      out.push_back(points[idx[i]]);
    }
    return;
  }

  float min_v[3] = {
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max()
  };
  float max_v[3] = {
    -std::numeric_limits<float>::max(),
    -std::numeric_limits<float>::max(),
    -std::numeric_limits<float>::max()
  };

  for (size_t i = begin; i < end; ++i) {
    const auto &p = points[idx[i]];
    const float v[3] = {p.x, p.y, p.z};
    for (int a = 0; a < 3; ++a) {
      min_v[a] = std::min(min_v[a], v[a]);
      max_v[a] = std::max(max_v[a], v[a]);
    }
  }

  int axis = 0;
  float best_range = max_v[0] - min_v[0];
  for (int a = 1; a < 3; ++a) {
    float r = max_v[a] - min_v[a];
    if (r > best_range) {
      best_range = r;
      axis = a;
    }
  }

  const size_t mid = begin + n / 2;

  std::nth_element(
    idx.begin() + begin,
    idx.begin() + mid,
    idx.begin() + end,
    [&](size_t ia, size_t ib) {
      const XYZ &a = points[ia];
      const XYZ &b = points[ib];

      if (axis == 0) return a.x < b.x;
      if (axis == 1) return a.y < b.y;
      return a.z < b.z;
    }
  );

  kdtreeReorderRecursive(points, idx, begin, mid, out, leaf_size);
  kdtreeReorderRecursive(points, idx, mid, end, out, leaf_size);
}

std::vector<XYZ> kdtreeReorder(const std::vector<XYZ> &points, size_t leaf_size = 2048) {
  if (points.size() <= leaf_size) {
    return points;
  }

  std::vector<size_t> idx(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    idx[i] = i;
  }

  std::vector<XYZ> out;
  out.reserve(points.size());
  kdtreeReorderRecursive(points, idx, 0, idx.size(), out, leaf_size);
  return out;
}

bool writeBinaryPly(const std::string &path, const std::vector<XYZ> &points) {
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }

  f << "ply\n";
  f << "format binary_little_endian 1.0\n";
  f << "element vertex " << points.size() << "\n";
  f << "property float x\n";
  f << "property float y\n";
  f << "property float z\n";
  f << "end_header\n";

  if (!points.empty()) {
    f.write(reinterpret_cast<const char *>(points.data()), static_cast<std::streamsize>(points.size() * sizeof(XYZ)));
  }

  return static_cast<bool>(f);
}

bool readWholeFile(const std::string &path, std::vector<uint8_t> &out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }

  f.seekg(0, std::ios::end);
  std::streamoff size = f.tellg();
  f.seekg(0, std::ios::beg);

  if (size < 0) {
    return false;
  }

  out.resize(static_cast<size_t>(size));
  if (size > 0) {
    f.read(reinterpret_cast<char *>(out.data()), size);
  }

  return static_cast<bool>(f) || f.eof();
}

std::string tempPath(const std::string &suffix) {
  char tmpl[] = "/tmp/udp_codec_XXXXXX";
  int fd = mkstemp(tmpl);
  if (fd >= 0) {
    close(fd);
    unlink(tmpl);
  }
  return std::string(tmpl) + suffix;
}

bool encodeDraco(const std::vector<XYZ> &points, std::vector<uint8_t> &drc_payload) {
  std::string ply_path = tempPath(".ply");
  std::string drc_path = tempPath(".drc");

  if (!writeBinaryPly(ply_path, points)) {
    return false;
  }

  std::string cmd =
    "draco_encoder -i " + ply_path +
    " -o " + drc_path +
    " -point_cloud -qp 14 -cl 7 >/tmp/udp_draco_encoder.log 2>&1";

  int ret = std::system(cmd.c_str());
  if (ret != 0) {
    std::remove(ply_path.c_str());
    std::remove(drc_path.c_str());
    return false;
  }

  bool ok = readWholeFile(drc_path, drc_payload);

  std::remove(ply_path.c_str());
  std::remove(drc_path.c_str());

  return ok;
}

void prependCodecHeader(
  const sensor_msgs::PointCloud2ConstPtr &msg,
  EncodedPointCloud &out,
  const std::vector<uint8_t> &body
) {
  CodecPayloadHeader h{};
  h.magic = CODEC_MAGIC;
  h.compression_mode = static_cast<uint8_t>(out.mode);
  h.reserved0 = 0;
  h.reserved1 = 0;
  h.stamp_sec = msg->header.stamp.sec;
  h.stamp_nsec = msg->header.stamp.nsec;
  h.raw_point_count = out.raw_point_count;
  h.encoded_point_count = out.encoded_point_count;
  h.original_size_bytes = out.original_size_bytes;
  h.encoded_size_bytes = body.size();
  h.compress_time_ms = out.compress_time_ms;

  out.encoded_size_bytes = body.size();
  out.payload.clear();
  out.payload.reserve(sizeof(CodecPayloadHeader) + body.size());
  appendStruct(out.payload, h);
  appendBytes(out.payload, body.data(), body.size());
}

}  // namespace

CompressionMode parseCompressionMode(const std::string &name) {
  if (name == "0" || name == "none" || name == "raw") {
    return COMP_NONE;
  }
  if (name == "1" || name == "octree") {
    return COMP_OCTREE;
  }
  if (name == "2" || name == "projection_2d" || name == "projection") {
    return COMP_PROJECTION_2D;
  }
  if (name == "3" || name == "kdtree_draco" || name == "draco") {
    return COMP_KDTREE_DRACO;
  }
  return COMP_NONE;
}

std::string compressionModeName(CompressionMode mode) {
  switch (mode) {
    case COMP_NONE: return "none";
    case COMP_OCTREE: return "octree";
    case COMP_PROJECTION_2D: return "projection_2d";
    case COMP_KDTREE_DRACO: return "kdtree_draco";
    default: return "unknown";
  }
}

bool encodePointCloud(
  const sensor_msgs::PointCloud2ConstPtr &msg,
  CompressionMode mode,
  EncodedPointCloud &out
) {
  std::vector<XYZ> points;
  if (!extractXYZ(msg, points)) {
    return false;
  }

  if (points.empty()) {
    return false;
  }

  out = EncodedPointCloud{};
  out.mode = mode;
  out.raw_point_count = static_cast<uint32_t>(points.size());
  out.original_size_bytes = static_cast<uint64_t>(points.size() * sizeof(XYZ));

  auto t0 = std::chrono::steady_clock::now();

  std::vector<uint8_t> body;

  if (mode == COMP_NONE) {
    out.encoded_point_count = out.raw_point_count;
    body = xyzToBytes(points);
  } else if (mode == COMP_OCTREE) {
    // 对齐旧版 tcp_octree_sender.py: voxel_size=0.01
    std::vector<XYZ> down = voxelDownsample(points, 0.01f);
    out.encoded_point_count = static_cast<uint32_t>(down.size());
    body = xyzToBytes(down);
  } else if (mode == COMP_PROJECTION_2D) {
    constexpr float resolution = 0.05f;

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();

    for (const auto &p : points) {
      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
    }

    uint32_t width = static_cast<uint32_t>(std::floor((max_x - min_x) / resolution)) + 1;
    uint32_t height = static_cast<uint32_t>(std::floor((max_y - min_y) / resolution)) + 1;

    std::unordered_map<ProjKey, float, ProjKeyHash> grid;
    grid.reserve(points.size());

    for (const auto &p : points) {
      uint32_t u = static_cast<uint32_t>((p.x - min_x) / resolution);
      uint32_t v = static_cast<uint32_t>((p.y - min_y) / resolution);
      ProjKey key{u, v};

      auto it = grid.find(key);
      if (it == grid.end()) {
        grid.emplace(key, p.z);
      } else if (p.z > it->second) {
        it->second = p.z;
      }
    }

    ProjectionHeader ph{};
    ph.cell_count = static_cast<uint32_t>(grid.size());
    ph.min_x = min_x;
    ph.min_y = min_y;
    ph.resolution = resolution;
    ph.width = width;
    ph.height = height;

    appendStruct(body, ph);

    for (const auto &kv : grid) {
      ProjectionCell c{};
      c.u = kv.first.u;
      c.v = kv.first.v;
      c.z = kv.second;
      appendStruct(body, c);
    }

    out.encoded_point_count = ph.cell_count;
  } else if (mode == COMP_KDTREE_DRACO) {
    auto kt0 = std::chrono::steady_clock::now();
    std::vector<XYZ> ordered = kdtreeReorder(points);
    auto kt1 = std::chrono::steady_clock::now();

    std::vector<uint8_t> drc;
    auto de0 = std::chrono::steady_clock::now();
    if (!encodeDraco(ordered, drc)) {
      return false;
    }
    auto de1 = std::chrono::steady_clock::now();

    DracoMetaHeader dh{};
    dh.draco_magic = DRACO_MAGIC;
    dh.raw_points = static_cast<uint32_t>(points.size());
    dh.kdtree_points = static_cast<uint32_t>(ordered.size());
    dh.raw_size = static_cast<uint64_t>(points.size() * sizeof(XYZ));
    dh.compressed_size = static_cast<uint64_t>(drc.size());
    dh.kdtree_time_ms = std::chrono::duration<double, std::milli>(kt1 - kt0).count();
    dh.draco_encode_time_ms = std::chrono::duration<double, std::milli>(de1 - de0).count();

    appendStruct(body, dh);
    appendBytes(body, drc.data(), drc.size());

    out.encoded_point_count = static_cast<uint32_t>(ordered.size());
  } else {
    return false;
  }

  auto t1 = std::chrono::steady_clock::now();
  out.compress_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  prependCodecHeader(msg, out, body);
  return true;
}

}  // namespace udp_reliable
