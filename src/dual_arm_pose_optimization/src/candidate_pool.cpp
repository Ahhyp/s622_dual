// src/candidate_pool.cpp
#include "dual_arm_pose_optimization/candidate_pool.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace dual_arm_pose_optimization {

namespace {

// C4.1 record layout, frozen in dual_arm_workspace_analysis/workspace_sampler.hpp
constexpr int kFields = 37;
constexpr int kPLeft = 12;
constexpr int kPRight = 15;
constexpr int kPObject = 18;
constexpr int kDArm = 27;

/// Read-only memory map of C4.1's float32 record file (411 MB; never copied).
class MappedRecords {
public:
  explicit MappedRecords(const std::string& path) {
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0) throw SceneLoadError("WorkspaceIndex: cannot open " + path);
    struct stat st {};
    if (::fstat(fd_, &st) != 0) throw SceneLoadError("WorkspaceIndex: fstat failed on " + path);
    bytes_ = static_cast<std::size_t>(st.st_size);
    if (bytes_ % (kFields * sizeof(float)) != 0) {
      throw SceneLoadError("WorkspaceIndex: " + path +
                           " size is not a multiple of the record size");
    }
    void* m = ::mmap(nullptr, bytes_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (m == MAP_FAILED) throw SceneLoadError("WorkspaceIndex: mmap failed on " + path);
    data_ = static_cast<const float*>(m);
    n_ = bytes_ / (kFields * sizeof(float));
  }
  ~MappedRecords() {
    if (data_ != nullptr) ::munmap(const_cast<float*>(data_), bytes_);
    if (fd_ >= 0) ::close(fd_);
  }
  MappedRecords(const MappedRecords&) = delete;
  MappedRecords& operator=(const MappedRecords&) = delete;

  std::uint64_t size() const { return n_; }
  const float* rec(std::uint64_t i) const { return data_ + i * kFields; }

private:
  int fd_ = -1;
  const float* data_ = nullptr;
  std::size_t bytes_ = 0;
  std::uint64_t n_ = 0;
};

Eigen::Vector3d vec(const float* r, int off) {
  return Eigen::Vector3d(r[off], r[off + 1], r[off + 2]);
}

}  // namespace

std::size_t WorkspaceIndex::index(int i, int j, int k) const {
  return (static_cast<std::size_t>(i) * static_cast<std::size_t>(ny_) +
          static_cast<std::size_t>(j)) *
             static_cast<std::size_t>(nz_) +
         static_cast<std::size_t>(k);
}

bool WorkspaceIndex::has(const std::vector<std::uint8_t>& m,
                         const Eigen::Vector3d& p) const {
  const int i = static_cast<int>(std::floor((p.x() - origin_.x()) / voxel_));
  const int j = static_cast<int>(std::floor((p.y() - origin_.y()) / voxel_));
  const int k = static_cast<int>(std::floor((p.z() - origin_.z()) / voxel_));
  if (i < 0 || j < 0 || k < 0 || i >= nx_ || j >= ny_ || k >= nz_) return false;
  return m[index(i, j, k)] != 0;
}

bool WorkspaceIndex::leftHas(const Eigen::Vector3d& p) const { return has(left_, p); }
bool WorkspaceIndex::rightHas(const Eigen::Vector3d& p) const { return has(right_, p); }
bool WorkspaceIndex::dualHas(const Eigen::Vector3d& p) const {
  return leftHas(p) && rightHas(p);
}

WorkspaceIndex WorkspaceIndex::load(const Options& options) {
  MappedRecords rec(options.c41_bin);
  WorkspaceIndex idx;
  idx.voxel_ = options.voxel;
  idx.total_records_ = rec.size();
  if (idx.total_records_ == 0) throw SceneLoadError("WorkspaceIndex: empty C4.1 file");

  // ---- pass 1: bounds over the TCP points ------------------------------
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d hi = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());
  for (std::uint64_t i = 0; i < rec.size(); ++i) {
    const float* r = rec.rec(i);
    for (const auto& v : {vec(r, kPLeft), vec(r, kPRight)}) {
      lo = lo.cwiseMin(v);
      hi = hi.cwiseMax(v);
    }
  }
  idx.bbox_min_ = lo;
  idx.bbox_max_ = hi;
  idx.origin_ = lo;
  idx.nx_ = static_cast<int>(std::floor((hi.x() - lo.x()) / idx.voxel_)) + 1;
  idx.ny_ = static_cast<int>(std::floor((hi.y() - lo.y()) / idx.voxel_)) + 1;
  idx.nz_ = static_cast<int>(std::floor((hi.z() - lo.z()) / idx.voxel_)) + 1;

  const std::size_t cells =
      static_cast<std::size_t>(idx.nx_) * idx.ny_ * idx.nz_;
  idx.left_.assign(cells, 0);
  idx.right_.assign(cells, 0);

  // ---- pass 2: fill the masks -----------------------------------------
  for (std::uint64_t i = 0; i < rec.size(); ++i) {
    const float* r = rec.rec(i);
    const Eigen::Vector3d pl = vec(r, kPLeft);
    const Eigen::Vector3d pr = vec(r, kPRight);
    auto mark = [&](std::vector<std::uint8_t>& m, const Eigen::Vector3d& p) {
      const int a = static_cast<int>(std::floor((p.x() - idx.origin_.x()) / idx.voxel_));
      const int b = static_cast<int>(std::floor((p.y() - idx.origin_.y()) / idx.voxel_));
      const int c = static_cast<int>(std::floor((p.z() - idx.origin_.z()) / idx.voxel_));
      if (a >= 0 && b >= 0 && c >= 0 && a < idx.nx_ && b < idx.ny_ && c < idx.nz_) {
        m[idx.index(a, b, c)] = 1;
      }
    };
    mark(idx.left_, pl);
    mark(idx.right_, pr);
  }
  for (std::size_t c = 0; c < cells; ++c) {
    if (idx.left_[c]) ++idx.n_left_;
    if (idx.right_[c]) ++idx.n_right_;
    if (idx.left_[c] && idx.right_[c]) ++idx.n_dual_;
  }

  // ---- pass 3: handover samples for P1 ---------------------------------
  std::uint64_t n_hand = 0;
  for (std::uint64_t i = 0; i < rec.size(); ++i) {
    const float* r = rec.rec(i);
    if (r[kDArm] <= options.handover_max_d) ++n_hand;
  }
  const std::uint64_t stride =
      (options.max_handover_samples == 0 || n_hand <= options.max_handover_samples)
          ? 1
          : (n_hand + options.max_handover_samples - 1) / options.max_handover_samples;

  std::uint64_t seen = 0;
  idx.handover_.reserve(static_cast<std::size_t>(
      std::min<std::uint64_t>(n_hand, options.max_handover_samples ? options.max_handover_samples
                                                                   : n_hand)));
  for (std::uint64_t i = 0; i < rec.size(); ++i) {
    const float* r = rec.rec(i);
    if (r[kDArm] > options.handover_max_d) continue;
    if ((seen++ % stride) != 0) continue;
    C41Sample s;
    for (int j = 0; j < 6; ++j) s.q_left[j] = r[j];
    s.p_left = vec(r, kPLeft);
    s.p_right = vec(r, kPRight);
    s.p_object = vec(r, kPObject);
    s.d_tcp = r[kDArm];
    idx.handover_.push_back(s);
  }
  return idx;
}

std::vector<Eigen::Vector3d> WorkspaceIndex::gridObjectCentres(double step) const {
  std::vector<Eigen::Vector3d> pts;
  if (step <= 0.0) return pts;
  const Eigen::Vector3d lo = bbox_min_;
  const Eigen::Vector3d hi = bbox_max_;
  const int nx = static_cast<int>(std::floor((hi.x() - lo.x()) / step)) + 1;
  const int ny = static_cast<int>(std::floor((hi.y() - lo.y()) / step)) + 1;
  const int nz = static_cast<int>(std::floor((hi.z() - lo.z()) / step)) + 1;
  pts.reserve(static_cast<std::size_t>(nx) * ny * nz);
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) {
        pts.emplace_back(lo.x() + i * step, lo.y() + j * step, lo.z() + k * step);
      }
    }
  }
  return pts;
}

}  // namespace dual_arm_pose_optimization
