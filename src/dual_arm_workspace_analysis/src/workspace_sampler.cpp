// src/workspace_sampler.cpp
// C4.1 implementation: Halton sampling + filtering + metrics + binary output.

#include "dual_arm_workspace_analysis/workspace_sampler.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

#include "dual_arm_workspace_analysis/halton.hpp"

namespace dual_arm_workspace {

namespace {

constexpr std::size_t kWriteChunkRecords = 65536;  // ~9.7 MB float32 per flush

std::size_t flushCount(std::uint64_t num_samples, int num_threads, int t) {
    const std::uint64_t chunk =
        (num_samples + static_cast<std::uint64_t>(num_threads) - 1) /
        static_cast<std::uint64_t>(num_threads);
    const std::uint64_t begin = static_cast<std::uint64_t>(t) * chunk;
    if (begin >= num_samples) return 0;
    return static_cast<std::size_t>(std::min<std::uint64_t>(chunk, num_samples - begin));
}

}  // namespace

const char* const kRecordFields =
    "q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,"
    "p_left_x,p_left_y,p_left_z,"
    "p_right_x,p_right_y,p_right_z,"
    "p_object_x,p_object_y,p_object_z,"
    "p_rel_x,p_rel_y,p_rel_z,"
    "rel_rotvec_x,rel_rotvec_y,rel_rotvec_z,"
    "d_arms,w_left,w_right,smin_left,smin_right,"
    "w_rel_body,smin_rel_body,w_rel_left,w_rel_world,rank_rel";

void encodeRecord(const SampleRecord& r, float* out) {
    int k = 0;
    for (int i = 0; i < 12; ++i) out[k++] = static_cast<float>(r.q(i));
    for (int i = 0; i < 3; ++i) out[k++] = static_cast<float>(r.p_left(i));
    for (int i = 0; i < 3; ++i) out[k++] = static_cast<float>(r.p_right(i));
    for (int i = 0; i < 3; ++i) out[k++] = static_cast<float>(r.p_object(i));
    for (int i = 0; i < 3; ++i) out[k++] = static_cast<float>(r.p_rel(i));
    for (int i = 0; i < 3; ++i) out[k++] = static_cast<float>(r.rel_rotvec(i));
    out[k++] = static_cast<float>(r.d_arms);
    out[k++] = static_cast<float>(r.w_left);
    out[k++] = static_cast<float>(r.w_right);
    out[k++] = static_cast<float>(r.smin_left);
    out[k++] = static_cast<float>(r.smin_right);
    out[k++] = static_cast<float>(r.w_rel_body);
    out[k++] = static_cast<float>(r.smin_rel_body);
    out[k++] = static_cast<float>(r.w_rel_left);
    out[k++] = static_cast<float>(r.w_rel_world);
    out[k++] = static_cast<float>(r.rank_rel);
}

SvdMetrics svdMetrics(const Eigen::MatrixXd& J, double rank_tol) {
    SvdMetrics m;
    if (J.size() == 0) return m;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const Eigen::VectorXd& s = svd.singularValues();
    m.num_sv = static_cast<int>(s.size());
    if (s.size() == 0) return m;

    m.smin = s.minCoeff();

    // prod(sigma_i) via logs: sqrt(det(J J^T)) underflows for near-singular J.
    double logsum = 0.0;
    for (int i = 0; i < s.size(); ++i) {
        logsum += std::log(std::max(s(i), 1e-300));
    }
    m.w = std::exp(logsum);

    const double thresh = rank_tol * s(0);
    int rank = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > thresh) ++rank;
    }
    m.rank = rank;
    return m;
}

WorkspaceSampler::WorkspaceSampler(const SamplerParams& params)
    : params_(params),
      kin_(),
      collision_(kin_, params_.link_radius) {}

bool WorkspaceSampler::aboveFloor(const Q12& q) const {
    if (!params_.use_z_floor) return true;
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    if (kin_.worldTgrasp(Arm::Left, ql).translation().z() < params_.z_floor) return false;
    if (kin_.worldTgrasp(Arm::Right, qr).translation().z() < params_.z_floor) return false;
    for (int a = 0; a < 2; ++a) {
        const Arm arm = (a == 0) ? Arm::Left : Arm::Right;
        const JointConfig& qa = (a == 0) ? ql : qr;
        const std::array<Eigen::Matrix4d, 7> frames = kin_.dh(arm).fkineAll(qa);
        const Eigen::Matrix4d wb = kin_.worldTbase(arm).matrix();
        for (int i = 0; i < 7; ++i) {
            const Eigen::Matrix4d wTf = wb * frames[static_cast<std::size_t>(i)];
            if (wTf(2, 3) < params_.z_floor) return false;  // plain 4x4, no .translation()
        }
    }
    return true;
}

bool WorkspaceSampler::evaluate(const Q12& q, SampleRecord* out) const {
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);

    const dual_arm::Iso3 wTl = kin_.worldTgrasp(Arm::Left, ql);
    const dual_arm::Iso3 wTr = kin_.worldTgrasp(Arm::Right, qr);

    // Ground / work-plane filter (see SamplerParams::use_z_floor).
    if (!aboveFloor(q)) return false;

    const auto dist = collision_.distance(q);
    // `!(x >= d)` also rejects NaN distances.
    if (!(dist.distance >= params_.d_min)) return false;

    SampleRecord r;
    r.q = q;
    r.d_arms = dist.distance;
    r.p_left = wTl.translation();
    r.p_right = wTr.translation();
    r.p_object = kin_.worldTobject(q).translation();

    const dual_arm::Iso3 lTr = kin_.leftTright(q);
    r.p_rel = lTr.translation();
    r.rel_rotvec = dual_arm::logSO3(lTr.rotation());

    const SvdMetrics mL = svdMetrics(kin_.jacobianBase(Arm::Left, ql), params_.rank_tol);
    const SvdMetrics mR = svdMetrics(kin_.jacobianBase(Arm::Right, qr), params_.rank_tol);
    r.w_left = mL.w;
    r.smin_left = mL.smin;
    r.w_right = mR.w;
    r.smin_right = mR.smin;

    const SvdMetrics mBody =
        svdMetrics(kin_.relativeJacobian(q, RelativeFrame::BodyR), params_.rank_tol);
    r.w_rel_body = mBody.w;
    r.smin_rel_body = mBody.smin;
    r.rank_rel = mBody.rank;  // frame-invariant (see header)

    r.w_rel_left = svdMetrics(kin_.relativeJacobian(q, RelativeFrame::LeftL),
                              params_.rank_tol).w;
    r.w_rel_world = svdMetrics(kin_.relativeJacobian(q, RelativeFrame::WorldSpatial),
                               params_.rank_tol).w;

    *out = r;
    return true;
}

std::string defaultOutputPrefix() {
    return "workspace_dual_arm";
}

SamplerStats runSampling(const SamplerParams& params, const std::string& out_prefix) {
    SamplerStats stats;
    stats.requested = params.num_samples;

    int threads = params.num_threads;
    if (threads <= 0) {
        threads = static_cast<int>(std::thread::hardware_concurrency());
    }
    if (threads <= 0) threads = 1;
    if (static_cast<std::uint64_t>(threads) > params.num_samples) {
        threads = static_cast<int>(std::max<std::uint64_t>(1, params.num_samples));
    }

    const auto t0 = std::chrono::steady_clock::now();

    std::vector<std::uint64_t> evaluated(static_cast<std::size_t>(threads), 0);
    std::vector<std::uint64_t> rejected_floor(static_cast<std::size_t>(threads), 0);
    std::vector<std::uint64_t> rejected(static_cast<std::size_t>(threads), 0);
    std::vector<std::uint64_t> accepted(static_cast<std::size_t>(threads), 0);
    std::vector<std::uint64_t> defect(static_cast<std::size_t>(threads), 0);

    // One part file per worker: keeps peak RAM at a few MB per thread instead of
    // holding all accepted records in memory (this box has 7 GB total).
    std::vector<std::string> part_paths(static_cast<std::size_t>(threads));
    for (int t = 0; t < threads; ++t) {
        part_paths[static_cast<std::size_t>(t)] =
            out_prefix + ".part" + std::to_string(t) + ".bin";
    }

    const std::uint64_t chunk =
        (params.num_samples + static_cast<std::uint64_t>(threads) - 1) /
        static_cast<std::uint64_t>(threads);

    auto worker = [&](int t) {
        const std::size_t count = flushCount(params.num_samples, threads, t);
        const std::size_t idx = static_cast<std::size_t>(t);
        if (count == 0) return;

        WorkspaceSampler sampler(params);  // per-thread: no shared mutable state
        Halton halton(12, params.seed_offset + static_cast<std::uint64_t>(t) * chunk);

        std::ofstream out(part_paths[idx], std::ios::binary);
        std::vector<float> buffer;
        buffer.reserve(kWriteChunkRecords * kRecordFloats);

        double u[12];
        for (std::size_t i = 0; i < count; ++i) {
            halton.next(u);
            Q12 q;
            for (int j = 0; j < 6; ++j) {
                const double lo = params.limits.lower[static_cast<std::size_t>(j)];
                const double hi = params.limits.upper[static_cast<std::size_t>(j)];
                q(j) = lo + u[j] * (hi - lo);        // left arm
                q(j + 6) = lo + u[j + 6] * (hi - lo);  // right arm
            }
            ++evaluated[idx];

            SampleRecord rec;
            if (!sampler.evaluate(q, &rec)) {
                // Attribute the rejection: cheap re-check of just the floor test.
                if (params.use_z_floor && !sampler.aboveFloor(q)) {
                    ++rejected_floor[idx];
                } else {
                    ++rejected[idx];
                }
                continue;
            }
            ++accepted[idx];
            if (rec.rank_rel < 6.0) ++defect[idx];

            const std::size_t base = buffer.size();
            buffer.resize(base + kRecordFloats);
            encodeRecord(rec, buffer.data() + base);

            if (buffer.size() >= kWriteChunkRecords * kRecordFloats) {
                out.write(reinterpret_cast<const char*>(buffer.data()),
                          static_cast<std::streamsize>(buffer.size() * sizeof(float)));
                buffer.clear();
            }
        }
        if (!buffer.empty()) {
            out.write(reinterpret_cast<const char*>(buffer.data()),
                      static_cast<std::streamsize>(buffer.size() * sizeof(float)));
        }
        out.flush();
        out.close();
    };

    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(threads));
    for (int t = 1; t < threads; ++t) pool.emplace_back(worker, t);
    worker(0);
    for (auto& th : pool) th.join();

    for (int t = 0; t < threads; ++t) {
        const std::size_t idx = static_cast<std::size_t>(t);
        stats.evaluated += evaluated[idx];
        stats.rejected_floor += rejected_floor[idx];
        stats.rejected_distance += rejected[idx];
        stats.accepted += accepted[idx];
        stats.rank_defect += defect[idx];
    }

    // Merge part files (streaming, 1 MB at a time).
    const std::string bin_path = out_prefix + ".bin";
    {
        std::ofstream merged(bin_path, std::ios::binary);
        std::vector<char> copy_buffer(1 << 20);
        for (int t = 0; t < threads; ++t) {
            std::ifstream in(part_paths[static_cast<std::size_t>(t)], std::ios::binary);
            while (in) {
                in.read(copy_buffer.data(), static_cast<std::streamsize>(copy_buffer.size()));
                const std::streamsize got = in.gcount();
                if (got > 0) merged.write(copy_buffer.data(), got);
            }
            in.close();
            std::error_code ec;
            std::filesystem::remove(part_paths[static_cast<std::size_t>(t)], ec);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    stats.wall_seconds = std::chrono::duration<double>(t1 - t0).count();

    // CSV preview (first 1000 accepted records).
    {
        std::ifstream in(bin_path, std::ios::binary);
        std::ofstream csv(out_prefix + "_preview.csv");
        csv << kRecordFields << "\n";
        std::vector<float> row(kRecordFloats);
        for (int i = 0; i < 1000; ++i) {
            in.read(reinterpret_cast<char*>(row.data()),
                    static_cast<std::streamsize>(kRecordFloats * sizeof(float)));
            if (in.gcount() != static_cast<std::streamsize>(kRecordFloats * sizeof(float))) {
                break;
            }
            for (int k = 0; k < kRecordFloats; ++k) {
                csv << (k == 0 ? "" : ",") << row[static_cast<std::size_t>(k)];
            }
            csv << "\n";
        }
    }

    // Run manifest.
    {
        std::ofstream js(out_prefix + ".json");
        js << "{\n";
        js << "  \"schema\": {\"dtype\": \"float32\", \"fields\": " << kRecordFloats
           << ", \"order\": \"" << kRecordFields << "\"},\n";
        js << "  \"params\": {\n";
        js << "    \"num_samples\": " << params.num_samples << ",\n";
        js << "    \"num_threads\": " << threads << ",\n";
        js << "    \"link_radius\": " << params.link_radius << ",\n";
        js << "    \"d_min\": " << params.d_min << ",\n";
        js << "    \"use_z_floor\": " << (params.use_z_floor ? "true" : "false") << ",\n";
        js << "    \"z_floor\": " << params.z_floor << ",\n";
        js << "    \"rank_tol\": " << params.rank_tol << ",\n";
        js << "    \"seed_offset\": " << params.seed_offset << ",\n";
        js << "    \"joint_lower\": [";
        for (int i = 0; i < 6; ++i) {
            js << (i ? ", " : "") << params.limits.lower[static_cast<std::size_t>(i)];
        }
        js << "],\n    \"joint_upper\": [";
        for (int i = 0; i < 6; ++i) {
            js << (i ? ", " : "") << params.limits.upper[static_cast<std::size_t>(i)];
        }
        js << "]\n  },\n";
        js << "  \"stats\": {\n";
        js << "    \"requested\": " << stats.requested << ",\n";
        js << "    \"evaluated\": " << stats.evaluated << ",\n";
        js << "    \"rejected_floor\": " << stats.rejected_floor << ",\n";
        js << "    \"rejected_distance\": " << stats.rejected_distance << ",\n";
        js << "    \"accepted\": " << stats.accepted << ",\n";
        js << "    \"rank_defect\": " << stats.rank_defect << ",\n";
        js << "    \"acceptance_rate\": "
           << (stats.evaluated ? static_cast<double>(stats.accepted) /
                                     static_cast<double>(stats.evaluated)
                               : 0.0)
           << ",\n";
        js << "    \"rank_defect_rate\": "
           << (stats.accepted ? static_cast<double>(stats.rank_defect) /
                                    static_cast<double>(stats.accepted)
                              : 0.0)
           << ",\n";
        js << "    \"wall_seconds\": " << stats.wall_seconds << "\n";
        js << "  }\n}\n";
    }

    return stats;
}

}  // namespace dual_arm_workspace
