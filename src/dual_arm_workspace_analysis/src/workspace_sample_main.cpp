// src/workspace_sample_main.cpp
// C4.1 CLI: sample the dual-arm configuration space and write the record files.
//
//   workspace_sample --samples 10000000 --out analysis/workspace_dual_arm
//                    [--threads N] [--d-min D] [--radius R] [--rank-tol T] [--seed S]

#include <cstdlib>
#include <iostream>
#include <string>

#include "dual_arm_workspace_analysis/workspace_sampler.hpp"

namespace {

void printUsage(const char* argv0) {
    std::cout
        << "usage: " << argv0 << " [options]\n"
        << "\n"
        << "  --samples N     number of configuration pairs to evaluate (default 10000000)\n"
        << "  --out PREFIX    output prefix; writes PREFIX.bin/.json/_preview.csv\n"
        << "                  (default " << dual_arm_workspace::defaultOutputPrefix() << ")\n"
        << "  --threads N     worker threads (default: hardware_concurrency)\n"
        << "  --d-min D       reject samples with arm-arm distance < D m (default 0.02)\n"
        << "  --radius R      capsule radius in m (default 0.06, same as C2.7)\n"
        << "  --rank-tol T    numerical-rank threshold sigma_i > T * sigma_1 (default 1e-6)\n"
        << "  --z-floor Z     ground/work-plane z in m (default 0.0); samples whose DH\n"
        << "                  frame origins or TCP dip below it are rejected\n"
        << "  --no-z-floor    disable the ground-plane filter entirely\n"
        << "  --seed S        Halton start offset (default 0)\n"
        << "  -h, --help      this message\n";
}

/// Parse the next argv entry as a value for `flag`.
std::string nextValue(int argc, char** argv, int* i, const char* flag) {
    if (*i + 1 >= argc) {
        std::cerr << "error: missing value for " << flag << "\n";
        std::exit(2);
    }
    return std::string(argv[++(*i)]);
}

}  // namespace

int main(int argc, char** argv) {
    dual_arm_workspace::SamplerParams params;
    std::string prefix = dual_arm_workspace::defaultOutputPrefix();

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--samples") {
            params.num_samples = std::stoull(nextValue(argc, argv, &i, "--samples"));
        } else if (a == "--threads") {
            params.num_threads = std::stoi(nextValue(argc, argv, &i, "--threads"));
        } else if (a == "--out") {
            prefix = nextValue(argc, argv, &i, "--out");
        } else if (a == "--d-min") {
            params.d_min = std::stod(nextValue(argc, argv, &i, "--d-min"));
        } else if (a == "--radius") {
            params.link_radius = std::stod(nextValue(argc, argv, &i, "--radius"));
        } else if (a == "--rank-tol") {
            params.rank_tol = std::stod(nextValue(argc, argv, &i, "--rank-tol"));
        } else if (a == "--z-floor") {
            params.z_floor = std::stod(nextValue(argc, argv, &i, "--z-floor"));
            params.use_z_floor = true;
        } else if (a == "--no-z-floor") {
            params.use_z_floor = false;
        } else if (a == "--seed") {
            params.seed_offset = std::stoull(nextValue(argc, argv, &i, "--seed"));
        } else if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "error: unknown argument '" << a << "'\n";
            printUsage(argv[0]);
            return 2;
        }
    }

    std::cout << "C4.1 workspace sampling\n"
              << "  samples     : " << params.num_samples << "\n"
              << "  threads     : "
              << (params.num_threads > 0 ? params.num_threads : 0) << " (0 = all cores)\n"
              << "  d_min       : " << params.d_min << " m\n"
              << "  capsule r   : " << params.link_radius << " m\n"
              << "  rank_tol    : " << params.rank_tol << "\n"
              << "  z floor     : "
              << (params.use_z_floor ? std::to_string(params.z_floor) + " m" : "disabled")
              << "\n"
              << "  out prefix  : " << prefix << "\n"
              << std::flush;

    const dual_arm_workspace::SamplerStats stats =
        dual_arm_workspace::runSampling(params, prefix);

    const double acc = stats.evaluated
                           ? static_cast<double>(stats.accepted) /
                                 static_cast<double>(stats.evaluated)
                           : 0.0;
    const double flo = stats.evaluated
                           ? static_cast<double>(stats.rejected_floor) /
                                 static_cast<double>(stats.evaluated)
                           : 0.0;
    const double def = stats.accepted
                           ? static_cast<double>(stats.rank_defect) /
                                 static_cast<double>(stats.accepted)
                           : 0.0;

    std::cout << "\ndone in " << stats.wall_seconds << " s\n"
              << "  evaluated          : " << stats.evaluated << "\n"
              << "  rejected (floor)   : " << stats.rejected_floor << "  ("
              << 100.0 * flo << " %)\n"
              << "  rejected (distance): " << stats.rejected_distance << "\n"
              << "  accepted           : " << stats.accepted << "  ("
              << 100.0 * acc << " %)\n"
              << "  rank_rel < 6       : " << stats.rank_defect << "  (" << 100.0 * def
              << " % of accepted)\n"
              << "  wrote              : " << prefix << ".bin / .json / _preview.csv\n";
    return 0;
}
