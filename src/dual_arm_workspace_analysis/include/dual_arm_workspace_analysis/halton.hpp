// include/dual_arm_workspace_analysis/halton.hpp
// C4.1: Halton low-discrepancy sequence (self-contained, no new dependency).
//
// Why not uniform random: the S622 joint ranges differ a lot in width
// (j2/j4 span 6.109 rad, j1/j5/j6 span 6.109 rad, j3 spans 5.655 rad -- but the
// *shape* of the box is elongated), and plain Monte Carlo leaves holes at
// 1e7 samples in 12-D.  Halton covers the box far more evenly for the same
// budget while staying deterministic and trivially parallelisable (each worker
// takes an independent index segment).

#pragma once

#include <cstdint>
#include <vector>

namespace dual_arm_workspace {

/// Van der Corput radical inverse of `i` in `base`.
inline double radicalInverse(std::uint64_t i, std::uint64_t base) {
    double f = 1.0;
    double r = 0.0;
    while (i > 0) {
        f /= static_cast<double>(base);
        r += f * static_cast<double>(i % base);
        i /= base;
    }
    return r;
}

/// 1-based prime lookup: nthPrime(1) == 2, nthPrime(2) == 3, ...
inline std::uint64_t nthPrime(int n) {
    if (n < 1) return 2;
    int count = 0;
    std::uint64_t candidate = 1;
    while (count < n) {
        ++candidate;
        bool prime = true;
        for (std::uint64_t d = 2; d * d <= candidate; ++d) {
            if (candidate % d == 0) {
                prime = false;
                break;
            }
        }
        if (prime) ++count;
    }
    return candidate;
}

/// Halton sequence in `dim` dimensions.  Deterministic; `start` selects the
/// (start+1)-th point onwards, which gives parallel workers disjoint segments.
class Halton {
public:
    explicit Halton(int dim, std::uint64_t start = 0)
        : base_(static_cast<std::size_t>(dim)), index_(start) {
        for (int d = 0; d < dim; ++d) {
            base_[static_cast<std::size_t>(d)] = nthPrime(d + 1);
        }
    }

    /// Fill `out[0..dim)` with the next point, each component in [0, 1).
    void next(double* out) {
        ++index_;
        for (std::size_t d = 0; d < base_.size(); ++d) {
            out[d] = radicalInverse(index_, base_[d]);
        }
    }

    /// Number of points consumed so far (including the initial `start` offset).
    std::uint64_t index() const { return index_; }

    int dim() const { return static_cast<int>(base_.size()); }

private:
    std::vector<std::uint64_t> base_;
    std::uint64_t index_;
};

}  // namespace dual_arm_workspace
