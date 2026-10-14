// src/optimizer.cpp
#include "dual_arm_pose_optimization/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_pose_optimization {

namespace {

double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) { return dual_arm::expSO3(w); }

}  // namespace

ObjectiveSpec ObjectiveSpec::named(const std::string& set) {
  ObjectiveSpec s;
  if (set == "equal") {
    for (double& w : s.weight) w = 1.0;
  } else if (set == "precision_first") {
    s.weight[0] = 3.0; s.weight[1] = 3.0; s.weight[2] = 3.0;
    s.weight[3] = 0.5; s.weight[4] = 1.0; s.weight[5] = 1.0;
    s.weight[6] = 1.0; s.weight[7] = 0.5;
  } else if (set == "safety_first") {
    s.weight[0] = 1.0; s.weight[1] = 1.0; s.weight[2] = 1.0;
    s.weight[3] = 4.0; s.weight[4] = 3.0; s.weight[5] = 0.5;
    s.weight[6] = 0.5; s.weight[7] = 4.0;
  }
  return s;
}

Eigen::Matrix<double, 8, 1> normalisedObjectives(const CandidateResult& r,
                                                 const ObjectiveSpec& spec) {
  Eigen::Matrix<double, 8, 1> j;
  j[0] = clamp01(r.smin_left / spec.scale[0]);
  j[1] = clamp01(r.smin_right / spec.scale[1]);
  j[2] = clamp01(r.w_rel / spec.scale[2]);
  j[3] = clamp01(r.mesh_arm_arm / spec.scale[3]);
  j[4] = clamp01(r.joint_margin / spec.scale[4]);
  j[5] = clamp01(1.0 - r.deviation / spec.scale[5]);
  j[6] = clamp01(r.continuity / spec.scale[6]);
  j[7] = clamp01(r.arm_proximal_clearance / spec.scale[7]);
  for (int i = 0; i < 8; ++i) {
    if (!std::isfinite(j[i])) j[i] = 0.0;
  }
  return j;
}

double scoreResult(const CandidateResult& r, const ObjectiveSpec& spec) {
  const Eigen::Matrix<double, 8, 1> j = normalisedObjectives(r, spec);
  double num = 0.0, den = 0.0;
  for (int i = 0; i < 8; ++i) {
    num += spec.weight[i] * j[i];
    den += spec.weight[i];
  }
  const double base = den > 0.0 ? num / den : 0.0;

  if (r.layer.s4_collision_free) return base;

  // Layered penalty: strictly ordered so the simplex is pulled towards deeper
  // feasibility layers before it starts trading off objectives.
  double pen = -0.05;
  if (!r.layer.s3_grasp) pen -= 0.10;
  if (!r.layer.s2_dual) pen -= 0.20;
  if (!r.layer.s0_point_reachable) pen -= 0.50;
  return pen;
}

PoseRefiner::PoseRefiner(FeasibilityPipeline* pipeline, const WorkspaceIndex* index,
                         const ObjectiveSpec& spec, const RefineOptions& options)
    : pipeline_(pipeline), index_(index), spec_(spec), opts_(options) {
  if (pipeline_ == nullptr) throw SceneLoadError("PoseRefiner: null pipeline");
}

void PoseRefiner::project(PoseParams* p) const {
  if (!opts_.allow_orientation) {
    p->rotvec.setZero();
    return;
  }
  const double n = p->rotvec.norm();
  if (n > opts_.theta_max && n > 1e-12) {
    p->rotvec *= opts_.theta_max / n;  // project onto the ||dtheta|| ball
  }
}

double PoseRefiner::evaluate(const PoseParams& p, const Eigen::Matrix3d& R_O_base,
                             CandidateResult* out) {
  Candidate c;
  c.p_O = p.p_O;
  c.R_O = R_O_base * expSO3(p.rotvec);
  c.pool = "refine";
  CandidateResult r = pipeline_->evaluate(c, index_);
  ++total_evals_;
  const double s = scoreResult(r, spec_);
  if (out != nullptr) *out = r;
  return s;
}

RefineResult PoseRefiner::refine(const PoseParams& start, const Eigen::Matrix3d& R_O_base) {
  const int n = opts_.allow_orientation ? 6 : 3;

  auto toVec = [&](const PoseParams& p) {
    Eigen::VectorXd x(n);
    x.head<3>() = p.p_O;
    if (n == 6) x.tail<3>() = p.rotvec;
    return x;
  };
  auto toParams = [&](const Eigen::VectorXd& x) {
    PoseParams p;
    p.p_O = x.head<3>();
    if (n == 6) p.rotvec = x.tail<3>();
    project(&p);
    return p;
  };
  auto f = [&](const Eigen::VectorXd& x) { return -evaluate(toParams(x), R_O_base, nullptr); };

  // ---- Nelder-Mead (standard coefficients) ------------------------------
  std::vector<Eigen::VectorXd> simplex(n + 1);
  std::vector<double> fv(n + 1);
  simplex[0] = toVec(start);
  for (int i = 0; i < n; ++i) {
    simplex[i + 1] = simplex[0];
    simplex[i + 1][i] += (i < 3) ? 0.01 : (2.0 * M_PI / 180.0);
  }
  for (int i = 0; i <= n; ++i) fv[i] = f(simplex[i]);

  const double alpha = 1.0, gamma = 2.0, rho = 0.5, sigma = 0.5;
  int evals = n + 1;
  while (evals < opts_.max_evals) {
    std::vector<int> idx(n + 1);
    for (int i = 0; i <= n; ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return fv[a] < fv[b]; });
    std::vector<Eigen::VectorXd> s2(n + 1);
    std::vector<double> f2(n + 1);
    for (int i = 0; i <= n; ++i) { s2[i] = simplex[idx[i]]; f2[i] = fv[idx[i]]; }
    simplex.swap(s2);
    fv.swap(f2);

    // convergence: simplex size and function spread
    double maxdx = 0.0, maxdf = 0.0;
    for (int i = 1; i <= n; ++i) {
      maxdx = std::max(maxdx, (simplex[i] - simplex[0]).cwiseAbs().maxCoeff());
      maxdf = std::max(maxdf, std::abs(fv[i] - fv[0]));
    }
    if (maxdx < opts_.tol_x && maxdf < opts_.tol_f) break;

    Eigen::VectorXd centroid = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) centroid += simplex[i];
    centroid /= static_cast<double>(n);

    const Eigen::VectorXd xr = centroid + alpha * (centroid - simplex[n]);
    const double fr = f(xr);
    ++evals;

    if (fr < fv[0]) {
      const Eigen::VectorXd xe = centroid + gamma * (xr - centroid);
      const double fe = f(xe);
      ++evals;
      if (fe < fr) { simplex[n] = xe; fv[n] = fe; } else { simplex[n] = xr; fv[n] = fr; }
    } else if (fr < fv[n - 1]) {
      simplex[n] = xr;
      fv[n] = fr;
    } else {
      const Eigen::VectorXd xc = centroid + rho * (simplex[n] - centroid);
      const double fc = f(xc);
      ++evals;
      if (fc < fv[n]) {
        simplex[n] = xc;
        fv[n] = fc;
      } else {
        for (int i = 1; i <= n; ++i) {
          simplex[i] = simplex[0] + sigma * (simplex[i] - simplex[0]);
          fv[i] = f(simplex[i]);
          ++evals;
        }
      }
    }
  }

  // best vertex, fully evaluated so the caller gets the metrics too
  std::size_t best = 0;
  for (std::size_t i = 1; i < fv.size(); ++i) {
    if (fv[i] < fv[best]) best = i;
  }
  RefineResult out;
  out.best = toParams(simplex[best]);
  out.score = evaluate(out.best, R_O_base, &out.result);
  ++total_evals_;
  out.feasible = out.result.layer.s4_collision_free;
  out.evals = evals + 1;
  return out;
}

}  // namespace dual_arm_pose_optimization
