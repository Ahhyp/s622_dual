// C4.3 S4 W0: trajectory (moved from dual_arm_transport_mock, numerics unchanged).  See trajectory.hpp for the contract.
#include "dual_arm_object_task/trajectory.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <sstream>

#include "dual_arm_kinematics/object_frame.hpp"

namespace dual_arm_object_task {

namespace {

constexpr double kEpsDuration = 1e-12;

/// Path parameter and its time derivative: s(tau) and ds/dt = s'(tau)/duration.
struct Scaling {
    double s = 0.0;
    double sdot_per_s = 0.0;
};

Scaling evalScaling(const std::string& kind, double tau, double duration) {
    Scaling out;
    if (kind == "linear") {
        out.s = tau;
        out.sdot_per_s = 1.0 / duration;
    } else {  // "smoothstep": s = 3tau^2 - 2tau^3, s' = 6tau(1-tau)
        out.s = tau * tau * (3.0 - 2.0 * tau);
        out.sdot_per_s = 6.0 * tau * (1.0 - tau) / duration;
    }
    return out;
}

Eigen::Matrix3d rotZ(double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix3d R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

double yamlDouble(const YAML::Node& n, const std::string& key, double fallback, const std::string& ctx) {
    if (!n[key]) return fallback;
    if (!n[key].IsScalar()) {
        throw TrajectoryError(ctx + ": '" + key + "' must be a scalar");
    }
    try {
        return n[key].as<double>();
    } catch (const YAML::Exception& e) {
        throw TrajectoryError(ctx + ": '" + key + "' is not a number (" + e.what() + ")");
    }
}

}  // namespace

TrajectorySpec loadTrajectorySpec(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw TrajectoryError("cannot load trajectory '" + path + "': " + e.what());
    }
    if (!root.IsMap()) throw TrajectoryError("trajectory '" + path + "' must be a mapping");

    TrajectorySpec spec;
    spec.dt = yamlDouble(root, "dt", spec.dt, path);
    if (spec.dt <= 0.0) throw TrajectoryError(path + ": dt must be > 0");
    if (root["time_scaling"]) spec.time_scaling = root["time_scaling"].as<std::string>();
    if (spec.time_scaling != "smoothstep" && spec.time_scaling != "linear") {
        throw TrajectoryError(path + ": time_scaling must be 'smoothstep' or 'linear', got '" +
                              spec.time_scaling + "'");
    }

    if (root["start"]) {
        const std::string start = root["start"].as<std::string>();
        if (start == "from_baseline") {
            spec.start_from_baseline = true;
        } else if (start == "explicit") {
            spec.start_from_baseline = false;
        } else {
            throw TrajectoryError(path + ": start must be 'from_baseline' or 'explicit', got '" +
                                  start + "'");
        }
    }

    if (root["start_pose"]) {
        const YAML::Node sp = root["start_pose"];
        if (!sp["position"] || sp["position"].size() != 3) {
            throw TrajectoryError(path + ": start_pose.position must have 3 entries");
        }
        spec.start_world_T_object = dual_arm::Iso3::Identity();
        spec.start_world_T_object.translation() =
            Eigen::Vector3d(sp["position"][0].as<double>(), sp["position"][1].as<double>(),
                            sp["position"][2].as<double>());
        if (sp["rpy"]) {
            if (sp["rpy"].size() != 3) {
                throw TrajectoryError(path + ": start_pose.rpy must have 3 entries");
            }
            const double r = sp["rpy"][0].as<double>();
            const double p = sp["rpy"][1].as<double>();
            const double y = sp["rpy"][2].as<double>();
            const Eigen::AngleAxisd rz(y, Eigen::Vector3d::UnitZ());
            const Eigen::AngleAxisd ry(p, Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd rx(r, Eigen::Vector3d::UnitX());
            spec.start_world_T_object.linear() = (rz * ry * rx).toRotationMatrix();
        }
        spec.start_from_baseline = false;
    }

    if (!root["phases"] || !root["phases"].IsSequence() || root["phases"].size() == 0) {
        throw TrajectoryError(path + ": phases must be a non-empty sequence");
    }
    for (std::size_t i = 0; i < root["phases"].size(); ++i) {
        const YAML::Node pn = root["phases"][i];
        const std::string ctx = path + ": phases[" + std::to_string(i) + "]";
        if (!pn.IsMap()) throw TrajectoryError(ctx + " must be a mapping");
        TrajectoryPhase ph;
        if (!pn["name"]) throw TrajectoryError(ctx + ": name is required");
        ph.name = pn["name"].as<std::string>();
        if (!pn["duration"]) throw TrajectoryError(ctx + ": duration is required");
        ph.duration = yamlDouble(pn, "duration", 0.0, ctx);
        if (ph.duration <= 0.0) throw TrajectoryError(ctx + ": duration must be > 0");
        ph.dx = yamlDouble(pn, "dx", 0.0, ctx);
        ph.dy = yamlDouble(pn, "dy", 0.0, ctx);
        ph.dz = yamlDouble(pn, "dz", 0.0, ctx);
        spec.phases.push_back(ph);
    }

    if (root["yaw_sweep_deg"]) {
        if (!root["yaw_sweep_deg"].IsSequence()) {
            throw TrajectoryError(path + ": yaw_sweep_deg must be a sequence");
        }
        spec.yaw_sweep_deg.clear();
        for (std::size_t i = 0; i < root["yaw_sweep_deg"].size(); ++i) {
            spec.yaw_sweep_deg.push_back(root["yaw_sweep_deg"][i].as<double>());
        }
    }

    // Validate the yaw node count here so that the error names the file.
    FrozenObjectTrajectory check(spec);
    (void)check;
    return spec;
}

FrozenObjectTrajectory::FrozenObjectTrajectory(const TrajectorySpec& spec) : spec_(spec) {
    if (spec_.dt <= 0.0) throw TrajectoryError("dt must be > 0");
    if (spec_.phases.empty()) throw TrajectoryError("phases must be non-empty");
    if (spec_.yaw_sweep_deg.size() != 1 &&
        spec_.yaw_sweep_deg.size() != spec_.phases.size() + 1) {
        std::ostringstream os;
        os << "yaw_sweep_deg must have 1 entry (constant) or phases.size()+1 = "
           << spec_.phases.size() + 1 << " boundary nodes, got " << spec_.yaw_sweep_deg.size();
        throw TrajectoryError(os.str());
    }

    boundary_times_.reserve(spec_.phases.size() + 1);
    phase_start_p_.reserve(spec_.phases.size() + 1);
    boundary_times_.push_back(0.0);
    phase_start_p_.push_back(spec_.start_world_T_object.translation());
    double t = 0.0;
    for (const TrajectoryPhase& ph : spec_.phases) {
        t += ph.duration;
        boundary_times_.push_back(t);
        phase_start_p_.push_back(phase_start_p_.back() + Eigen::Vector3d(ph.dx, ph.dy, ph.dz));
    }
    duration_ = t;
    steps_ = static_cast<std::size_t>(std::ceil(duration_ / spec_.dt - kEpsDuration));
    if (steps_ == 0) steps_ = 1;
}

void FrozenObjectTrajectory::locate(double t, std::size_t* phase, double* tau) const {
    std::size_t k = spec_.phases.size() - 1;
    for (std::size_t i = 0; i < spec_.phases.size(); ++i) {
        if (t < boundary_times_[i + 1]) {
            k = i;
            break;
        }
    }
    const double dur = spec_.phases[k].duration;
    double local = (t - boundary_times_[k]) / dur;
    local = std::clamp(local, 0.0, 1.0);
    *phase = k;
    *tau = local;
}

dual_arm::Iso3 FrozenObjectTrajectory::at(double t, dual_arm::Vec6* body_twist_ff) const {
    dual_arm::Iso3 T_d = dual_arm::Iso3::Identity();
    dual_arm::Vec6 v = dual_arm::Vec6::Zero();

    // No special case for t <= 0 or t >= duration: locate() clamps tau to [0,1] and
    // sdot is 0 at both ends, so the pose is C1 at the ends and the twist vanishes
    // there.  A separate "hold" branch would only introduce a rotation discontinuity.
    std::size_t k = 0;
    double tau = 0.0;
    locate(t, &k, &tau);

    const TrajectoryPhase& ph = spec_.phases[k];
    const Scaling sc = evalScaling(spec_.time_scaling, tau, ph.duration);

    const Eigen::Vector3d p_start = phase_start_p_[k];
    const Eigen::Vector3d dp(ph.dx, ph.dy, ph.dz);
    const Eigen::Vector3d p_d = p_start + sc.s * dp;

    const double yaw0 = spec_.yaw_sweep_deg.size() == 1 ? spec_.yaw_sweep_deg[0]
                                                       : spec_.yaw_sweep_deg[k];
    const double yaw1 = spec_.yaw_sweep_deg.size() == 1 ? spec_.yaw_sweep_deg[0]
                                                       : spec_.yaw_sweep_deg[k + 1];
    const double yaw = (yaw0 + sc.s * (yaw1 - yaw0)) * M_PI / 180.0;
    const double yaw_dot = (yaw1 - yaw0) * M_PI / 180.0 * sc.sdot_per_s;

    T_d = dual_arm::Iso3::Identity();
    T_d.linear() = rotZ(yaw);
    T_d.translation() = p_d;

    if (body_twist_ff) {
        const Eigen::Vector3d p_dot = sc.sdot_per_s * dp;
        v.head<3>() = T_d.linear().transpose() * p_dot;
        // World angular velocity is (0,0,yaw_dot); rotating it into the body frame
        // about z leaves it unchanged.
        v.tail<3>() = Eigen::Vector3d(0.0, 0.0, yaw_dot);
        *body_twist_ff = v;
    }
    return T_d;
}

}  // namespace dual_arm_object_task
