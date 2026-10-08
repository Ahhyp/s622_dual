// src/dual_arm_kinematics.cpp

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

#include <algorithm>

namespace dual_arm {

Iso3 DualArmKinematics::toIso(const Eigen::Matrix4d& T) {
    Iso3 out = Iso3::Identity();
    out.matrix() = T;
    return out;
}

Iso3 DualArmKinematics::defaultWorldTleftBase() {
    Iso3 T = Iso3::Identity();
    T.translation() = Vec3(0.35, 0.0, 0.0);
    T.linear() = Eigen::AngleAxisd(M_PI, Vec3::UnitZ()).toRotationMatrix();
    return T;
}

Iso3 DualArmKinematics::defaultWorldTrightBase() {
    Iso3 T = Iso3::Identity();
    T.translation() = Vec3(-0.35, 0.0, 0.0);
    return T;
}

DualArmKinematics::DualArmKinematics()
    : DualArmKinematics(defaultWorldTleftBase(), defaultWorldTrightBase()) {}

DualArmKinematics::DualArmKinematics(const Iso3& world_T_left_base,
                                     const Iso3& world_T_right_base)
    : DualArmKinematics(world_T_left_base, world_T_right_base, DHParams{},
                        ToolParams::gripper()) {}

DualArmKinematics::DualArmKinematics(const Iso3& world_T_left_base,
                                     const Iso3& world_T_right_base,
                                     const DHParams& dh,
                                     const ToolParams& tool)
    : base_{world_T_left_base, world_T_right_base},
      dh_{DHKinematics(dh, tool), DHKinematics(dh, tool)} {}

Iso3 DualArmKinematics::baseTgrasp(Arm a, const JointConfig& q) const {
    return toIso(dh_[index(a)].fkine(q, ToolModel::GRIPPER));
}

Iso3 DualArmKinematics::worldTgrasp(Arm a, const JointConfig& q) const {
    return base_[index(a)] * baseTgrasp(a, q);
}

Jacobian6d DualArmKinematics::jacobianBase(Arm a, const JointConfig& q) const {
    return dh_[index(a)].jacobian(q, ToolModel::GRIPPER);
}

Mat6 DualArmKinematics::jacobianBody(Arm a, const JointConfig& q) const {
    const Mat3 R_gb = baseTgrasp(a, q).linear().transpose();  // grasp axes -> base axes
    Mat6 R6 = Mat6::Zero();
    R6.topLeftCorner<3, 3>() = R_gb;
    R6.bottomRightCorner<3, 3>() = R_gb;
    return R6 * jacobianBase(a, q);
}

Mat6 DualArmKinematics::spatialJacobianWorld(Arm a, const JointConfig& q) const {
    return adjoint(worldTgrasp(a, q)) * jacobianBody(a, q);
}

Mat6 DualArmKinematics::jacobianWorldPoint(Arm a, const JointConfig& q) const {
    const Mat3 R_wg = worldTgrasp(a, q).linear();
    Mat6 R6 = Mat6::Zero();
    R6.topLeftCorner<3, 3>() = R_wg;
    R6.bottomRightCorner<3, 3>() = R_wg;
    return R6 * jacobianBody(a, q);
}

Iso3 DualArmKinematics::baseTframe(Arm a, const JointConfig& q, int frame_index) const {
    const auto Ts = dh_[index(a)].fkineAll(q);
    const int i = std::min(6, std::max(0, frame_index));
    Iso3 T = Iso3::Identity();
    T.matrix() = Ts[i];
    return T;
}

Iso3 DualArmKinematics::worldTframe(Arm a, const JointConfig& q, int frame_index) const {
    return base_[index(a)] * baseTframe(a, q, frame_index);
}

Iso3 DualArmKinematics::worldTobject(const Q12& q12) const {
    JointConfig ql, qr;
    split(q12, &ql, &qr);
    const Iso3 W_T_L = worldTgrasp(Arm::Left, ql);
    const Iso3 W_T_R = worldTgrasp(Arm::Right, qr);

    Iso3 W_T_O = Iso3::Identity();
    W_T_O.translation() = 0.5 * (W_T_L.translation() + W_T_R.translation());
    const Mat3 A = W_T_L.linear().transpose() * W_T_R.linear();
    W_T_O.linear() = W_T_L.linear() * expSO3(0.5 * logSO3(A));
    return W_T_O;
}

Mat6x12 DualArmKinematics::objectJacobianWorld(const Q12& q12) const {
    JointConfig ql, qr;
    split(q12, &ql, &qr);

    const Mat6 J_L = jacobianWorldPoint(Arm::Left, ql);
    const Mat6 J_R = jacobianWorldPoint(Arm::Right, qr);

    const Mat3 R_L = worldTgrasp(Arm::Left, ql).linear();
    const Mat3 R_R = worldTgrasp(Arm::Right, qr).linear();
    const Vec3 xi = logSO3(R_L.transpose() * R_R);

    // w_O = w_L + G (w_R - w_L)
    const Mat3 G = R_L * leftJacobianSO3(0.5 * xi) * 0.5 * leftJacobianInverseSO3(xi) *
                   R_L.transpose();

    Mat6x12 J = Mat6x12::Zero();
    J.topRows<3>().leftCols<6>() = 0.5 * J_L.topRows<3>();
    J.topRows<3>().rightCols<6>() = 0.5 * J_R.topRows<3>();
    J.bottomRows<3>().leftCols<6>() = (Mat3::Identity() - G) * J_L.bottomRows<3>();
    J.bottomRows<3>().rightCols<6>() = G * J_R.bottomRows<3>();
    return J;
}

Mat6x12 DualArmKinematics::objectJacobianBody(const Q12& q12) const {
    const Mat3 R_wo_T = worldTobject(q12).linear().transpose();
    Mat6x12 J = objectJacobianWorld(q12);  // axes world, ref object origin
    J.topRows<3>() = R_wo_T * J.topRows<3>();
    J.bottomRows<3>() = R_wo_T * J.bottomRows<3>();
    return J;
}

Vec6 DualArmKinematics::objectError(const Q12& q12, const Iso3& target) const {
    const Iso3 T_err = worldTobject(q12).inverse() * target;
    return logSE3(T_err);
}

Mat6x12 DualArmKinematics::objectErrorJacobian(const Q12& q12, const Iso3& target) const {
    const Iso3 T_err = worldTobject(q12).inverse() * target;
    const Mat6x12 J_obj = objectJacobianBody(q12);
    // small-error approximation, same convention as relativeErrorJacobian()
    return -adjoint(T_err.inverse()) * J_obj;
}

Q12 DualArmKinematics::stack(const JointConfig& q_left, const JointConfig& q_right) {
    Q12 out;
    out.head<6>() = q_left;
    out.tail<6>() = q_right;
    return out;
}

void DualArmKinematics::split(const Q12& q12, JointConfig* q_left, JointConfig* q_right) {
    if (q_left != nullptr) *q_left = q12.head<6>();
    if (q_right != nullptr) *q_right = q12.tail<6>();
}

Iso3 DualArmKinematics::leftTright(const Q12& q12) const {
    JointConfig ql, qr;
    split(q12, &ql, &qr);
    return leftTright(ql, qr);
}

Iso3 DualArmKinematics::leftTright(const JointConfig& q_left,
                                   const JointConfig& q_right) const {
    return worldTgrasp(Arm::Left, q_left).inverse() *
           worldTgrasp(Arm::Right, q_right);
}

Mat6x12 DualArmKinematics::relativeJacobian(const Q12& q12, RelativeFrame frame) const {
    JointConfig ql, qr;
    split(q12, &ql, &qr);

    const Mat6 JbL = jacobianBody(Arm::Left, ql);
    const Mat6 JbR = jacobianBody(Arm::Right, qr);
    const Iso3 T_rel = leftTright(ql, qr);

    Mat6x12 J;
    switch (frame) {
        case RelativeFrame::BodyR: {
            // xi_rel^R = V_R^R - Ad_{R_T_L} V_L^L
            const Iso3 T_rel_inv = T_rel.inverse();
            J.leftCols<6>() = -adjoint(T_rel_inv) * JbL;
            J.rightCols<6>() = JbR;
            break;
        }
        case RelativeFrame::LeftL: {
            // xi_rel^L = Ad_{L_T_R} V_R^R - V_L^L
            J.leftCols<6>() = -JbL;
            J.rightCols<6>() = adjoint(T_rel) * JbR;
            break;
        }
        case RelativeFrame::WorldSpatial: {
            // xi_rel^W = Xi_R - Xi_L  (spatial twists, world axes)
            J.leftCols<6>() = -spatialJacobianWorld(Arm::Left, ql);
            J.rightCols<6>() = spatialJacobianWorld(Arm::Right, qr);
            break;
        }
    }
    return J;
}

Vec6 DualArmKinematics::relativeError(const Q12& q12,
                                      const Iso3& left_T_right_target) const {
    const Iso3 T_err = leftTright(q12).inverse() * left_T_right_target;
    return logSE3(T_err);
}

Mat6x12 DualArmKinematics::relativeErrorJacobian(
    const Q12& q12, const Iso3& left_T_right_target) const {
    const Iso3 T_rel = leftTright(q12);
    const Iso3 T_err = T_rel.inverse() * left_T_right_target;
    const Mat6x12 J_rel = relativeJacobian(q12, RelativeFrame::BodyR);
    // edot = -J_r^{SE3}(e)^{-1} * Ad_{T_err^{-1}} * xi_rel^R ; small-error
    // approximation J_r ~= I (validated numerically in test_step0_relative).
    return -adjoint(T_err.inverse()) * J_rel;
}

}  // namespace dual_arm
