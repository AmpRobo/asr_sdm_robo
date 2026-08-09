#include "asr_sdm_control_manager/asr_sdm_kinematic_model.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <string>

namespace asr
{

namespace
{

constexpr double kHalfPi = 1.5707963267948966;

pinocchio::SE3 translationX(double x)
{
  return pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, 0.0, 0.0));
}

std::string jointName(std::size_t joint, const char * suffix)
{
  return "joint" + std::to_string(joint) + "_" + std::string(suffix);
}

}  // namespace

AsrSdmKinematicModel::AsrSdmKinematicModel(const AsrSdmKinematicModelParameters & params)
: params_(params)
{
  build();
  data_ = Data(model_);
}

void AsrSdmKinematicModel::build()
{
  using pinocchio::JointIndex;
  using pinocchio::SE3;

  model_.name = "asr_sdm_kinematic_model";

  const double link_length = params_.link_length;
  // Negated axes reproduce the Rz(-theta_yaw) * Ry(-theta_pitch) convention, so a
  // Pinocchio joint coordinate is the very same number as the physical angle.
  const Eigen::Vector3d yaw_axis(0.0, 0.0, -1.0);
  const Eigen::Vector3d pitch_axis(0.0, -1.0, 0.0);

  // A link spans from its own frame origin towards -x, hence the cylinder (built
  // along z) is rotated onto x and shifted back by half a link.
  const SE3 link_body_placement(
    Eigen::Matrix3d(Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitY())),
    Eigen::Vector3d(-0.5 * link_length, 0.0, 0.0));
  const pinocchio::Inertia link_inertia =
    pinocchio::Inertia::FromCylinder(params_.link_mass, params_.link_radius, link_length);

  const Eigen::VectorXd joint_effort = Eigen::VectorXd::Constant(1, params_.joint_effort_limit);
  const Eigen::VectorXd joint_velocity = Eigen::VectorXd::Constant(1, params_.joint_velocity_limit);
  const Eigen::VectorXd joint_lower = Eigen::VectorXd::Constant(1, -params_.joint_limit);
  const Eigen::VectorXd joint_upper = Eigen::VectorXd::Constant(1, params_.joint_limit);

  head_joint_id_ =
    model_.addJoint(0, pinocchio::JointModelFreeFlyer(), SE3::Identity(), "head_joint");
  model_.addJointFrame(head_joint_id_);
  model_.appendBodyToJoint(head_joint_id_, link_inertia, link_body_placement);
  model_.addBodyFrame("link_0", head_joint_id_, SE3::Identity());
  body_point_frame_ids_[0] = model_.addFrame(
    pinocchio::Frame("body_point_0", head_joint_id_, SE3::Identity(), pinocchio::OP_FRAME));

  for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
    const JointIndex parent = joint == 0 ? head_joint_id_ : pitch_joint_ids_[joint - 1];

    yaw_joint_ids_[joint] = model_.addJoint(
      parent, pinocchio::JointModelRevoluteUnaligned(yaw_axis), translationX(-link_length),
      jointName(joint, "yaw"), joint_effort, joint_velocity, joint_lower, joint_upper);
    model_.addJointFrame(yaw_joint_ids_[joint]);

    pitch_joint_ids_[joint] = model_.addJoint(
      yaw_joint_ids_[joint], pinocchio::JointModelRevoluteUnaligned(pitch_axis), SE3::Identity(),
      jointName(joint, "pitch"), joint_effort, joint_velocity, joint_lower, joint_upper);
    model_.addJointFrame(pitch_joint_ids_[joint]);

    model_.appendBodyToJoint(pitch_joint_ids_[joint], link_inertia, link_body_placement);
    model_.addBodyFrame(
      "link_" + std::to_string(joint + 1), pitch_joint_ids_[joint], SE3::Identity());
    body_point_frame_ids_[joint + 1] = model_.addFrame(
      pinocchio::Frame(
        "body_point_" + std::to_string(joint + 1), pitch_joint_ids_[joint], SE3::Identity(),
        pinocchio::OP_FRAME));
  }

  body_point_frame_ids_[kNumPoints - 1] = model_.addFrame(
    pinocchio::Frame(
      "body_point_" + std::to_string(kNumPoints - 1), pitch_joint_ids_[kNumJoints - 1],
      translationX(-link_length), pinocchio::OP_FRAME));
}

Eigen::VectorXd AsrSdmKinematicModel::neutralConfiguration() const
{
  return pinocchio::neutral(model_);
}

Eigen::VectorXd AsrSdmKinematicModel::toConfiguration(
  const Eigen::Vector3d & head_position, const Eigen::Matrix3d & head_frame,
  const JointVector & theta) const
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);

  Eigen::Quaterniond orientation(head_frame);
  orientation.normalize();

  const int head_idx = model_.joints[head_joint_id_].idx_q();
  q.segment<3>(head_idx) = head_position;
  q[head_idx + 3] = orientation.x();
  q[head_idx + 4] = orientation.y();
  q[head_idx + 5] = orientation.z();
  q[head_idx + 6] = orientation.w();

  for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
    q[model_.joints[yaw_joint_ids_[joint]].idx_q()] =
      theta[static_cast<int>(yawIndex(joint))];
    q[model_.joints[pitch_joint_ids_[joint]].idx_q()] =
      theta[static_cast<int>(pitchIndex(joint))];
  }
  return q;
}

Eigen::VectorXd AsrSdmKinematicModel::toVelocity(
  double linear_velocity, double pitch_rate, double yaw_rate, const JointVector & theta_dot) const
{
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);

  const int head_idx = model_.joints[head_joint_id_].idx_v();
  v[head_idx + 0] = linear_velocity;
  v[head_idx + 4] = pitch_rate;
  v[head_idx + 5] = yaw_rate;

  for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
    v[model_.joints[yaw_joint_ids_[joint]].idx_v()] =
      theta_dot[static_cast<int>(yawIndex(joint))];
    v[model_.joints[pitch_joint_ids_[joint]].idx_v()] =
      theta_dot[static_cast<int>(pitchIndex(joint))];
  }
  return v;
}

void AsrSdmKinematicModel::updateKinematics(const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

void AsrSdmKinematicModel::updateKinematics(const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
}

std::array<Eigen::Vector3d, AsrSdmKinematicModel::kNumPoints>
AsrSdmKinematicModel::computeBodyPoints() const
{
  std::array<Eigen::Vector3d, kNumPoints> points{};
  for (std::size_t point = 0; point < kNumPoints; ++point) {
    points[point] = data_.oMf[body_point_frame_ids_[point]].translation();
  }
  return points;
}

std::array<Eigen::Matrix3d, AsrSdmKinematicModel::kNumLinks>
AsrSdmKinematicModel::computeLinkFrames() const
{
  std::array<Eigen::Matrix3d, kNumLinks> frames{};
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    frames[link] = data_.oMf[body_point_frame_ids_[link]].rotation();
  }
  return frames;
}

std::array<Eigen::Vector3d, AsrSdmKinematicModel::kNumLinks>
AsrSdmKinematicModel::computeLinkAxes() const
{
  std::array<Eigen::Vector3d, kNumLinks> axes{};
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    axes[link] = data_.oMf[body_point_frame_ids_[link]].rotation().col(0);
  }
  return axes;
}

Eigen::Vector3d AsrSdmKinematicModel::bodyPointVelocity(std::size_t point) const
{
  return pinocchio::getFrameVelocity(
    model_, data_, body_point_frame_ids_[point], pinocchio::LOCAL_WORLD_ALIGNED).linear();
}

AsrSdmKinematicModel::Matrix6xd AsrSdmKinematicModel::bodyPointJacobian(
  const Eigen::VectorXd & q, std::size_t point)
{
  Matrix6xd jacobian = Matrix6xd::Zero(6, model_.nv);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::getFrameJacobian(
    model_, data_, body_point_frame_ids_[point], pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  return jacobian;
}

Eigen::MatrixXd AsrSdmKinematicModel::massMatrix(const Eigen::VectorXd & q)
{
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data_.M;
}

Eigen::VectorXd AsrSdmKinematicModel::gravityTorque(const Eigen::VectorXd & q)
{
  return pinocchio::computeGeneralizedGravity(model_, data_, q);
}

}  // namespace asr
