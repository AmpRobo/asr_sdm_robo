#ifndef ASR_SDM_KINEMATIC_MODEL_HPP_
#define ASR_SDM_KINEMATIC_MODEL_HPP_

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>

namespace asr
{

struct AsrSdmKinematicModelParameters
{
  double link_length{0.25};
  double link_mass{1.5};
  double link_radius{0.05};
  double joint_limit{1.5707963267948966};
  double joint_velocity_limit{2.0};
  double joint_effort_limit{50.0};
};

/// Follow-the-leader body assembled joint by joint instead of parsed from a
/// URDF, reproducing the chain
///
///   R_{i+1} = R_i * Rz(-theta_yaw_i) * Ry(-theta_pitch_i)
///   p_{i+1} = p_i - link_length * R_i.col(0)
///
/// The head is carried by a free-flyer, and every articulation is split into a
/// yaw revolute about -Z followed by a pitch revolute about -Y so that a
/// Pinocchio joint coordinate is numerically the physical joint angle itself.
class AsrSdmKinematicModel
{
public:
  static constexpr std::size_t kNumJoints = 3;
  static constexpr std::size_t kNumLinks = kNumJoints + 1;
  static constexpr std::size_t kNumPoints = kNumLinks + 1;
  static constexpr std::size_t kNumJointDofs = 2 * kNumJoints;

  using Model = pinocchio::Model;
  using Data = pinocchio::Data;
  using JointVector = Eigen::Matrix<double, static_cast<int>(kNumJointDofs), 1>;
  using Matrix6xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  /// Each physical serial joint pair is yaw first, then pitch.
  static constexpr std::size_t yawIndex(std::size_t joint) {return 2 * joint;}
  static constexpr std::size_t pitchIndex(std::size_t joint) {return 2 * joint + 1;}

  explicit AsrSdmKinematicModel(const AsrSdmKinematicModelParameters & params = {});

  const Model & model() const {return model_;}
  Data & data() {return data_;}
  const Data & data() const {return data_;}
  const AsrSdmKinematicModelParameters & parameters() const {return params_;}

  pinocchio::JointIndex headJointId() const {return head_joint_id_;}
  pinocchio::JointIndex yawJointId(std::size_t joint) const {return yaw_joint_ids_[joint];}
  pinocchio::JointIndex pitchJointId(std::size_t joint) const {return pitch_joint_ids_[joint];}
  pinocchio::FrameIndex bodyPointFrameId(std::size_t point) const
  {
    return body_point_frame_ids_[point];
  }

  Eigen::VectorXd neutralConfiguration() const;
  Eigen::VectorXd toConfiguration(
    const Eigen::Vector3d & head_position, const Eigen::Matrix3d & head_frame,
    const JointVector & theta) const;
  /// The head twist is expressed in the head frame: forward along x, pitch about
  /// y, yaw about z.
  Eigen::VectorXd toVelocity(
    double linear_velocity, double pitch_rate, double yaw_rate,
    const JointVector & theta_dot) const;

  void updateKinematics(const Eigen::VectorXd & q);
  void updateKinematics(const Eigen::VectorXd & q, const Eigen::VectorXd & v);

  std::array<Eigen::Vector3d, kNumPoints> computeBodyPoints() const;
  std::array<Eigen::Matrix3d, kNumLinks> computeLinkFrames() const;
  std::array<Eigen::Vector3d, kNumLinks> computeLinkAxes() const;
  /// World-oriented velocity of a body point, valid after the two-argument
  /// updateKinematics().
  Eigen::Vector3d bodyPointVelocity(std::size_t point) const;

  /// LOCAL_WORLD_ALIGNED frame Jacobian, so rows 0-2 map v to the body point
  /// linear velocity expressed in world axes.
  Matrix6xd bodyPointJacobian(const Eigen::VectorXd & q, std::size_t point);
  Eigen::MatrixXd massMatrix(const Eigen::VectorXd & q);
  Eigen::VectorXd gravityTorque(const Eigen::VectorXd & q);

private:
  void build();

  AsrSdmKinematicModelParameters params_;
  Model model_;
  Data data_;
  pinocchio::JointIndex head_joint_id_{0};
  std::array<pinocchio::JointIndex, kNumJoints> yaw_joint_ids_{};
  std::array<pinocchio::JointIndex, kNumJoints> pitch_joint_ids_{};
  std::array<pinocchio::FrameIndex, kNumPoints> body_point_frame_ids_{};
};

}  // namespace asr

#endif  // ASR_SDM_KINEMATIC_MODEL_HPP_
