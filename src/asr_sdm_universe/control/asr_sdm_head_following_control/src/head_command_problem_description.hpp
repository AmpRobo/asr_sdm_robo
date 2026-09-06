#ifndef ASR_SDM_HEAD_FOLLOWING_CONTROL_HEAD_COMMAND_PROBLEM_DESCRIPTION_HPP_
#define ASR_SDM_HEAD_FOLLOWING_CONTROL_HEAD_COMMAND_PROBLEM_DESCRIPTION_HPP_

#include <grampc_s/problem_description/problem_description.hpp>

#include <array>

namespace asr
{

/// Nonholonomic head kinematics used by GRAMPC:
/// x = [px, py, pz, yaw, pitch], u = [v, ωy, ωz].
class HeadCommandProblemDescription : public grampc::ProblemDescription
{
public:
  explicit HeadCommandProblemDescription(const std::array<typeRNum, 13> & p_cost);

  void setCostWeights(const std::array<typeRNum, 13> & p_cost);
  void setReference(
    const std::array<typeRNum, 3> & p0,
    const std::array<typeRNum, 3> & v0,
    const std::array<typeRNum, 3> & a0,
    typeRNum yaw0,
    typeRNum pitch0,
    typeRNum yaw_dot,
    typeRNum pitch_dot,
    typeRNum t0);

  void ffct(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;
  void dfdx_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
    grampc::VectorConstRef p, grampc::VectorConstRef vec,
    const grampc::GrampcParam & param) override;
  void dfdu_vec(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
    grampc::VectorConstRef p, grampc::VectorConstRef vec,
    const grampc::GrampcParam & param) override;

  void lfct(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;
  void dldx(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;
  void dldu(
    grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
    grampc::VectorConstRef p, const grampc::GrampcParam & param) override;

  void Vfct(
    grampc::VectorRef out, ctypeRNum T, grampc::VectorConstRef x, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;
  void dVdx(
    grampc::VectorRef out, ctypeRNum T, grampc::VectorConstRef x, grampc::VectorConstRef p,
    const grampc::GrampcParam & param) override;

private:
  void poseError(
    ctypeRNum t, grampc::VectorConstRef x, typeRNum & ex, typeRNum & ey, typeRNum & ez,
    typeRNum & eyaw, typeRNum & epitch) const;

  std::array<typeRNum, 13> p_cost_{};
  std::array<typeRNum, 3> p0_{};
  std::array<typeRNum, 3> v0_{};
  std::array<typeRNum, 3> a0_{};
  typeRNum yaw0_{0.0};
  typeRNum pitch0_{0.0};
  typeRNum yaw_dot_{0.0};
  typeRNum pitch_dot_{0.0};
  typeRNum t0_{0.0};
};

}  // namespace asr

#endif  // ASR_SDM_HEAD_FOLLOWING_CONTROL_HEAD_COMMAND_PROBLEM_DESCRIPTION_HPP_
