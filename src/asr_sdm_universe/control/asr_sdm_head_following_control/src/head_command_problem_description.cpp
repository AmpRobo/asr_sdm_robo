#include "head_command_problem_description.hpp"

#include "asr_sdm_head_following_control/control_utils.hpp"

#include <cmath>

namespace asr
{
namespace
{
typeRNum wrap(typeRNum angle)
{
  return static_cast<typeRNum>(wrapAngle(static_cast<double>(angle)));
}
}  // namespace

HeadCommandProblemDescription::HeadCommandProblemDescription(const std::array<typeRNum, 13> & p_cost)
: ProblemDescription(5, 3, 0, 0, 0, 0, 0), p_cost_(p_cost)
{
}

void HeadCommandProblemDescription::setCostWeights(const std::array<typeRNum, 13> & p_cost)
{
  p_cost_ = p_cost;
}

void HeadCommandProblemDescription::setReference(
  const std::array<typeRNum, 3> & p0,
  const std::array<typeRNum, 3> & v0,
  const std::array<typeRNum, 3> & a0,
  typeRNum yaw0,
  typeRNum pitch0,
  typeRNum yaw_dot,
  typeRNum pitch_dot,
  typeRNum t0)
{
  p0_ = p0;
  v0_ = v0;
  a0_ = a0;
  yaw0_ = yaw0;
  pitch0_ = pitch0;
  yaw_dot_ = yaw_dot;
  pitch_dot_ = pitch_dot;
  t0_ = t0;
}

void HeadCommandProblemDescription::poseError(
  ctypeRNum t, grampc::VectorConstRef x, typeRNum & ex, typeRNum & ey, typeRNum & ez,
  typeRNum & eyaw, typeRNum & epitch) const
{
  const typeRNum tau = t - t0_;
  const typeRNum px_ref = p0_[0] + v0_[0] * tau + typeRNum(0.5) * a0_[0] * tau * tau;
  const typeRNum py_ref = p0_[1] + v0_[1] * tau + typeRNum(0.5) * a0_[1] * tau * tau;
  const typeRNum pz_ref = p0_[2] + v0_[2] * tau + typeRNum(0.5) * a0_[2] * tau * tau;
  const typeRNum yaw_ref = yaw0_ + yaw_dot_ * tau;
  const typeRNum pitch_ref = pitch0_ + pitch_dot_ * tau;
  ex = x[0] - px_ref;
  ey = x[1] - py_ref;
  ez = x[2] - pz_ref;
  eyaw = wrap(x[3] - yaw_ref);
  epitch = wrap(x[4] - pitch_ref);
}

void HeadCommandProblemDescription::ffct(
  grampc::VectorRef out, ctypeRNum /*t*/, grampc::VectorConstRef x, grampc::VectorConstRef u,
  grampc::VectorConstRef /*p*/, const grampc::GrampcParam & /*param*/)
{
  const typeRNum cy = std::cos(x[3]);
  const typeRNum sy = std::sin(x[3]);
  const typeRNum cp = std::cos(x[4]);
  const typeRNum sp = std::sin(x[4]);
  out[0] = u[0] * cy * cp;
  out[1] = u[0] * sy * cp;
  out[2] = -u[0] * sp;
  out[3] = u[2];
  out[4] = u[1];
}

void HeadCommandProblemDescription::dfdx_vec(
  grampc::VectorRef out, ctypeRNum /*t*/, grampc::VectorConstRef x, grampc::VectorConstRef u,
  grampc::VectorConstRef /*p*/, grampc::VectorConstRef vec, const grampc::GrampcParam & /*param*/)
{
  const typeRNum cy = std::cos(x[3]);
  const typeRNum sy = std::sin(x[3]);
  const typeRNum cp = std::cos(x[4]);
  const typeRNum sp = std::sin(x[4]);
  const typeRNum v = u[0];
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  out[3] = vec[0] * (-v * sy * cp) + vec[1] * (v * cy * cp);
  out[4] = vec[0] * (-v * cy * sp) + vec[1] * (-v * sy * sp) + vec[2] * (-v * cp);
}

void HeadCommandProblemDescription::dfdu_vec(
  grampc::VectorRef out, ctypeRNum /*t*/, grampc::VectorConstRef x, grampc::VectorConstRef /*u*/,
  grampc::VectorConstRef /*p*/, grampc::VectorConstRef vec, const grampc::GrampcParam & /*param*/)
{
  const typeRNum cy = std::cos(x[3]);
  const typeRNum sy = std::sin(x[3]);
  const typeRNum cp = std::cos(x[4]);
  const typeRNum sp = std::sin(x[4]);
  out[0] = vec[0] * cy * cp + vec[1] * sy * cp + vec[2] * (-sp);
  out[1] = vec[4];
  out[2] = vec[3];
}

void HeadCommandProblemDescription::lfct(
  grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef u,
  grampc::VectorConstRef /*p*/, const grampc::GrampcParam & param)
{
  typeRNum ex, ey, ez, eyaw, epitch;
  poseError(t, x, ex, ey, ez, eyaw, epitch);
  const auto & udes = param.udes;
  out[0] = p_cost_[0] * ex * ex + p_cost_[1] * ey * ey + p_cost_[2] * ez * ez +
    p_cost_[3] * eyaw * eyaw + p_cost_[4] * epitch * epitch +
    p_cost_[5] * (u[0] - udes[0]) * (u[0] - udes[0]) +
    p_cost_[6] * (u[1] - udes[1]) * (u[1] - udes[1]) +
    p_cost_[7] * (u[2] - udes[2]) * (u[2] - udes[2]);
}

void HeadCommandProblemDescription::dldx(
  grampc::VectorRef out, ctypeRNum t, grampc::VectorConstRef x, grampc::VectorConstRef /*u*/,
  grampc::VectorConstRef /*p*/, const grampc::GrampcParam & /*param*/)
{
  typeRNum ex, ey, ez, eyaw, epitch;
  poseError(t, x, ex, ey, ez, eyaw, epitch);
  out[0] = typeRNum(2) * p_cost_[0] * ex;
  out[1] = typeRNum(2) * p_cost_[1] * ey;
  out[2] = typeRNum(2) * p_cost_[2] * ez;
  out[3] = typeRNum(2) * p_cost_[3] * eyaw;
  out[4] = typeRNum(2) * p_cost_[4] * epitch;
}

void HeadCommandProblemDescription::dldu(
  grampc::VectorRef out, ctypeRNum /*t*/, grampc::VectorConstRef /*x*/, grampc::VectorConstRef u,
  grampc::VectorConstRef /*p*/, const grampc::GrampcParam & param)
{
  const auto & udes = param.udes;
  out[0] = typeRNum(2) * p_cost_[5] * (u[0] - udes[0]);
  out[1] = typeRNum(2) * p_cost_[6] * (u[1] - udes[1]);
  out[2] = typeRNum(2) * p_cost_[7] * (u[2] - udes[2]);
}

void HeadCommandProblemDescription::Vfct(
  grampc::VectorRef out, ctypeRNum T, grampc::VectorConstRef x, grampc::VectorConstRef /*p*/,
  const grampc::GrampcParam & /*param*/)
{
  typeRNum ex, ey, ez, eyaw, epitch;
  poseError(T, x, ex, ey, ez, eyaw, epitch);
  out[0] = p_cost_[8] * ex * ex + p_cost_[9] * ey * ey + p_cost_[10] * ez * ez +
    p_cost_[11] * eyaw * eyaw + p_cost_[12] * epitch * epitch;
}

void HeadCommandProblemDescription::dVdx(
  grampc::VectorRef out, ctypeRNum T, grampc::VectorConstRef x, grampc::VectorConstRef /*p*/,
  const grampc::GrampcParam & /*param*/)
{
  typeRNum ex, ey, ez, eyaw, epitch;
  poseError(T, x, ex, ey, ez, eyaw, epitch);
  out[0] = typeRNum(2) * p_cost_[8] * ex;
  out[1] = typeRNum(2) * p_cost_[9] * ey;
  out[2] = typeRNum(2) * p_cost_[10] * ez;
  out[3] = typeRNum(2) * p_cost_[11] * eyaw;
  out[4] = typeRNum(2) * p_cost_[12] * epitch;
}

}  // namespace asr
