#include "svo_vio_backend/vins_types.h"
#include <cmath>

namespace svo_vio_backend {

Eigen::Matrix3d CameraCalibration::getK() const
{
  Eigen::Matrix3d K;
  K << fx, 0, cx,
       0, fy, cy,
       0, 0, 1;
  return K;
}

Eigen::Quaterniond VinsUtility::deltaQ(const Eigen::Vector3d& theta)
{
  Eigen::Quaterniond q;
  Eigen::Vector3d half_theta = theta * 0.5;
  q.w() = 1.0 - half_theta.squaredNorm() / 8.0;
  q.x() = half_theta.x() * (1.0 - half_theta.squaredNorm() / 24.0);
  q.y() = half_theta.y() * (1.0 - half_theta.squaredNorm() / 24.0);
  q.z() = half_theta.z() * (1.0 - half_theta.squaredNorm() / 24.0);
  return q.normalized();
}

Eigen::Matrix3d VinsUtility::skewSymmetric(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(),
       v.z(), 0, -v.x(),
      -v.y(), v.x(), 0;
  return S;
}

Eigen::Matrix3d VinsUtility::quatToRotation(const Eigen::Quaterniond& q)
{
  return q.toRotationMatrix();
}

Eigen::Quaterniond VinsUtility::rotationToQuat(const Eigen::Matrix3d& R)
{
  return Eigen::Quaterniond(R);
}

Eigen::Quaterniond VinsUtility::normalizeQuat(const Eigen::Quaterniond& q)
{
  return q.normalized();
}

Eigen::Matrix4d VinsUtility::inverseSE3(const Eigen::Matrix4d& T)
{
  Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
  T_inv.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
  T_inv.block<3,1>(0,3) = -T.block<3,3>(0,0).transpose() * T.block<3,1>(0,3);
  return T_inv;
}

Eigen::Matrix3d VinsUtility::ypr2R(const Eigen::Vector3d& ypr)
{
  double y = ypr[0] / 180.0 * M_PI;
  double p = ypr[1] / 180.0 * M_PI;
  double r = ypr[2] / 180.0 * M_PI;

  Eigen::Matrix3d Rpitch = Eigen::Matrix3d::Identity();
  Rpitch << cos(p), 0, sin(p),
            0, 1, 0,
           -sin(p), 0, cos(p);

  Eigen::Matrix3d Ryaw = Eigen::Matrix3d::Identity();
  Ryaw << cos(y), -sin(y), 0,
          sin(y), cos(y), 0,
          0, 0, 1;

  Eigen::Matrix3d Rroll = Eigen::Matrix3d::Identity();
  Rroll << 1, 0, 0,
           0, cos(r), -sin(r),
           0, sin(r), cos(r);

  return Ryaw * Rpitch * Rroll;
}

Eigen::Vector3d VinsUtility::R2ypr(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr;
  ypr[2] = atan2(n(1), n(0));
  double r = atan2(n(2), sqrt(n(0)*n(0) + n(1)*n(1)));
  double p = atan2(-n(2), sqrt(n(0)*n(0) + n(1)*n(1) + n(2)*n(2)));
  
  ypr[0] = ypr[2] * 180.0 / M_PI;
  ypr[1] = p * 180.0 / M_PI;
  ypr[2] = r * 180.0 / M_PI;
  
  return ypr;
}

Eigen::Matrix3d VinsUtility::angle2R(const Eigen::Vector3d& rpy)
{
  return ypr2R(rpy);
}

}  // namespace svo_vio_backend
