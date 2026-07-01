#include "svo_vio_backend/vins_sliding_window.h"

namespace svo_vio_backend {

VinsSlidingWindow::VinsSlidingWindow()
{
  for (int i = 0; i < WINDOW_SIZE + 1; ++i) {
    Rs_[i].setIdentity();
    Ps_[i].setZero();
    Vs_[i].setZero();
    Bas_[i].setZero();
    Bgs_[i].setZero();
  }
  
  ric_[0].setIdentity();
  tic_[0].setZero();
}

VinsSlidingWindow::~VinsSlidingWindow()
{
}

void VinsSlidingWindow::setWindowSize(int size)
{
  window_size_ = size;
}

void VinsSlidingWindow::clear()
{
  window_size_ = 0;
  for (int i = 0; i < WINDOW_SIZE + 1; ++i) {
    Rs_[i].setIdentity();
    Ps_[i].setZero();
    Vs_[i].setZero();
    Bas_[i].setZero();
    Bgs_[i].setZero();
  }
}

void VinsSlidingWindow::addState(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
{
  if (window_size_ < WINDOW_SIZE + 1) {
    Rs_[window_size_] = R;
    Ps_[window_size_] = p;
    Vs_[window_size_].setZero();
    Bas_[window_size_].setZero();
    Bgs_[window_size_].setZero();
    ++window_size_;
  }
}

void VinsSlidingWindow::slideWindow()
{
  if (window_size_ <= 0) return;
  
  for (int i = 0; i < WINDOW_SIZE; ++i) {
    Rs_[i] = Rs_[i + 1];
    Ps_[i] = Ps_[i + 1];
    Vs_[i] = Vs_[i + 1];
    Bas_[i] = Bas_[i + 1];
    Bgs_[i] = Bgs_[i + 1];
  }
  
  Rs_[WINDOW_SIZE].setIdentity();
  Ps_[WINDOW_SIZE].setZero();
  Vs_[WINDOW_SIZE].setZero();
  Bas_[WINDOW_SIZE].setZero();
  Bgs_[WINDOW_SIZE].setZero();
}

}  // namespace svo_vio_backend
