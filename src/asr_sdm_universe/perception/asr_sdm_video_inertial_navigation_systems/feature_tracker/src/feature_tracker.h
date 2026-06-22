#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <rcutils/logging_macros.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"
#include "sparse_img_align.h"
#include "imu_preintegrate.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    // Sparse-image-align helpers. The node pushes IMU samples to imu_preint_
    // and sets the rotation prior between (prev_time, cur_time) before
    // calling readImage.
    void addImuSample(double t,
                      const Eigen::Vector3d& gyro,
                      const Eigen::Vector3d& accel);
    void setImuRotationPrior(const Eigen::Matrix3d& R_k_km1);
    void pruneImuBuffer(double t_min);

    // D2.1: expose last sparse-align result for the node to publish.
    // Written by readImage() whenever USE_SPARSE_ALIGN && have_imu_prior_.
    // Node reads trackerData[i].last_align_res_ after readImage returns.
    vins_sparse::SparseAlignResult last_align_res_;

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    // Sparse image alignment state.
    vins_sparse::ImuPreintegrator imu_preint_;
    std::vector<cv::Mat> prev_pyr_;
    std::vector<cv::Mat> saved_prev_pyr_;  // prev_pyr_ reused across frames
    std::vector<cv::Mat> cur_pyr_;
    Eigen::Matrix3d R_prev_cur_ = Eigen::Matrix3d::Identity();
    bool have_imu_prior_ = false;

    // Per-session sparse-align statistics (logged every 10 frames).
    int n_sparse_frames_   = 0;
    int n_sparse_success_  = 0;
    int n_sparse_meas_sum_ = 0;
    int n_frames_seen_     = 0;
    int n_klt_for_align_sum_ = 0;
    double n_sparse_chi2_sum_ = 0.0;
    double n_sparse_time_sum_ = 0.0;

    // Per-session KLT statistics (logged every 50 frames).
    int n_klt_calls_       = 0;
    int n_klt_w_           = 0;
    int n_klt_l_           = 0;
    int n_klt_sparse_prior_ = 0;
    double n_klt_cost_sum_ = 0.0;

    static int n_id;
};
