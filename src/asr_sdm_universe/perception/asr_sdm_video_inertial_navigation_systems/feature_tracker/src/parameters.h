#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern double FX;
extern double FY;
extern double CX;
extern double CY;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// Sparse image alignment (SVO-style semi-direct refinement in the front-end).
extern int   USE_SPARSE_ALIGN;
extern int   USE_TD_PRE_CALIB;        // D2.2: extra pure-vision align + td estimator
extern int   SPARSE_ALIGN_PATCH_SIZE;
extern int   SPARSE_ALIGN_MAX_LEVEL;
extern int   SPARSE_ALIGN_MIN_LEVEL;
extern int   SPARSE_ALIGN_MAX_ITER;
extern double SPARSE_ALIGN_LAMBDA_ROT;
extern double SPARSE_ALIGN_LAMBDA_TRANS;
extern double SPARSE_ALIGN_CHI2_THRESH;
extern int    SPARSE_ALIGN_MIN_FEATURES;
extern int    SPARSE_ALIGN_MIN_ITER_FOR_OK;

void readParameters(rclcpp::Node::SharedPtr &n);
