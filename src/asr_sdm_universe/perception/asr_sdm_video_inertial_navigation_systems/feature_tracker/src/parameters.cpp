#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
double FX;
double FY;
double CX;
double CY;
int FISHEYE;
bool PUB_THIS_FRAME;

int   USE_SPARSE_ALIGN;
int   USE_TD_PRE_CALIB;
int   SPARSE_ALIGN_PATCH_SIZE;
int   SPARSE_ALIGN_MAX_LEVEL;
int   SPARSE_ALIGN_MIN_LEVEL;
int   SPARSE_ALIGN_MAX_ITER;
double SPARSE_ALIGN_LAMBDA_ROT;
double SPARSE_ALIGN_LAMBDA_TRANS;
double SPARSE_ALIGN_CHI2_THRESH;
int    SPARSE_ALIGN_MIN_FEATURES;
int    SPARSE_ALIGN_MIN_ITER_FOR_OK;

namespace
{

template <typename T>
T getParamOrDeclare(rclcpp::Node::SharedPtr n, const std::string& name, const T& default_value)
{
    if (!n->has_parameter(name)) {
        n->declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    n->get_parameter(name, value);
    return value;
}

void readFromRosParams(rclcpp::Node::SharedPtr& n)
{
    IMAGE_TOPIC = getParamOrDeclare<std::string>(n, "image_topic", "/cam0/image_raw");
    IMU_TOPIC = getParamOrDeclare<std::string>(n, "imu_topic", "/imu0");

    std::string config_pkg_share =
        getParamOrDeclare<std::string>(n, "config_pkg_share", "");
    if (config_pkg_share.empty()) {
        config_pkg_share = getParamOrDeclare<std::string>(n, "vins_folder", "");
    }
    if (config_pkg_share.empty()) {
        RCLCPP_ERROR(n->get_logger(), "Neither config_pkg_share nor vins_folder is set!");
    }

    MAX_CNT = getParamOrDeclare<int>(n, "max_cnt", 150);
    MIN_DIST = getParamOrDeclare<int>(n, "min_dist", 30);
    ROW = getParamOrDeclare<int>(n, "image_height", 480);
    COL = getParamOrDeclare<int>(n, "image_width", 752);
    FREQ = getParamOrDeclare<int>(n, "freq", 10);
    F_THRESHOLD = getParamOrDeclare<double>(n, "F_threshold", 1.0);
    SHOW_TRACK = getParamOrDeclare<int>(n, "show_track", 1);
    EQUALIZE = getParamOrDeclare<int>(n, "equalize", 0);
    FISHEYE = getParamOrDeclare<int>(n, "fisheye", 0);
    if (FISHEYE == 1) {
        FISHEYE_MASK = config_pkg_share + "/config/fisheye_mask.jpg";
    }

    FX = getParamOrDeclare<double>(n, "projection_fx", 460.0);
    FY = getParamOrDeclare<double>(n, "projection_fy", 460.0);
    CX = getParamOrDeclare<double>(n, "projection_cx", 320.0);
    CY = getParamOrDeclare<double>(n, "projection_cy", 240.0);
    FOCAL_LENGTH = static_cast<int>(0.5 * (FX + FY));

    std::string calib_file =
        getParamOrDeclare<std::string>(n, "camera_calibration_file", "");
    if (calib_file.empty()) {
        RCLCPP_ERROR(n->get_logger(), "camera_calibration_file is required");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(n->get_logger(), "calibration_file: " << calib_file);
    CAM_NAMES.push_back(calib_file);

    USE_SPARSE_ALIGN = getParamOrDeclare<int>(n, "use_sparse_align", 0);
    USE_TD_PRE_CALIB = getParamOrDeclare<int>(n, "use_td_pre_calib", 0);
    SPARSE_ALIGN_PATCH_SIZE = getParamOrDeclare<int>(n, "sparse_align_patch_size", 4);
    SPARSE_ALIGN_MAX_LEVEL = getParamOrDeclare<int>(n, "sparse_align_max_level", 3);
    SPARSE_ALIGN_MIN_LEVEL = getParamOrDeclare<int>(n, "sparse_align_min_level", 1);
    SPARSE_ALIGN_MAX_ITER = getParamOrDeclare<int>(n, "sparse_align_max_iter", 8);
    SPARSE_ALIGN_LAMBDA_ROT = getParamOrDeclare<double>(n, "sparse_align_lambda_rot", 0.5);
    SPARSE_ALIGN_LAMBDA_TRANS = getParamOrDeclare<double>(n, "sparse_align_lambda_trans", 0.0);
    SPARSE_ALIGN_CHI2_THRESH = getParamOrDeclare<double>(n, "sparse_align_chi2_thresh", 50.0);
    SPARSE_ALIGN_MIN_FEATURES = getParamOrDeclare<int>(n, "sparse_align_min_features", 30);
    SPARSE_ALIGN_MIN_ITER_FOR_OK = getParamOrDeclare<int>(n, "sparse_align_min_iter_for_ok", 2);
}

void readFromConfigFile(rclcpp::Node::SharedPtr& n, const std::string& config_file)
{
    RCLCPP_INFO_STREAM(n->get_logger(), "config_file: " << config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        RCLCPP_ERROR_STREAM(n->get_logger(), "ERROR: Wrong path to settings");
        return;
    }

    std::string config_pkg_share =
        getParamOrDeclare<std::string>(n, "config_pkg_share", "");
    if (config_pkg_share.empty()) {
        config_pkg_share = getParamOrDeclare<std::string>(n, "vins_folder", "");
    }
    if (config_pkg_share.empty()) {
        RCLCPP_ERROR(n->get_logger(), "Neither config_pkg_share nor vins_folder is set!");
    }

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1) {
        FISHEYE_MASK = config_pkg_share + "/config/fisheye_mask.jpg";
    }

    cv::FileNode pp = fsSettings["projection_parameters"];
    FX = pp["fx"];
    FY = pp["fy"];
    CX = pp["cx"];
    CY = pp["cy"];
    FOCAL_LENGTH = static_cast<int>(0.5 * (FX + FY));

    std::string calib_file = getParamOrDeclare<std::string>(n, "camera_calibration_file", "");
    if (calib_file.empty()) {
        calib_file = config_file;
    }
    RCLCPP_INFO_STREAM(n->get_logger(), "calibration_file: " << calib_file);
    CAM_NAMES.push_back(calib_file);

    auto readInt = [&](const char* key, int default_v) {
        cv::FileNode node = fsSettings[key];
        if (node.isInt() || node.isReal()) return static_cast<int>(static_cast<double>(node));
        return default_v;
    };
    auto readDouble = [&](const char* key, double default_v) {
        cv::FileNode node = fsSettings[key];
        if (node.isInt() || node.isReal()) return static_cast<double>(node);
        return default_v;
    };

    USE_SPARSE_ALIGN = readInt("use_sparse_align", 0);
    USE_TD_PRE_CALIB = readInt("use_td_pre_calib", 0);
    SPARSE_ALIGN_PATCH_SIZE = readInt("sparse_align_patch_size", 4);
    SPARSE_ALIGN_MAX_LEVEL = readInt("sparse_align_max_level", 3);
    SPARSE_ALIGN_MIN_LEVEL = readInt("sparse_align_min_level", 1);
    SPARSE_ALIGN_MAX_ITER = readInt("sparse_align_max_iter", 8);
    SPARSE_ALIGN_LAMBDA_ROT = readDouble("sparse_align_lambda_rot", 0.5);
    SPARSE_ALIGN_LAMBDA_TRANS = readDouble("sparse_align_lambda_trans", 0.0);
    SPARSE_ALIGN_CHI2_THRESH = readDouble("sparse_align_chi2_thresh", 50.0);
    SPARSE_ALIGN_MIN_FEATURES = readInt("sparse_align_min_features", 30);
    SPARSE_ALIGN_MIN_ITER_FOR_OK = readInt("sparse_align_min_iter_for_ok", 2);

    fsSettings.release();
}

}  // namespace

void readParameters(rclcpp::Node::SharedPtr &n)
{
    const std::string image_topic =
        getParamOrDeclare<std::string>(n, "image_topic", "");
    const std::string config_file =
        getParamOrDeclare<std::string>(n, "config_file", "");

    if (!image_topic.empty()) {
        readFromRosParams(n);
    } else if (!config_file.empty()) {
        readFromConfigFile(n, config_file);
    } else {
        RCLCPP_ERROR(n->get_logger(), "Set image_topic via ROS params yaml or pass config_file");
        rclcpp::shutdown();
        return;
    }

    RCUTILS_LOG_INFO("sparse_align enabled: %d (patch=%d, levels=[%d,%d], lambda_rot=%.3f, chi2_thresh=%.1f, min_features=%d, td_pre_calib=%d)",
                USE_SPARSE_ALIGN, SPARSE_ALIGN_PATCH_SIZE,
                SPARSE_ALIGN_MIN_LEVEL, SPARSE_ALIGN_MAX_LEVEL,
                SPARSE_ALIGN_LAMBDA_ROT, SPARSE_ALIGN_CHI2_THRESH,
                SPARSE_ALIGN_MIN_FEATURES, USE_TD_PRE_CALIB);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    if (FREQ == 0) {
        FREQ = 100;
    }
}
