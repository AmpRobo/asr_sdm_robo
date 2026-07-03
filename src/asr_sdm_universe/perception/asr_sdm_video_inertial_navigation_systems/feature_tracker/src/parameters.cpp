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
int   USE_TD_PRE_CALIB;     // D2.2: enable a second pure-vision align pass + td estimator
int   SPARSE_ALIGN_PATCH_SIZE;
int   SPARSE_ALIGN_MAX_LEVEL;
int   SPARSE_ALIGN_MIN_LEVEL;
int   SPARSE_ALIGN_MAX_ITER;
double SPARSE_ALIGN_LAMBDA_ROT;
double SPARSE_ALIGN_LAMBDA_TRANS;
double SPARSE_ALIGN_CHI2_THRESH;
int    SPARSE_ALIGN_MIN_FEATURES;
int    SPARSE_ALIGN_MIN_ITER_FOR_OK;

template <typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
    T ans;
    std::string default_value = "";
    n->declare_parameter<std::string>(name, default_value);
    if (n->get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(n->get_logger(), "Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

void readParameters(rclcpp::Node::SharedPtr &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    RCLCPP_INFO_STREAM(n->get_logger(), "config_file: " << config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        RCLCPP_ERROR_STREAM(n->get_logger(), "ERROR: Wrong path to settings");
    }
    // config_pkg_share points to <pkg>/share/config_pkg. Used to resolve
    // auxiliary files (e.g. fisheye_mask.jpg). Optional: pass via launch param.
    std::string config_pkg_share;
    config_pkg_share = readParam<std::string>(n, "config_pkg_share");
    if (config_pkg_share.empty()) {
        config_pkg_share = readParam<std::string>(n, "vins_folder");
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
    if (FISHEYE == 1)
        FISHEYE_MASK = config_pkg_share + "/config/fisheye_mask.jpg";

    // Read projection intrinsics (fx, fy, cx, cy) from the main yaml so the
    // sparse image alignment sees the same pinhole as the camodocal camera
    // model. FOCAL_LENGTH is kept for backwards compatibility (other code
    // may still reference it).
    cv::FileNode pp = fsSettings["projection_parameters"];
    FX = pp["fx"];
    FY = pp["fy"];
    CX = pp["cx"];
    CY = pp["cy"];
    FOCAL_LENGTH = static_cast<int>(0.5 * (FX + FY));

    // Resolve camera calibration file for camodocal.
    // Default: reuse the main config_file, which is expected to be a camodocal-
    // compatible OpenCV YAML (model_type + distortion_parameters +
    // projection_parameters + image_width/height). Extra VINS fields are
    // ignored by camodocal. Override with launch param `camera_calibration_file`
    // to point at a separate calibration yaml if desired.
    std::string calib_file = readParam<std::string>(n, "camera_calibration_file");
    if (calib_file.empty()) {
        calib_file = config_file;
    }
    RCLCPP_INFO_STREAM(n->get_logger(), "calibration_file: " << calib_file);
    CAM_NAMES.push_back(calib_file);

    // Sparse image alignment parameters (all optional, default off).
    auto readInt = [&](const char* key, int default_v) {
        cv::FileNode n = fsSettings[key];
        if (n.isInt() || n.isReal()) return (int)(double)n;
        return default_v;
    };
    auto readDouble = [&](const char* key, double default_v) {
        cv::FileNode n = fsSettings[key];
        if (n.isInt() || n.isReal()) return (double)n;
        return default_v;
    };

    USE_SPARSE_ALIGN          = readInt("use_sparse_align", 0);
    USE_TD_PRE_CALIB          = readInt("use_td_pre_calib", 0);  // D2.2: default off
    SPARSE_ALIGN_PATCH_SIZE   = readInt("sparse_align_patch_size", 4);
    SPARSE_ALIGN_MAX_LEVEL    = readInt("sparse_align_max_level", 3);
    SPARSE_ALIGN_MIN_LEVEL    = readInt("sparse_align_min_level", 1);
    SPARSE_ALIGN_MAX_ITER     = readInt("sparse_align_max_iter", 8);
    SPARSE_ALIGN_LAMBDA_ROT   = readDouble("sparse_align_lambda_rot", 0.5);
    SPARSE_ALIGN_LAMBDA_TRANS = readDouble("sparse_align_lambda_trans", 0.0);
    SPARSE_ALIGN_CHI2_THRESH  = readDouble("sparse_align_chi2_thresh", 50.0);
    SPARSE_ALIGN_MIN_FEATURES = readInt("sparse_align_min_features", 30);
    SPARSE_ALIGN_MIN_ITER_FOR_OK = readInt("sparse_align_min_iter_for_ok", 2);

    RCUTILS_LOG_INFO("sparse_align enabled: %d (patch=%d, levels=[%d,%d], lambda_rot=%.3f, chi2_thresh=%.1f, min_features=%d, td_pre_calib=%d)",
                USE_SPARSE_ALIGN, SPARSE_ALIGN_PATCH_SIZE,
                SPARSE_ALIGN_MIN_LEVEL, SPARSE_ALIGN_MAX_LEVEL,
                SPARSE_ALIGN_LAMBDA_ROT, SPARSE_ALIGN_CHI2_THRESH,
                SPARSE_ALIGN_MIN_FEATURES, USE_TD_PRE_CALIB);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
