#include "parameters.h"

#include <stdexcept>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

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

Eigen::Matrix3d toMatrix3dRowMajor(const std::vector<double>& data)
{
    if (data.size() != 9) {
        throw std::runtime_error("extrinsic_rotation must have 9 elements");
    }
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> map(data.data());
    return map;
}

Eigen::Vector3d toVector3d(const std::vector<double>& data)
{
    if (data.size() != 3) {
        throw std::runtime_error("extrinsic_translation must have 3 elements");
    }
    return Eigen::Vector3d(data[0], data[1], data[2]);
}

void loadExtrinsic(
    rclcpp::Node::SharedPtr n,
    const std::string& output_path,
    const Eigen::Matrix3d& eigen_R,
    const Eigen::Vector3d& eigen_T)
{
    if (ESTIMATE_EXTRINSIC == 2) {
        RCLCPP_WARN(n->get_logger(), "have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = output_path + "/extrinsic_parameter.csv";
        return;
    }

    if (ESTIMATE_EXTRINSIC == 1) {
        RCLCPP_WARN(n->get_logger(), " Optimize extrinsic param around initial guess!");
        EX_CALIB_RESULT_PATH = output_path + "/extrinsic_parameter.csv";
    }
    if (ESTIMATE_EXTRINSIC == 0) {
        RCLCPP_WARN(n->get_logger(), " fix extrinsic param ");
    }

    Eigen::Matrix3d eigen_R_norm = eigen_R;
    Eigen::Quaterniond Q(eigen_R_norm);
    eigen_R_norm = Q.normalized().toRotationMatrix();
    RIC.push_back(eigen_R_norm);
    TIC.push_back(eigen_T);
    RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_R : " << std::endl << RIC[0]);
    RCLCPP_INFO_STREAM(n->get_logger(), "Extrinsic_T : " << std::endl << TIC[0].transpose());
}

void readCommonEstimatorParams(
    rclcpp::Node::SharedPtr n,
    std::string& output_path)
{
    IMU_TOPIC = getParamOrDeclare<std::string>(n, "imu_topic", "/imu0");

    SOLVER_TIME = getParamOrDeclare<double>(n, "max_solver_time", 0.04);
    NUM_ITERATIONS = getParamOrDeclare<int>(n, "max_num_iterations", 8);
    MIN_PARALLAX = getParamOrDeclare<double>(n, "keyframe_parallax", 10.0);
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string config_pkg_share =
        getParamOrDeclare<std::string>(n, "config_pkg_share", "");
    if (config_pkg_share.empty()) {
        config_pkg_share = getParamOrDeclare<std::string>(n, "vins_folder", "");
    }

    output_path = FileSystemHelper::resolveWorkspacePath(
        getParamOrDeclare<std::string>(n, "output_path", "output"),
        config_pkg_share);
    VINS_RESULT_PATH = output_path + "/vins_result_no_loop.csv";
    RCLCPP_INFO(n->get_logger(), "result path %s", VINS_RESULT_PATH.c_str());

    FileSystemHelper::createDirectoryIfNotExists(output_path.c_str());
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = getParamOrDeclare<double>(n, "acc_n", 0.08);
    ACC_W = getParamOrDeclare<double>(n, "acc_w", 0.00004);
    GYR_N = getParamOrDeclare<double>(n, "gyr_n", 0.004);
    GYR_W = getParamOrDeclare<double>(n, "gyr_w", 2.0e-6);
    G.z() = getParamOrDeclare<double>(n, "g_norm", 9.81);
    ROW = getParamOrDeclare<int>(n, "image_height", 480);
    COL = getParamOrDeclare<int>(n, "image_width", 752);
    RCLCPP_INFO(n->get_logger(), "ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = getParamOrDeclare<int>(n, "estimate_extrinsic", 1);
    TD = getParamOrDeclare<double>(n, "td", 0.0);
    ESTIMATE_TD = getParamOrDeclare<int>(n, "estimate_td", 0);
    ROLLING_SHUTTER = getParamOrDeclare<int>(n, "rolling_shutter", 0);
    TR = ROLLING_SHUTTER
        ? getParamOrDeclare<double>(n, "rolling_shutter_tr", 0.0)
        : 0.0;
}

void readFromRosParams(rclcpp::Node::SharedPtr n)
{
    std::string output_path;
    readCommonEstimatorParams(n, output_path);

    RIC.clear();
    TIC.clear();

    if (ESTIMATE_EXTRINSIC == 2) {
        loadExtrinsic(n, output_path, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    } else {
        const auto R_vec = getParamOrDeclare<std::vector<double>>(
            n, "extrinsic_rotation",
            std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        const auto T_vec = getParamOrDeclare<std::vector<double>>(
            n, "extrinsic_translation",
            std::vector<double>{0.0, 0.0, 0.0});
        loadExtrinsic(n, output_path, toMatrix3dRowMajor(R_vec), toVector3d(T_vec));
    }
}

void readFromConfigFile(rclcpp::Node::SharedPtr n, const std::string& config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        RCLCPP_ERROR(n->get_logger(), "ERROR: Wrong path to settings");
        return;
    }

    std::string output_path;
    IMU_TOPIC.clear();
    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> output_path;
    std::string config_pkg_share =
        getParamOrDeclare<std::string>(n, "config_pkg_share", "");
    if (config_pkg_share.empty()) {
        config_pkg_share = getParamOrDeclare<std::string>(n, "vins_folder", "");
    }
    output_path = FileSystemHelper::resolveWorkspacePath(output_path, config_pkg_share);
    VINS_RESULT_PATH = output_path + "/vins_result_no_loop.csv";
    RCLCPP_INFO(n->get_logger(), "result path %s", VINS_RESULT_PATH.c_str());

    FileSystemHelper::createDirectoryIfNotExists(output_path.c_str());
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    RCLCPP_INFO(n->get_logger(), "ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    RIC.clear();
    TIC.clear();

    if (ESTIMATE_EXTRINSIC == 2) {
        loadExtrinsic(n, output_path, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    } else {
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        loadExtrinsic(n, output_path, eigen_R, eigen_T);
    }

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    TR = ROLLING_SHUTTER ? static_cast<double>(fsSettings["rolling_shutter_tr"]) : 0.0;

    fsSettings.release();
}

}  // namespace

void readParameters(rclcpp::Node::SharedPtr n)
{
    const std::string imu_topic =
        getParamOrDeclare<std::string>(n, "imu_topic", "");
    const std::string config_file =
        getParamOrDeclare<std::string>(n, "config_file", "");

    if (!imu_topic.empty()) {
        readFromRosParams(n);
    } else if (!config_file.empty()) {
        readFromConfigFile(n, config_file);
    } else {
        RCLCPP_ERROR(n->get_logger(), "Set imu_topic via ROS params yaml or pass config_file");
        rclcpp::shutdown();
        return;
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    if (ESTIMATE_TD) {
        RCLCPP_INFO_STREAM(n->get_logger(), "Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    } else {
        RCLCPP_INFO_STREAM(n->get_logger(), "Synchronized sensors, fix time offset: " << TD);
    }

    if (ROLLING_SHUTTER) {
        RCLCPP_INFO_STREAM(n->get_logger(), "rolling shutter camera, read out time per line: " << TR);
    }
}
