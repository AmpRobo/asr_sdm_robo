#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <cmath>
#include <limits>

#include "feature_tracker.h"
#include "imu_preintegrate.h"
#include "td_pre_calibrator/td_pre_calibrator.h"
#include <asr_sdm_perception_msgs/msg/sparse_rot.hpp>

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstPtr> img_buf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;
rclcpp::Publisher<asr_sdm_perception_msgs::msg::SparseRot>::SharedPtr pub_sparse_rot;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_td_estimate;   // D2.2
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

FeatureTracker trackerData[NUM_OF_CAM];
vins_sparse::TdPreCalibrator td_calib[NUM_OF_CAM];   // D2.2
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

// D2.2: gyro history (1 s rolling) shared across cameras.
std::vector<vins_sparse::GyroSample> g_gyro_buf;
constexpr double kGyroWindowSec = 1.0;

static void pruneGyroBufLocked(double t_min) {
    size_t k = 0;
    while (k < g_gyro_buf.size() && g_gyro_buf[k].t < t_min) ++k;
    if (k > 0) g_gyro_buf.erase(g_gyro_buf.begin(),
                                g_gyro_buf.begin() + (long)k);
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double t = imu_msg->header.stamp.sec +
               imu_msg->header.stamp.nanosec * 1e-9;
    Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z);
    Eigen::Vector3d accel(imu_msg->linear_acceleration.x,
                          imu_msg->linear_acceleration.y,
                          imu_msg->linear_acceleration.z);
    for (int i = 0; i < NUM_OF_CAM; ++i) {
        trackerData[i].addImuSample(t, gyro, accel);
        // Keep ~1 second of IMU history for sparse-align priors.
        trackerData[i].pruneImuBuffer(t - 1.0);
    }
    // D2.2: keep a *raw* gyro history for the td pre-calibrator.
    g_gyro_buf.push_back({t, gyro.x(), gyro.y(), gyro.z()});
    pruneGyroBufLocked(t - kGyroWindowSec);
}

void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
        last_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
        return;
    }

    // detect unstable camera stream
    if (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - last_image_time > 1.0 || img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) < last_image_time)
    {
        RCUTILS_LOG_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }

    last_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    // cv::imshow("img", show_img);
    // cv::waitKey(0);
    TicToc t_r;

    // Build per-frame IMU rotation priors before readImage so that
    // sparse-align has a warm start between consecutive frames.
    {
        double cur_t = img_msg->header.stamp.sec +
                       img_msg->header.stamp.nanosec * 1e-9;
        if (last_image_time > 0.0 && USE_SPARSE_ALIGN) {
            for (int i = 0; i < NUM_OF_CAM; ++i) {
                Eigen::Matrix3d R_prior = trackerData[i].imu_preint_
                    .integrateRotation(last_image_time, cur_t);
                trackerData[i].setImuRotationPrior(R_prior);
            }
        }
        // D2.2: stage gyro samples inside the calibrator's per-frame
        // window.  We do this *before* readImage so the calibrator's
        // solve() can use the same buffer the align used.
        if (USE_SPARSE_ALIGN && USE_TD_PRE_CALIB && last_image_time > 0.0) {
            for (int i = 0; i < NUM_OF_CAM; ++i) {
                if (!td_calib[i].beginFrame(last_image_time, cur_t)) {
                    continue;   // dt too small
                }
                for (const auto& s : g_gyro_buf) {
                    if (s.t < last_image_time) continue;
                    if (s.t > cur_t) break;
                    td_calib[i].addGyro(s.t, s.wx, s.wy, s.wz);
                }
            }
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCUTILS_LOG_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9));
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    // D2.2: solve for td using the pure-vision align result.  We do this
    // after readImage() so we can pull `last_align_vision_res_` for each
    // camera.  Solve once per camera, then publish the median / first.
    if (USE_SPARSE_ALIGN && USE_TD_PRE_CALIB && pub_td_estimate) {
        double last_td = std::numeric_limits<double>::quiet_NaN();
        double last_cost = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < NUM_OF_CAM; ++i) {
            if (!trackerData[i].has_align_vision_res_) continue;
            const auto& v = trackerData[i].last_align_vision_res_;
            if (!v.success || v.n_meas < 30) continue;
            td_calib[i].setVisionRotation(v.R_k_kminus1);
            double cost = 0.0;
            double td = td_calib[i].solve(&cost);
            if (std::isfinite(td)) {
                last_td   = td;
                last_cost = cost;
                if (td_calib[i].numSolves() % 20 == 0) {
                    RCUTILS_LOG_INFO(
                        "[TD_PRE_CALIB] cam=%d solve#%d td=%.4fms cost=%.4fdeg mean_td=%.4fms",
                        i, td_calib[i].numSolves(),
                        td * 1000.0, cost * 180.0 / M_PI,
                        td_calib[i].meanTd() * 1000.0);
                }
            }
        }
        if (std::isfinite(last_td)) {
            std_msgs::msg::Float64 msg;
            msg.data = last_td;
            pub_td_estimate->publish(msg);
        }
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        RCUTILS_LOG_DEBUG("publish %f, at %f", feature_points->header.stamp.sec+feature_points->header.stamp.nanosec * (1e-9), rclcpp::Clock().now().nanoseconds()*(1e-9));
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img->publish(*feature_points);

        // D2.1: publish sparse-align rotation result for the SPARSE1 pipeline.
        // Skips frames where alignment failed or had too few measurements.
        // Payload: asr_sdm_perception_msgs::SparseRot (row-major rot_mat[9], success, chi2, n_meas, iterations, stamp_sec)
        if (USE_SPARSE_ALIGN && pub_sparse_rot != nullptr) {
            for (int i = 0; i < NUM_OF_CAM; ++i) {
                const auto& res = trackerData[i].last_align_res_;
                // Gate: success == true AND n_meas >= 30
                if (!res.success || res.n_meas < 30) continue;

                asr_sdm_perception_msgs::msg::SparseRot msg;
                msg.header = img_msg->header;
                msg.header.frame_id = "world";
                const Eigen::Matrix3d& R = res.R_k_kminus1;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        msg.rot_mat[r * 3 + c] = R(r, c);
                msg.success     = res.success;
                msg.chi2        = static_cast<float>(res.final_chi2);
                msg.n_meas      = res.n_meas;
                msg.iterations   = res.iterations;
                msg.stamp_sec   = img_msg->header.stamp.sec +
                                  img_msg->header.stamp.nanosec * 1e-9;
                pub_sparse_rot->publish(msg);
            }
        }

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match->publish(*(ptr->toImageMsg()));
        }
    }
    // RCUTILS_LOG_INFO("whole feature tracker processing costs: %fms", t_r.toc());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("feature_tracker");

    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCUTILS_LOG_INFO("load mask fail");
            }
            else
                RCUTILS_LOG_INFO("load mask success");
        }
    }

    auto sub_img = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img_callback);

    if (USE_SPARSE_ALIGN) {
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, rclcpp::SensorDataQoS(), imu_callback);
        RCLCPP_INFO(n->get_logger(),
                    "Sparse image alignment enabled, subscribing to IMU %s",
                    IMU_TOPIC.c_str());
    }

    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("feature_img",1000);
    pub_restart = n->create_publisher<std_msgs::msg::Bool>("restart",1000);
    pub_sparse_rot = n->create_publisher<asr_sdm_perception_msgs::msg::SparseRot>("sparse_rot", 1000);
    if (USE_SPARSE_ALIGN && USE_TD_PRE_CALIB) {
        pub_td_estimate = n->create_publisher<std_msgs::msg::Float64>(
            "td_estimate", 100);
        RCLCPP_INFO(n->get_logger(),
                    "td pre-calibrator enabled, publishing /td_estimate");
    }
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    rclcpp::spin(n);
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?