#include "feature_tracker.h"

#include "half_sample.h"
#include "td_pre_calibrator/td_pre_calibrator.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::addImuSample(double t,
                                  const Eigen::Vector3d& gyro,
                                  const Eigen::Vector3d& accel)
{
    vins_sparse::ImuSample s;
    s.t = t;
    s.gyro = gyro;
    s.accel = accel;
    imu_preint_.add(s);
}

void FeatureTracker::setImuRotationPrior(const Eigen::Matrix3d& R_k_km1)
{
    R_prev_cur_ = R_k_km1;
    have_imu_prior_ = true;
}

void FeatureTracker::pruneImuBuffer(double t_min)
{
    imu_preint_.pruneOld(t_min);
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        RCUTILS_LOG_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    // ------------------------------------------------------------------
    // Sparse image alignment (SVO-style semi-direct pose estimation).
    //
    // Goal: provide a *frame-to-frame* rotation/translation estimate
    // from the photometric residual, then use it to shrink the KLT
    // search window. The KLT start points stay where they are (in the
    // previous frame) -- the SVO trick is purely a tighter pyramid +
    // window, not a KLT start-point transformation, because OpenCV's
    // calcOpticalFlowPyrLK cannot accept a transformed starting pixel
    // that lives in the *current* frame (it would converge to itself
    // when searching cur_img for cur_img).
    //
    // This is the SVO trick that lets the front end run an order of
    // magnitude cheaper than full-pyramid large-window KLT: a 5x5
    // single-level KLT run on a photometrically-predicted image costs
    // ~3% of a 21x21 three-level KLT.
    // ------------------------------------------------------------------
    cv::Size klt_win(21, 21);
    int klt_level = 3;
    bool used_sparse = false;

    if (cur_pts.size() > 0 && USE_SPARSE_ALIGN && have_imu_prior_ &&
        !prev_img.empty())
    {
        TicToc t_a;
        const int n_levels = SPARSE_ALIGN_MAX_LEVEL + 1;

        // Reuse the previous frame's pyramid from the last call (we
        // saved it as prev_pyr_ at the end of the previous readImage).
        // Only build the pyramid for the *current* image.
        prev_pyr_ = saved_prev_pyr_;  // carry over from prior frame
        if (static_cast<int>(prev_pyr_.size()) != n_levels) {
            // First frame or n_levels changed; rebuild both.
            vins_sparse::buildHalfSamplePyramid(prev_img, n_levels, prev_pyr_);
        }
        vins_sparse::buildHalfSamplePyramid(img, n_levels, cur_pyr_);

        // Build bearings for cur_pts in the previous frame.
        std::vector<Eigen::Vector3d> ref_bearings;
        ref_bearings.reserve(cur_pts.size());
        for (size_t i = 0; i < cur_pts.size(); ++i) {
            Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            double nrm = b.norm();
            if (nrm < 1e-9) ref_bearings.emplace_back(0.0, 0.0, 1.0);
            else            ref_bearings.push_back(b / nrm);
        }

        vins_sparse::SparseAlignOptions opt;
        opt.patch_size      = SPARSE_ALIGN_PATCH_SIZE;
        opt.min_level       = SPARSE_ALIGN_MIN_LEVEL;
        opt.max_level       = SPARSE_ALIGN_MAX_LEVEL;
        opt.max_iter        = SPARSE_ALIGN_MAX_ITER;
        opt.lambda_rot      = SPARSE_ALIGN_LAMBDA_ROT;
        opt.lambda_trans    = SPARSE_ALIGN_LAMBDA_TRANS;
        opt.chi2_thresh     = SPARSE_ALIGN_CHI2_THRESH;
        opt.min_features    = SPARSE_ALIGN_MIN_FEATURES;
        opt.min_iter_for_ok = SPARSE_ALIGN_MIN_ITER_FOR_OK;
        opt.fx              = FX;
        opt.fy              = FY;
        opt.cx              = CX;
        opt.cy              = CY;
        opt.image_width     = COL;
        opt.image_height    = ROW;

        // ----- (a) WITH IMU prior (the production path) -----
        auto align_res = vins_sparse::sparseAlign(
            prev_pyr_, cur_pyr_, cur_pts, ref_bearings,
            R_prev_cur_, Eigen::Vector3d::Zero(), opt);

        // D2.1: stash for the node to publish.
        last_align_res_ = align_res;

        // ----- (b) D2.2: PURE-VISION re-alignment for the td pre-calib.
        //  Same pyr, same ref_pts, but lambda_rot=0 and R_prior=Identity.
        //  The result R_vision is the rotation the camera would estimate
        //  if the IMU were not biased by a wrong td.  We run this only
        //  when the user enabled the calibrator (default off; cheap if on).
        if (USE_TD_PRE_CALIB) {
            vins_sparse::SparseAlignOptions opt_v = opt;
            opt_v.lambda_rot   = 0.0;
            opt_v.lambda_trans = 0.0;
            last_align_vision_res_ = vins_sparse::sparseAlign(
                prev_pyr_, cur_pyr_, cur_pts, ref_bearings,
                Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), opt_v);
            has_align_vision_res_ = true;
        } else {
            has_align_vision_res_ = false;
        }

        // Stats logging.
        ++n_sparse_frames_;
        if (align_res.success) ++n_sparse_success_;
        n_sparse_meas_sum_   += align_res.n_meas;
        n_sparse_chi2_sum_   += align_res.final_chi2;
        n_sparse_time_sum_   += t_a.toc();
        n_klt_for_align_sum_ += (int)cur_pts.size();

        if ((++n_frames_seen_ % 10) == 0) {
            double mean_chi2  = n_sparse_meas_sum_ > 0
                                ? n_sparse_chi2_sum_ / n_sparse_frames_ : 0.0;
            double mean_nmeas = n_sparse_frames_ > 0
                                ? (double)n_sparse_meas_sum_ / n_sparse_frames_ : 0.0;
            double mean_time  = n_sparse_frames_ > 0
                                ? n_sparse_time_sum_ / n_sparse_frames_ : 0.0;
            double succ_rate  = n_sparse_frames_ > 0
                                ? 100.0 * n_sparse_success_ / n_sparse_frames_ : 0.0;
            double mean_klt   = n_sparse_frames_ > 0
                                ? (double)n_klt_for_align_sum_ / n_sparse_frames_ : 0.0;
            RCUTILS_LOG_INFO(
                "[SPARSE_STATS] frames=%d/%d success_rate=%.1f%% "
                "mean_nmeas=%.0f mean_chi2=%.2f mean_time=%.2fms mean_klt=%.1f",
                n_sparse_frames_, n_frames_seen_, succ_rate,
                mean_nmeas, mean_chi2, mean_time, mean_klt);
        }

        // SVO-style window/level decision: the better the photometric
        // fit, the smaller the KLT search window can be. We use the
        // KLT start points unchanged (cur_pts in the previous frame's
        // pixel coordinates) but shrink the KLT pyramid + window.
        if (align_res.success && align_res.n_meas >= SPARSE_ALIGN_MIN_FEATURES) {
            const double chi2 = align_res.final_chi2;
            if (chi2 < 5.0) {
                klt_win   = cv::Size(5, 5);
                klt_level = 1;
            } else if (chi2 < 15.0) {
                klt_win   = cv::Size(9, 9);
                klt_level = 1;
            } else {
                klt_win   = cv::Size(15, 15);
                klt_level = 2;
            }
            used_sparse = true;
        }
    }

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        // KLT start points are always cur_pts (in the previous frame).
        // The window/level come from sparse_align's confidence when
        // available, else fall back to the original 21x21 / 3 levels.
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts,
                                  status, err, klt_win, klt_level);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        RCUTILS_LOG_DEBUG("temporal optical flow costs: %fms "
                          "(win=%dx%d lvls=%d sparse_prior=%d)",
                          t_o.toc(), klt_win.width, klt_win.height,
                          klt_level, (int)used_sparse);

        // Aggregate KLT cost for periodic logging.
        n_klt_calls_++;
        n_klt_cost_sum_ += t_o.toc();
        n_klt_w_  += klt_win.width;
        n_klt_l_  += klt_level;
        n_klt_sparse_prior_ += (int)used_sparse;
        if ((n_klt_calls_ % 50) == 0) {
            double mean_cost = n_klt_cost_sum_ / n_klt_calls_;
            double mean_w    = (double)n_klt_w_ / n_klt_calls_;
            double mean_l    = (double)n_klt_l_ / n_klt_calls_;
            double prior_pct = 100.0 * n_klt_sparse_prior_ / n_klt_calls_;
            RCUTILS_LOG_INFO(
                "[KLT_STATS] frames=%d mean_cost=%.2fms mean_win=%.1f "
                "mean_lvls=%.2f sparse_prior=%.1f%%",
                n_klt_calls_, mean_cost, mean_w, mean_l, prior_pct);
        }
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();        RCUTILS_LOG_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        RCUTILS_LOG_DEBUG("set mask costs %fms", t_m.toc());

        RCUTILS_LOG_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                RCUTILS_LOG_INFO("mask is empty ");
            if (mask.type() != CV_8UC1)
                RCUTILS_LOG_INFO("mask type wrong ");
            if (mask.size() != forw_img.size())
                RCUTILS_LOG_INFO("wrong size ");
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        RCUTILS_LOG_DEBUG("detect feature costs: %fms", t_t.toc());

        RCUTILS_LOG_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();

        RCUTILS_LOG_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    // Save the current image's pyramid as next frame's previous pyramid.
    saved_prev_pyr_ = cur_pyr_;
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        RCUTILS_LOG_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        // RCUTILS_LOG_INFO("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // RCUTILS_LOG_INFO("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    RCUTILS_LOG_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
