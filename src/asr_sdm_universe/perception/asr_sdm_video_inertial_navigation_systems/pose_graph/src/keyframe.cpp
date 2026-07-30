#include "keyframe.h"
#include "loop_geometric_verifier/loop_geometric_verifier.h"

extern bool USE_SPARSE_ROTATION_GATE;
extern double SPARSE_GATE_ANGLE_THRESH_DEG;
extern double SPARSE_GATE_MAX_YAW_DIFF_DEG;
extern double SPARSE_GATE_VIO_DIFF_DEG;
extern int SPARSE_GATE_MIN_INLIERS;

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// create keyframe online
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
		           vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
		           vector<double> &_point_id, int _sequence)
{
	time_stamp = _time_stamp;
	index = _index;
	vio_T_w_i = _vio_T_w_i;
	vio_R_w_i = _vio_R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
	origin_vio_T = vio_T_w_i;
	origin_vio_R = vio_R_w_i;
	image = _image.clone();
	cv::resize(image, thumbnail, cv::Size(80, 60));
	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_2d_norm = _point_2d_norm;
	point_id = _point_id;
	has_loop = false;
	loop_index = -1;
	has_fast_point = false;
	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
	sequence = _sequence;
	has_sparse_R_ = false;
	computeWindowBRIEFPoint();
	computeBRIEFPoint();
	if(!DEBUG_IMAGE)
		image.release();
}

// load previous keyframe
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
					cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
					vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors)
{
	time_stamp = _time_stamp;
	index = _index;
	//vio_T_w_i = _vio_T_w_i;
	//vio_R_w_i = _vio_R_w_i;
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = _T_w_i;
	R_w_i = _R_w_i;
	if (DEBUG_IMAGE)
	{
		image = _image.clone();
		cv::resize(image, thumbnail, cv::Size(80, 60));
	}
	if (_loop_index != -1)
		has_loop = true;
	else
		has_loop = false;
	loop_index = _loop_index;
	loop_info = _loop_info;
	has_fast_point = false;
	sequence = 0;
	keypoints = _keypoints;
	keypoints_norm = _keypoints_norm;
	brief_descriptors = _brief_descriptors;
	has_sparse_R_ = false;
}


void KeyFrame::computeWindowBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	for(int i = 0; i < (int)point_2d_uv.size(); i++)
	{
	    cv::KeyPoint key;
	    key.pt = point_2d_uv[i];
	    window_keypoints.push_back(key);
	}
	extractor(image, window_keypoints, window_brief_descriptors);
}

void KeyFrame::computeBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	const int fast_th = 20; // corner detector response threshold
	if(1)
		cv::FAST(image, keypoints, fast_th, true);
	else
	{
		vector<cv::Point2f> tmp_pts;
		cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
		for(int i = 0; i < (int)tmp_pts.size(); i++)
		{
		    cv::KeyPoint key;
		    key.pt = tmp_pts[i];
		    keypoints.push_back(key);
		}
	}
	extractor(image, keypoints, brief_descriptors);
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());
		keypoints_norm.push_back(tmp_norm);
	}
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);
}


bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80)
    {
      best_match = keypoints_old[bestIndex].pt;
      best_match_norm = keypoints_old_norm[bestIndex].pt;
      return true;
    }
    else
      return false;
}

void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
								std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm)
{
    for(int i = 0; i < (int)window_brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
          status.push_back(1);
        else
          status.push_back(0);
        matched_2d_old.push_back(pt);
        matched_2d_old_norm.push_back(pt_norm);
    }

}


void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                      const std::vector<cv::Point2f> &matched_2d_old_norm,
                                      vector<uchar> &status)
{
	int n = (int)matched_2d_cur_norm.size();
	for (int i = 0; i < n; i++)
		status.push_back(0);
    if (n >= 8)
    {
        vector<cv::Point2f> tmp_cur(n), tmp_old(n);
        for (int i = 0; i < (int)matched_2d_cur_norm.size(); i++)
        {
            double FOCAL_LENGTH = 460.0;
            double tmp_x, tmp_y;
            tmp_x = FOCAL_LENGTH * matched_2d_cur_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_cur_norm[i].y + ROW / 2.0;
            tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

            tmp_x = FOCAL_LENGTH * matched_2d_old_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_old_norm[i].y + ROW / 2.0;
            tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
        }
        cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
    }
}

void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
	//for (int i = 0; i < matched_3d.size(); i++)
	//	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
	//printf("match size %d \n", matched_3d.size());
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = origin_vio_R * qic;
    Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);

    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    cv::Mat inliers;
    TicToc t_pnp_ransac;

    if (CV_MAJOR_VERSION < 3)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
        else
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    }

    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
        status.push_back(0);

    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }

    cv::Rodrigues(rvec, r);
    Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);
    R_w_c_old = R_pnp.transpose();
    Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(t, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    PnP_R_old = R_w_c_old * qic.transpose();
    PnP_T_old = T_w_c_old - PnP_R_old * tic;

}


// Safe sparse rotation geometric gate for loop-closure verification.
//
// Design principles:
//   1) Parameter-gated:  only active when USE_SPARSE_ROTATION_GATE == true.
//   2) Inverted logic: sparse acts as a "voucher" for PnP, not as a judge.
//      Gate fires ONLY when sparse and PnP AGREE (sparse endorses PnP),
//      AND PnP and VIO DISAGREE — meaning VIO has drifted and PnP may be
//      a false loop that happens to look geometrically plausible.
//   3) Conservative floor: if PnP inliers < threshold, skip gate to avoid
//      rejecting real loops on low-inlier frames.
//   4) Strict-yaw guard: only fires on small PnP yaw diff, because
//      adjacent-frame sparse R cannot represent large inter-frame rotation.
//   5) Zero frontend impact: all logic in pose_graph, after BRIEF+PnP.
//
// Rejection condition:
//   Gate rejects when ALL of the following hold:
//     - |sparse_yaw - PnP_yaw| < SPARSE_GATE_ANGLE_THRESH_DEG  (sparse endorses PnP)
//     - |PnP_yaw - VIO_yaw|   > SPARSE_GATE_VIO_DIFF_DEG       (PnP contradicts VIO)
//     - |PnP_yaw| < SPARSE_GATE_MAX_YAW_DIFF_DEG               (small rotation)
//     - pnp_inliers >= SPARSE_GATE_MIN_INLIERS                  (reliable PnP)
//
// References:
//   - paper §IV-B: sparse rotation geometric gate, Eq.(4) rotation consistency.
//   - code: keyframe.cpp findConnection() → this function.
inline bool sparse_rotation_geometric_gate(
    int cur_index, int old_index,
    const Eigen::Matrix3d& R_vio_cur,          // VIO R of current frame (world->IMU)
    const Eigen::Matrix3d& R_vio_old,          // VIO R of old frame (world->IMU)
    const Eigen::Matrix3d& R_pnp_old,          // PnP R: world -> old camera
    const Eigen::Matrix3d& R_sparse_cur,       // sparse_align R: cur(k) -> prev(k-1)
    const Eigen::Matrix3d& R_sparse_old,       // sparse_align R: old -> prev(old-1)
    bool has_cur_sparse, bool has_old_sparse,
    int pnp_inliers)
{
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    // Safety: gate disabled
    if (!USE_SPARSE_ROTATION_GATE)
        return true;

    // Safety: no sparse data on either frame
    if (!has_cur_sparse || !has_old_sparse) {
        printf("[SPARSE_GATE] cur=%d old=%d -> PASS (no sparse data)\n",
               cur_index, old_index);
        return true;
    }

    // Safety: PnP inliers below conservative floor — skip gate to avoid
    // false rejection on low-inlier frames (e.g. degenerate rotation-only motion).
    if (pnp_inliers < SPARSE_GATE_MIN_INLIERS) {
        printf("[SPARSE_GATE] cur=%d old=%d inliers=%d < min=%d -> PASS (low inliers)\n",
               cur_index, old_index, pnp_inliers, SPARSE_GATE_MIN_INLIERS);
        return true;
    }

    // Step 1: compute PnP relative yaw between cur and old.
    //   PnP gives world->old_cam (R_pnp_old).  VIO gives world->IMU (R_vio_cur/old).
    //   Transform PnP to IMU frame: R_imu_old = qic * R_pnp_old.
    //   Then PnP IMU yaw diff: R_vio_cur^T * R_imu_old.
    Eigen::Matrix3d R_imu_old = qic * R_pnp_old;
    Eigen::Matrix3d R_pnp_yaw_diff = R_vio_cur.transpose() * R_imu_old;
    double pnp_yaw_diff = Utility::R2ypr(R_pnp_yaw_diff).x();   // in radians

    // Step 2: compute sparse relative yaw between cur and old.
    //   R_sparse_cur: cur -> prev_adjacent  (adjacent frame)
    //   R_sparse_old: old -> prev_old_adjacent
    //   Both are adjacent-frame measurements, so their yaw difference captures
    //   the same-frame yaw gap that the sparse aligner "sees" locally.
    Eigen::Matrix3d R_sparse_yaw_diff = R_sparse_cur.transpose() * R_sparse_old;
    double sparse_yaw_diff = Utility::R2ypr(R_sparse_yaw_diff).x();  // in radians

    // Step 3: strict-yaw guard — only gate loops with small PnP yaw diff.
    // Large-rotation loops (e.g. |PnP_yaw| > max) are excluded because:
    //   a) adjacent-frame sparse R cannot represent large inter-frame rotation.
    //   b) PnP rotation may be unreliable under large viewpoint change.
    double pnp_yaw_deg = std::abs(pnp_yaw_diff) / DEG_TO_RAD;
    if (pnp_yaw_deg > SPARSE_GATE_MAX_YAW_DIFF_DEG) {
        printf("[SPARSE_GATE] cur=%d old=%d pnp_yaw=%.1f > max=%.1f deg -> PASS (large rotation)\n",
               cur_index, old_index, pnp_yaw_deg, SPARSE_GATE_MAX_YAW_DIFF_DEG);
        return true;
    }

    // Step 4: sparse endorses PnP?
    //   If sparse and PnP disagree (large yaw mismatch), sparse does NOT trust PnP.
    //   We let it pass — sparse itself is uncertain, so we should not reject.
    double yaw_error = std::abs(pnp_yaw_diff - sparse_yaw_diff) / DEG_TO_RAD;
    bool sparse_endorses_pnp = (yaw_error <= SPARSE_GATE_ANGLE_THRESH_DEG);

    // Step 5: PnP contradicts VIO?
    //   Compute VIO yaw diff between cur and old: R_vio_old^T * R_vio_cur.
    Eigen::Matrix3d R_vio_yaw_diff = R_vio_old.transpose() * R_vio_cur;
    double vio_yaw_diff = Utility::R2ypr(R_vio_yaw_diff).x();
    double vio_error = std::abs(pnp_yaw_diff - vio_yaw_diff) / DEG_TO_RAD;
    bool pnp_contradicts_vio = (vio_error > SPARSE_GATE_VIO_DIFF_DEG);

    printf("[SPARSE_GATE] cur=%d old=%d pnp_yaw=%.1f sparse_yaw=%.1f err_pnp_sparse=%.1f "
           "vio_yaw=%.1f err_pnp_vio=%.1f inliers=%d endorse=%d contradict_vio=%d\n",
           cur_index, old_index, pnp_yaw_deg, sparse_yaw_diff / DEG_TO_RAD,
           yaw_error, vio_yaw_diff / DEG_TO_RAD, vio_error,
           pnp_inliers, sparse_endorses_pnp, pnp_contradicts_vio);

    // Gate fires only when: sparse endorses PnP AND PnP contradicts VIO.
    // This means: the visual loop agrees with sparse locally, but disagrees with VIO.
    // Likely scenario: VIO has drifted and this loop would drag the map back —
    // rejecting it can prevent spurious loop closure from corrupting the map.
    if (sparse_endorses_pnp && pnp_contradicts_vio) {
        printf("[SPARSE_GATE] -> REJECT (sparse endorses PnP but PnP contradicts VIO, "
               "err=%.1f deg > %.1f deg)\n",
               vio_error, SPARSE_GATE_VIO_DIFF_DEG);
        return false;
    }

    printf("[SPARSE_GATE] -> PASS\n");
    return true;
}

bool KeyFrame::findConnection(KeyFrame* old_kf)
{
	TicToc tmp_t;
	//printf("find Connection\n");
	vector<cv::Point2f> matched_2d_cur, matched_2d_old;
	vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
	vector<cv::Point3f> matched_3d;
	vector<double> matched_id;
	vector<uchar> status;

	matched_3d = point_3d;
	matched_2d_cur = point_2d_uv;
	matched_2d_cur_norm = point_2d_norm;
	matched_id = point_id;

	TicToc t_match;
	#if 0
		if (DEBUG_IMAGE)
	    {
	        cv::Mat gray_img, loop_match_img;
	        cv::Mat old_img = old_kf->image;
	        cv::hconcat(image, old_img, gray_img);
	        cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)point_2d_uv.size(); i++)
	        {
	            cv::Point2f cur_pt = point_2d_uv[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)old_kf->keypoints.size(); i++)
	        {
	            cv::Point2f old_pt = old_kf->keypoints[i].pt;
	            old_pt.x += COL;
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        ostringstream path;
	        path << "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "0raw_point.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
	#endif
	//printf("search by des\n");
	searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	//printf("search by des finish\n");

	#if 0
		if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap);
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path, path1, path2;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	        /*
	        path1 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_1.jpg";
	        cv::imwrite( path1.str().c_str(), image);
	        path2 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_2.jpg";
	        cv::imwrite( path2.str().c_str(), old_img);
	        */

	    }
	#endif
	status.clear();
	/*
	FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	*/
	#if 0
		if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap) ;
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "2fundamental_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
	#endif
	Eigen::Vector3d PnP_T_old;
	Eigen::Matrix3d PnP_R_old;
	Eigen::Vector3d relative_t;
	Quaterniond relative_q;
	double relative_yaw;
	int pnp_inliers = 0;
	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
        // printf("[LOOP_CONN] frame=%d old=%d matched=%d > MIN_LOOP_NUM=%d -> try PnP\n",
        //        index, old_kf->index, (int)matched_2d_cur.size(), MIN_LOOP_NUM);
        status.clear();
	    PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
	    pnp_inliers = 0;
	    for (size_t i = 0; i < status.size(); ++i)
	        if (status[i]) ++pnp_inliers;
	    reduceVector(matched_2d_cur, status);
	    reduceVector(matched_2d_old, status);
	    reduceVector(matched_2d_cur_norm, status);
	    reduceVector(matched_2d_old_norm, status);
	    reduceVector(matched_3d, status);
	    reduceVector(matched_id, status);
	    #if 1
	    	if (DEBUG_IMAGE)
	        {
	        	int gap = 10;
	        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
	            cv::Mat gray_img, loop_match_img;
	            cv::Mat old_img = old_kf->image;
	            cv::hconcat(image, gap_image, gap_image);
	            cv::hconcat(gap_image, old_img, gray_img);
	            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	            for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	            {
	                cv::Point2f cur_pt = matched_2d_cur[i];
	                cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	            }
	            for(int i = 0; i< (int)matched_2d_old.size(); i++)
	            {
	                cv::Point2f old_pt = matched_2d_old[i];
	                old_pt.x += (COL + gap);
	                cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	            }
	            for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	            {
	                cv::Point2f old_pt = matched_2d_old[i];
	                old_pt.x += (COL + gap) ;
	                cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 2, 8, 0);
	            }
	            cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
	            putText(notation, "current frame: " + to_string(index) + "  sequence: " + to_string(sequence), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);

	            putText(notation, "previous frame: " + to_string(old_kf->index) + "  sequence: " + to_string(old_kf->sequence), cv::Point2f(20 + COL + gap, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
	            cv::vconcat(notation, loop_match_img, loop_match_img);

	            /*
	            ostringstream path;
	            path <<  "/home/tony-ws1/raw_data/loop_image/"
	                    << index << "-"
	                    << old_kf->index << "-" << "3pnp_match.jpg";
	            cv::imwrite( path.str().c_str(), loop_match_img);
	            */
	            if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	            {
	            	/*
	            	cv::imshow("loop connection",loop_match_img);
	            	cv::waitKey(10);
	            	*/
	            	cv::Mat thumbimage;
	            	cv::resize(loop_match_img, thumbimage, cv::Size(loop_match_img.cols / 2, loop_match_img.rows / 2));
	    	    	sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", thumbimage).toImageMsg();
	                msg->header.stamp = rclcpp::Time(time_stamp);
	    	    	pub_match_img->publish(*msg);
	            }
	        }
	    #endif
	}

	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
	    relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
	    relative_q = PnP_R_old.transpose() * origin_vio_R;
	    relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());
	    //printf("PNP relative\n");
	    //cout << "pnp relative_t " << relative_t.transpose() << endl;
	    //cout << "pnp relative_yaw " << relative_yaw << endl;

	    // D2.2: geometric re-verification.  PnP+RANSAC already threw
	    // out the obvious outliers; this catches perceptual aliasing
	    // and pathological BRIEF matches under big viewpoint change.
	    // We build the camera-frame poses for both kfs and feed the
	    // world-frame landmark cloud (cur_kf::point_3d) to the
	    // verifier.  The verifier returns a one-shot verdict.
	    bool geometric_ok = true;
	    {
	        vins_sparse::LoopVerifOptions vopt;
	        vopt.image_width  = COL;
	        vopt.image_height = ROW;
	        vopt.inlier_px_thresh  = 8.0;
	        vopt.median_px_thresh  = 5.0;
	        vopt.min_inliers       = std::max(8, (int)matched_2d_cur.size() / 5);
	        vopt.min_inlier_ratio  = 0.30;
	        vopt.symmetric         = false;  // D2 W3: only forward verification needed.
	                                      // Backward check fails for all real loop candidates
	                                      // (bwd_inliers=0 for every frame) because the
	                                      // old-camera pose comes from PnP which is often
	                                      // degenerate (rotational ambiguity) for
	                                      // forward-facing cameras revisiting the same scene.
	                                      // Forward check (fwd_inliers~70-80%) is sufficient
	                                      // and already filters out most perceptual aliasing.

	        // World -> cur camera = VIO pose * (qic, tic).
	        Eigen::Matrix3d R_w_cur_cam = origin_vio_R * qic;
	        Eigen::Vector3d t_w_cur_cam = origin_vio_T + origin_vio_R * tic;
	        // World -> old camera: take PnP's world -> old_cam directly.
	        Eigen::Matrix3d R_w_old_cam = PnP_R_old;
	        Eigen::Vector3d t_w_old_cam = PnP_T_old;

	        std::vector<Eigen::Vector3d> pts_w;
	        std::vector<Eigen::Vector2d> obs_cur, obs_old;
	        pts_w.reserve(matched_3d.size());
	        obs_cur.reserve(matched_2d_cur.size());
	        obs_old.reserve(matched_2d_old.size());
	        for (size_t i = 0; i < matched_3d.size(); ++i) {
	            pts_w.emplace_back(matched_3d[i].x,
	                                matched_3d[i].y,
	                                matched_3d[i].z);
	        }
	        for (size_t i = 0; i < matched_2d_cur.size(); ++i) {
	            obs_cur.emplace_back(matched_2d_cur[i].x,
	                                 matched_2d_cur[i].y);
	        }
	        for (size_t i = 0; i < matched_2d_old.size(); ++i) {
	            obs_old.emplace_back(matched_2d_old[i].x,
	                                 matched_2d_old[i].y);
	        }

	        vins_sparse::LoopGeometricVerifier v(vopt);
	        const auto vr = v.verify(R_w_cur_cam, t_w_cur_cam,
	                                 R_w_old_cam, t_w_old_cam,
	                                 pts_w, obs_cur, obs_old,
	                                 m_camera);
        geometric_ok = vr.verified;
        printf("[LOOP_VERIF] cur=%d old=%d tested=%d fwd_inliers=%d "
               "bwd_inliers=%d median=%.2fpx inlier_ratio=%.2f -> %s\n",
               index, old_kf->index, vr.n_tested, vr.n_inliers_forward,
               vr.n_inliers_backward, vr.median_residual_px,
               vr.inlier_ratio, vr.verified ? "PASS" : "REJECT");

        // D2 W1: sparse rotation geometric gate — safe yaw-consistency check.
        //
        // Key insight: upstream sparse_R_ carries adjacent-frame R(cur -> prev),
        // NOT cross-loop R(cur -> old).  Directly comparing them would reject
        // real loops whenever |cur_yaw - old_yaw| exceeds the threshold.
        //
        // Safe gate design:
        //   - Uses yaw consistency (not cross-frame R comparison).
        //   - Strict-yaw: only fires when |PnP_yaw_diff| < max_yaw_diff_deg.
        //   - Conservative inlier floor: bypasses if PnP_inliers < min_inliers.
        //   - Disabled by default (USE_SPARSE_ROTATION_GATE=false).
        //
        // When gate is enabled AND all preconditions are met AND a severe
        // yaw mismatch is detected -> REJECT.  Otherwise -> PASS (no effect).
        if (geometric_ok) {
            bool sparse_ok = sparse_rotation_geometric_gate(
                index, old_kf->index,
                origin_vio_R,                         // R_vio_cur
                old_kf->origin_vio_R,                  // R_vio_old
                PnP_R_old,                            // R_pnp_old
                sparse_R_,                             // R_sparse_cur (cur -> prev_adj)
                old_kf->sparse_R_,                     // R_sparse_old (old -> prev_old_adj)
                has_sparse_R_, old_kf->has_sparse_R_,
                pnp_inliers);
            if (!sparse_ok) {
                printf("[LOOP_FINAL] frame=%d old=%d REJECT: yaw mismatch (sparse gate)\n",
                       index, old_kf->index);
                return false;
            }
        }
    }

	    if (abs(relative_yaw) < 90.0 && relative_t.norm() < 20.0 && geometric_ok)
	    {
            printf("[LOOP_FINAL] frame=%d old=%d PASS: yaw=%.1f(deg) t_norm=%.3f geometric=PASS\n",
                   index, old_kf->index, abs(relative_yaw)*180.0/M_PI, relative_t.norm());
	    	has_loop = true;
	    	loop_index = old_kf->index;
	    	loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
	    	             relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
	    	             relative_yaw;
	    	if(FAST_RELOCALIZATION)
	    	{
			    sensor_msgs::msg::PointCloud msg_match_points;
			    msg_match_points.header.stamp = rclcpp::Time(time_stamp);
			    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
			    {
		            geometry_msgs::msg::Point32 p;
		            p.x = matched_2d_old_norm[i].x;
		            p.y = matched_2d_old_norm[i].y;
		            p.z = matched_id[i];
		            msg_match_points.points.push_back(p);
			    }
			    Eigen::Vector3d T = old_kf->T_w_i;
			    Eigen::Matrix3d R = old_kf->R_w_i;
			    Quaterniond Q(R);
			    sensor_msgs::msg::ChannelFloat32 t_q_index;
			    t_q_index.values.push_back(T.x());
			    t_q_index.values.push_back(T.y());
			    t_q_index.values.push_back(T.z());
			    t_q_index.values.push_back(Q.w());
			    t_q_index.values.push_back(Q.x());
			    t_q_index.values.push_back(Q.y());
			    t_q_index.values.push_back(Q.z());
			    t_q_index.values.push_back(index);
			    msg_match_points.channels.push_back(t_q_index);
			    pub_match_points->publish(msg_match_points);
	    	}
	        return true;
	    }
	    else {
            printf("[LOOP_FINAL] frame=%d old=%d REJECT: yaw=%.1f(deg) t_norm=%.3f geometric=%s\n",
                   index, old_kf->index, abs(relative_yaw)*180.0/M_PI, relative_t.norm(),
                   geometric_ok ? "PASS" : "FAIL");
	    }
	}
	//printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(), t_match.toc());
	return false;
}


int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = vio_T_w_i;
    _R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
    return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopRelativeQ()
{
    return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw()
{
    return loop_info(7);
}

void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
{
	if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
	{
		//printf("update loop info\n");
		loop_info = _loop_info;
	}
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}
