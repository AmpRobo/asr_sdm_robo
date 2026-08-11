#pragma once

// Time-offset (td) pre-calibration for the camera-IMU stream.
//
// Goal
// ----
//   VINS-style BA learns a scalar time offset `td` such that
//   `t_cam_physical = t_cam_stamp + td`. The default value is 0 because the
//   bag is assumed to be hardware-time-synced. When sync is imperfect the
//   projection factors in BA have a systematic bias and the BA can take
//   hundreds of iterations to converge (and often settles on a wrong local
//   minimum).
//
//   This module produces a *pre-calibrated* td before BA is ever run. It
//   exploits the fact that the front-end already runs
//   `vins_sparse::sparseAlign` twice'ish (once with the IMU prior for
//   R_prior=warm-start, and a theoretical no-prior version). When those two
//   rotations disagree, the disagreement scales with the local angular rate
//   `||ω||` and the unknown td.
//
// Model
// -----
//   Assume a constant angular velocity vector ω in the IMU frame between
//   (t_img_prev, t_img_cur). With sync error td, the IMU integration window
//   is shifted by td, so the integrated R_imu(td) is approximately
//       R_imu(td)  =  exp(ω · dt_imu)         with  dt_imu = (t_img_cur + td) - (t_img_prev + td) = dt_img
//   i.e. to first order in td the *magnitude* of the integrated rotation is
//   unchanged but the *axis* is rotated by (ω × ...). The actual error
//   geometry is non-trivial, so we solve the following 1-D problem
//   numerically:
//
//       minimize_td  || log( R_imu(td) * R_vision^T ) ||_2
//
//   where R_imu(td) is computed by replaying the gyro samples in the
//   interval [t_img_prev + td, t_img_cur + td].
//
//   R_vision comes from `sparse_img_align` with the IMU prior DISABLED
//   (lambda_rot=0). That pure-vision rotation is reference-quality as long
//   as parallax is reasonable.
//
// Usage
// -----
//   TdPreCalibrator c;
//   c.setGyroSamples(samples);            // all gyro samples from the bag
//   c.beginFrame(t_prev, t_cur, R_vision);
//   c.addGyro(t, wx, wy, wz);             // per-sample within the window
//   double td = c.solve();                // returns seconds; NaN on failure
//
// Threading
// ---------
//   Pure CPU, no ROS, no threading. Construct one instance per frame
//   pipeline (i.e. one per `FeatureTracker`); or one shared instance with a
//   mutex if the same calibrator is reused across cameras.

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vins_sparse {

struct GyroSample {
    double t;          // seconds, monotonic
    double wx, wy, wz; // rad/s, in IMU frame
};

struct TdPreCalibratorOptions {
    // Search range for td.  Symmetric around 0.  The published D435i
    // unsynced-drift is bounded by ~ +/- 5 ms in practice.
    double td_min        = -0.020;   // seconds
    double td_max        =  0.020;   // seconds
    int    n_samples     = 41;       // grid search resolution (20 steps per ms)
    double min_angle_deg = 0.05;     // ||ΔR|| in degrees; below this the frame
                                    // has too little rotation to be informative
    double min_dt        = 0.001;    // s; require at least 1 ms between frames
    int    max_iter      = 8;        // refinement iterations around the grid min
    double refine_window = 0.0005;   // s; half-width of the refinement window
};

class TdPreCalibrator {
  public:
    explicit TdPreCalibrator(const TdPreCalibratorOptions& opt = {});

    // Begin a new frame-pair measurement. `t_prev` and `t_cur` are the
    // *nominal* (un-shifted) image stamps.  After this call, push gyro
    // samples via addGyro() and then setVisionRotation() once the
    // pure-vision alignment is available, then call solve().
    // Returns false if the frame is unusable (dt too small).
    bool beginFrame(double t_prev, double t_cur);

    // Set the pure-vision (lambda_rot=0) sparse-align rotation for the
    // current frame.  Call after the front-end's second sparse-align
    // pass.  Has no effect if beginFrame() returned false.
    void setVisionRotation(const Eigen::Matrix3d& R_vision);

    // Push a gyro sample that falls inside the current frame's integration
    // window. Call this for every sample between [t_prev, t_cur].
    void addGyro(double t, double wx, double wy, double wz);

    // Solve for td. Cost: ||
    //     log( integrateRotation(t_prev+td, t_cur+td) * R_vision^T )
    // ||_2 over a 1-D grid then a Newton refinement.
    //
    // Returns the best td in seconds, or NaN if no usable frame was
    // accumulated.  Also writes the final cost in `*out_final_angle_rad`
    // (when non-null).
    double solve(double* out_final_angle_rad = nullptr);

    // Reset the per-frame gyro buffer (but keep options / accumulator
    // statistics).
    void resetFrame();

    // Accumulator across many solve() calls: useful for a rolling
    // estimate or for logging.  Successful solve() pushes (td, cost).
    int    numSolves()        const { return (int)samples_.size(); }
    double meanTd()           const;   // mean of all successful td
    double lastTd()           const { return last_td_; }

  private:
    TdPreCalibratorOptions opt_;

    // Current frame.
    bool                 have_frame_ = false;
    double               t_prev_     = 0.0;
    double               t_cur_      = 0.0;
    Eigen::Matrix3d      R_vision_   = Eigen::Matrix3d::Identity();
    std::vector<GyroSample> cur_gyro_;

    // History of successful solves.
    struct SolvedSample {
        double td;
        double cost;
    };
    std::vector<SolvedSample> samples_;
    double                    last_td_ = 0.0;

    // Integrate the gyro buffer between [t0, t1] (both already td-shifted
    // by the caller).  Returns a 3x3 rotation.
    Eigen::Matrix3d integrate(double t0, double t1) const;
};

}  // namespace vins_sparse
