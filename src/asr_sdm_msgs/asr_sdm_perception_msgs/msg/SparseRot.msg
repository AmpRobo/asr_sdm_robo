# Sparse image alignment rotation result.
#
# Published by feature_tracker (SPARSE1 pipeline) each time sparseAlign succeeds.
# Consumed by vins_estimator for stat logging (D2.1) and later for Ceres BA
# soft rotation prior (D2.2).
#
# Fields:
#   rot_mat[9]    : 3x3 rotation matrix, row-major (sparse-align output)
#   success       : alignment converged and met chi2 / min-feature gates
#   chi2          : mean photometric residual at last iteration
#   n_meas        : number of 4x4 patches used
#   iterations    : number of meaningful GN iterations
#   stamp_sec     : image timestamp as double
std_msgs/Header header
float64[9] rot_mat
bool success
float32 chi2
int32 n_meas
int32 iterations
float64 stamp_sec
