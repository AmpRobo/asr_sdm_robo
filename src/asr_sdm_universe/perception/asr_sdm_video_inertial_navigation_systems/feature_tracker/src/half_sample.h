#pragma once

#include <opencv2/core.hpp>
#include <cstdint>

namespace vins_sparse {

// 5-tap separable Gaussian, normalized by 256. Half-sample a single-channel
// 8-bit image. Output dimensions are floor(in.rows/2), floor(in.cols/2).
// Weights: [1, 5, 10, 10, 5, 1] / 32 summed along the row, applied to
// col-decimated pixels to get a 2D smoothing-and-subsample step identical
// to vikit's halfSample. CV_8UC1 only (sparse image align uses grayscale).
inline void halfSample8u(const cv::Mat& in, cv::Mat& out) {
    CV_Assert(in.type() == CV_8UC1);
    const int in_h = in.rows;
    const int in_w = in.cols;
    const int out_h = in_h / 2;
    const int out_w = in_w / 2;
    out.create(out_h, out_w, CV_8UC1);

    // Apply the 5-tap filter along x first into a temp buffer of height in_h.
    cv::Mat tmp(in_h, out_w, CV_16SC1);
    static const int kW[6] = {1, 5, 10, 10, 5, 1};
    const int kSum = 32;
    for (int y = 0; y < in_h; ++y) {
        const uint8_t* row_in = in.ptr<uint8_t>(y);
        int16_t* row_tmp = tmp.ptr<int16_t>(y);
        for (int x = 0; x < out_w; ++x) {
            const int sx = 2 * x;
            // Clamp the kernel on image borders.
            int idx[6] = {
                std::max(sx - 2, 0),
                std::max(sx - 1, 0),
                sx,
                std::min(sx + 1, in_w - 1),
                std::min(sx + 2, in_w - 1),
                std::min(sx + 3, in_w - 1)
            };
            int sum = 0;
            for (int k = 0; k < 6; ++k) sum += kW[k] * row_in[idx[k]];
            row_tmp[x] = static_cast<int16_t>(sum);
        }
    }

    // Then apply the same filter along y and divide.
    for (int y = 0; y < out_h; ++y) {
        const int sy = 2 * y;
        const int16_t* row0 = tmp.ptr<int16_t>(sy);
        const int16_t* row1 = (sy + 1 < in_h) ? tmp.ptr<int16_t>(sy + 1) : row0;
        const int16_t* row2 = (sy + 2 < in_h) ? tmp.ptr<int16_t>(sy + 2) : row1;
        const int16_t* row3 = (sy + 3 < in_h) ? tmp.ptr<int16_t>(sy + 3) : row2;
        const int16_t* row4 = (sy + 4 < in_h) ? tmp.ptr<int16_t>(sy + 4) : row3;
        const int16_t* row5 = (sy + 5 < in_h) ? tmp.ptr<int16_t>(sy + 5) : row4;
        uint8_t* row_out = out.ptr<uint8_t>(y);
        for (int x = 0; x < out_w; ++x) {
            int sum = kW[0] * row0[x] + kW[1] * row1[x] + kW[2] * row2[x] +
                      kW[3] * row3[x] + kW[4] * row4[x] + kW[5] * row5[x];
            row_out[x] = static_cast<uint8_t>((sum + kSum * kSum / 2) /
                                              (kSum * kSum));
        }
    }
}

// Build a pyramid of half-sampled images (level 0 == input). out[0] is a
// copy of the input; out[k] is halfSample(out[k-1]) for k >= 1.
inline void buildHalfSamplePyramid(const cv::Mat& img_level0,
                                   int n_levels,
                                   std::vector<cv::Mat>& pyr) {
    pyr.resize(n_levels);
    pyr[0] = img_level0;
    for (int i = 1; i < n_levels; ++i) {
        halfSample8u(pyr[i - 1], pyr[i]);
    }
}

}  // namespace vins_sparse
