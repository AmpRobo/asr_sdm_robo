#pragma once

// Tukey's biweight M-estimator weight function for IRLS.
// SVO pro uses the same formulation as vikit's vk::solver::TukeyWeightFunction:
//   w(x) = (1 - x^2 / b^2)^2  if |x| <= b
//   w(x) = 0                   otherwise
// Default b = 4.6851 is the 95% efficiency tuning constant for Tukey's biweight.

namespace vins_sparse {

struct TukeyWeight {
    TukeyWeight() = default;
    explicit TukeyWeight(float b) : b_square_(b * b) {}

    inline float weight(float x) const {
        const float x2 = x * x;
        if (x2 <= b_square_) {
            const float tmp = 1.0f - x2 / b_square_;
            return tmp * tmp;
        }
        return 0.0f;
    }

    float b_square() const { return b_square_; }

private:
    float b_square_ = 4.6851f * 4.6851f;
};

}  // namespace vins_sparse
