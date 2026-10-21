#include "rng/rng.h"

#include <stdint.h>
#include <stdio.h>

int main(void) {
  rng_seed(1, 1);
  uint32_t first = rng_u();
  float unit = rng_f();
  float normal = rng_norm();

  rng_seed(1, 1);
  if (rng_u() != first) {
    fprintf(stderr, "rng_seed is not deterministic\n");
    return 1;
  }
  if (!(unit >= 0.0f && unit < 1.0f)) {
    fprintf(stderr, "rng_f out of range: %f\n", (double)unit);
    return 1;
  }
  if (normal != normal) {
    fprintf(stderr, "rng_norm produced NaN\n");
    return 1;
  }

  return 0;
}
