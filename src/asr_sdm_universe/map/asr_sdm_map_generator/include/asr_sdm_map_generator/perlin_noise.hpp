#pragma once

#include <vector>

namespace asr_sdm_map_generator
{

// Improved Perlin noise (reference Java implementation by Ken Perlin, 2002) with a
// permutation table shuffled from a seed. Ported from mockamap.
class PerlinNoise
{
public:
  explicit PerlinNoise(unsigned int seed);

  // Noise value in [0, 1].
  double noise(double x, double y, double z) const;

private:
  static double fade(double t);
  static double lerp(double t, double a, double b);
  static double grad(int hash, double x, double y, double z);

  std::vector<int> p_;
};

}  // namespace asr_sdm_map_generator
