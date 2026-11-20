#include "asr_sdm_map_generator/perlin_noise.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

namespace asr_sdm_map_generator
{

PerlinNoise::PerlinNoise(unsigned int seed)
{
  p_.resize(256);
  std::iota(p_.begin(), p_.end(), 0);
  std::default_random_engine engine(seed);
  std::shuffle(p_.begin(), p_.end(), engine);
  // Duplicate so corner hashes can index past 255 without wrapping.
  p_.insert(p_.end(), p_.begin(), p_.end());
}

double PerlinNoise::noise(double x, double y, double z) const
{
  // Unit cube that contains the point.
  const int X = static_cast<int>(std::floor(x)) & 255;
  const int Y = static_cast<int>(std::floor(y)) & 255;
  const int Z = static_cast<int>(std::floor(z)) & 255;

  // Relative position inside the cube.
  x -= std::floor(x);
  y -= std::floor(y);
  z -= std::floor(z);

  const double u = fade(x);
  const double v = fade(y);
  const double w = fade(z);

  // Hash the 8 cube corners.
  const int A = p_[X] + Y;
  const int AA = p_[A] + Z;
  const int AB = p_[A + 1] + Z;
  const int B = p_[X + 1] + Y;
  const int BA = p_[B] + Z;
  const int BB = p_[B + 1] + Z;

  const double res = lerp(
    w,
    lerp(
      v, lerp(u, grad(p_[AA], x, y, z), grad(p_[BA], x - 1, y, z)),
      lerp(u, grad(p_[AB], x, y - 1, z), grad(p_[BB], x - 1, y - 1, z))),
    lerp(
      v, lerp(u, grad(p_[AA + 1], x, y, z - 1), grad(p_[BA + 1], x - 1, y, z - 1)),
      lerp(u, grad(p_[AB + 1], x, y - 1, z - 1), grad(p_[BB + 1], x - 1, y - 1, z - 1))));
  return (res + 1.0) / 2.0;
}

double PerlinNoise::fade(double t)
{
  return t * t * t * (t * (t * 6 - 15) + 10);
}

double PerlinNoise::lerp(double t, double a, double b)
{
  return a + t * (b - a);
}

double PerlinNoise::grad(int hash, double x, double y, double z)
{
  // Lower 4 bits of the hash select one of 12 gradient directions.
  const int h = hash & 15;
  const double u = h < 8 ? x : y;
  const double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
  return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

}  // namespace asr_sdm_map_generator
