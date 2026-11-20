#include "asr_sdm_map_generator/mock_maps.hpp"

#include "asr_sdm_map_generator/perlin_noise.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace asr_sdm_map_generator
{
namespace
{

int cellCount(double length, double resolution)
{
  return resolution > 0.0 ? static_cast<int>(length / resolution) : 0;
}

int randInt(std::default_random_engine & eng, int lo, int hi)
{
  return std::uniform_int_distribution<int>(lo, hi)(eng);
}

// Point at offset (dx, dy, dz) from the map's lower corner, shifted to the voxel center.
pcl::PointXYZ gridPoint(const MapGrid & grid, double dx, double dy, double dz)
{
  const double half = grid.resolution / 2.0;
  return pcl::PointXYZ(
    static_cast<float>(dx - grid.x_size / 2.0 + half),
    static_cast<float>(dy - grid.y_size / 2.0 + half), static_cast<float>(dz + half));
}

// Recursive division on maze(x, y), 1 = wall. Rows index x, columns index y.
void recursiveDivision(
  int xl, int xh, int yl, int yh, Eigen::MatrixXi & maze, std::default_random_engine & eng)
{
  constexpr int kMaxAttempts = 100;
  const int x_max = static_cast<int>(maze.rows()) - 1;
  const int y_max = static_cast<int>(maze.cols()) - 1;

  // Whether walls crossing at (xm, ym) would block a door in the enclosing walls.
  const auto blocks_outer_door = [&](int xm, int ym) {
    return (xl - 1 >= 0 && maze(xl - 1, ym) == 0) || (xh + 1 <= x_max && maze(xh + 1, ym) == 0) ||
           (yl - 1 >= 0 && maze(xm, yl - 1) == 0) || (yh + 1 <= y_max && maze(xm, yh + 1) == 0);
  };
  // Reopens doors of the enclosing walls that the new walls closed.
  const auto keep_outer_doors_open = [&](int xm, int ym) {
    if (yl - 1 >= 0 && maze(xm, yl - 1) == 0) {
      maze(xm, yl) = 0;
    }
    if (yh + 1 <= y_max && maze(xm, yh + 1) == 0) {
      maze(xm, yh) = 0;
    }
    if (xl - 1 >= 0 && maze(xl - 1, ym) == 0) {
      maze(xl, ym) = 0;
    }
    if (xh + 1 <= x_max && maze(xh + 1, ym) == 0) {
      maze(xh, ym) = 0;
    }
  };
  // Opens a door in three of the four wall segments around the crossing.
  const auto open_doors = [&](int xm, int ym) {
    const int d1 = randInt(eng, xl, xm - 1);
    const int d2 = randInt(eng, xm + 1, xh);
    const int d3 = randInt(eng, yl, ym - 1);
    const int d4 = randInt(eng, ym + 1, yh);
    switch (randInt(eng, 0, 3)) {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;
      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;
      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
      default:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    }
  };
  const auto add_cross_walls = [&](int xm, int ym) {
    for (int i = xl; i <= xh; ++i) {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; ++j) {
      maze(xm, j) = 1;
    }
  };

  if (xl < xh - 3 && yl < yh - 3) {
    // At least 5x5: split into four rooms and recurse.
    int xm = 0;
    int ym = 0;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
      xm = randInt(eng, xl + 1, xh - 1);
      ym = randInt(eng, yl + 1, yh - 1);
      if (!blocks_outer_door(xm, ym)) {
        break;
      }
    }
    add_cross_walls(xm, ym);
    open_doors(xm, ym);
    keep_outer_doors_open(xm, ym);
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze, eng);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze, eng);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze, eng);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze, eng);
  } else if (xl < xh - 2 && yl < yh - 2) {
    // 4x4: one last crossing.
    const int xm = randInt(eng, xl + 1, xh - 1);
    const int ym = randInt(eng, yl + 1, yh - 1);
    add_cross_walls(xm, ym);
    keep_outer_doors_open(xm, ym);
    open_doors(xm, ym);
  } else if (xl < xh - 1 && yl < yh - 2) {
    // 3 x 4+: a single wall along y.
    int doors = 0;
    for (int j = yl; j <= yh; ++j) {
      maze(xl + 1, j) = 1;
    }
    if (yl - 1 >= 0 && maze(xl + 1, yl - 1) == 0) {
      maze(xl + 1, yl) = 0;
      ++doors;
    }
    if (yh + 1 <= y_max && maze(xl + 1, yh + 1) == 0) {
      maze(xl + 1, yh) = 0;
      ++doors;
    }
    if (doors == 0) {
      maze(xl + 1, randInt(eng, yl, yh)) = 0;
    }
  } else if (xl < xh - 2 && yl < yh - 1) {
    // 4+ x 3: a single wall along x.
    int doors = 0;
    for (int i = xl; i <= xh; ++i) {
      maze(i, yl + 1) = 1;
    }
    if (xl - 1 >= 0 && maze(xl - 1, yl + 1) == 0) {
      maze(xl, yl + 1) = 0;
      ++doors;
    }
    if (xh + 1 <= x_max && maze(xh + 1, yl + 1) == 0) {
      maze(xh, yl + 1) = 0;
      ++doors;
    }
    if (doors == 0) {
      maze(randInt(eng, xl, xh), yl + 1) = 0;
    }
  } else if (xl < xh - 1 && yl < yh - 1) {
    // 3x3: a post in the middle.
    maze(xl + 1, yl + 1) = 1;
  }
}

}  // namespace

void generatePerlin3d(
  const MapGrid & grid, const Perlin3dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  const int nx = cellCount(grid.x_size, grid.resolution);
  const int ny = cellCount(grid.y_size, grid.resolution);
  const int nz = cellCount(grid.z_size, grid.resolution);
  if (nx <= 0 || ny <= 0 || nz <= 0) {
    return;
  }

  const PerlinNoise noise(static_cast<unsigned int>(eng()));
  std::vector<double> values;
  values.reserve(static_cast<size_t>(nx) * ny * nz);
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) {
        double value = 0.0;
        for (int it = 1; it <= params.fractal; ++it) {
          const double freq = std::pow(2.0, it) * params.complexity;
          value += params.attenuation / it * noise.noise(freq * i, freq * j, freq * k);
        }
        values.push_back(value);
      }
    }
  }

  // Threshold at the (1 - fill) quantile so roughly `fill` of the voxels are occupied.
  std::vector<double> sorted = values;
  const double fill = std::clamp(params.fill, 0.0, 1.0);
  const size_t pos = std::min(static_cast<size_t>(sorted.size() * (1.0 - fill)), sorted.size() - 1);
  std::nth_element(sorted.begin(), sorted.begin() + pos, sorted.end());
  const double threshold = sorted[pos];

  size_t idx = 0;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) {
        if (values[idx++] > threshold) {
          cloud.points.push_back(
            gridPoint(grid, i * grid.resolution, j * grid.resolution, k * grid.resolution));
        }
      }
    }
  }
}

void generatePost2d(
  const MapGrid & grid, const Post2dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (grid.resolution <= 0.0) {
    return;
  }
  const double res = grid.resolution;
  std::uniform_real_distribution<double> rand_x(-grid.x_size / 2.0, grid.x_size / 2.0);
  std::uniform_real_distribution<double> rand_y(-grid.y_size / 2.0, grid.y_size / 2.0);
  std::uniform_real_distribution<double> rand_w(params.width_min, params.width_max);
  std::uniform_real_distribution<double> rand_h(0.0, grid.z_size);

  for (int n = 0; n < params.obstacle_number; ++n) {
    const double x = rand_x(eng);
    const double y = rand_y(eng);
    const int wid_num = static_cast<int>(std::ceil(rand_w(eng) / res));
    const int hei_num = static_cast<int>(std::ceil(rand_h(eng) / res));
    const int lo = -wid_num / 2;
    const int hi = wid_num / 2;

    // Only the box surface is emitted.
    for (int r = lo; r < hi; ++r) {
      for (int s = lo; s < hi; ++s) {
        for (int t = 0; t < hei_num; ++t) {
          if (r == lo || r == hi - 1 || s == lo || s == hi - 1 || t == 0 || t == hei_num - 1) {
            cloud.points.emplace_back(
              static_cast<float>(x + r * res), static_cast<float>(y + s * res),
              static_cast<float>(t * res));
          }
        }
      }
    }
  }
}

void generateMaze2d(
  const MapGrid & grid, const Maze2dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (params.road_width <= 0.0 || grid.resolution <= 0.0) {
    return;
  }
  const int mx = static_cast<int>(grid.x_size / params.road_width);
  const int my = static_cast<int>(grid.y_size / params.road_width);
  if (mx <= 0 || my <= 0) {
    return;
  }

  Eigen::MatrixXi maze = Eigen::MatrixXi::Zero(mx, my);
  recursiveDivision(0, mx - 1, 0, my - 1, maze, eng);

  if (params.add_wall_x) {
    for (int i = 0; i < mx; ++i) {
      maze(i, 0) = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (params.add_wall_y) {
    for (int j = 0; j < my; ++j) {
      maze(0, j) = 1;
      maze(mx - 1, j) = 1;
    }
  }

  const double res = grid.resolution;
  const int cell_voxels = static_cast<int>(std::ceil(params.road_width / res - 1e-6));
  const int nz = cellCount(grid.z_size, res);
  for (int i = 0; i < mx; ++i) {
    for (int j = 0; j < my; ++j) {
      if (!maze(i, j)) {
        continue;
      }
      for (int ii = 0; ii < cell_voxels; ++ii) {
        for (int jj = 0; jj < cell_voxels; ++jj) {
          for (int k = 0; k < nz; ++k) {
            cloud.points.push_back(gridPoint(
              grid, i * params.road_width + ii * res, j * params.road_width + jj * res, k * res));
          }
        }
      }
    }
  }
}

void generateMaze3d(
  const MapGrid & grid, const Maze3dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  const int nx = cellCount(grid.x_size, grid.resolution);
  const int ny = cellCount(grid.y_size, grid.resolution);
  const int nz = cellCount(grid.z_size, grid.resolution);
  if (params.num_nodes < 2 || nx <= 0 || ny <= 0 || nz <= 0) {
    return;
  }

  std::uniform_real_distribution<double> rand_x(-grid.x_size / 2.0, grid.x_size / 2.0);
  std::uniform_real_distribution<double> rand_y(-grid.y_size / 2.0, grid.y_size / 2.0);
  std::uniform_real_distribution<double> rand_z(0.0, grid.z_size);
  std::vector<Eigen::Vector3d> nodes(params.num_nodes);
  for (auto & node : nodes) {
    node = Eigen::Vector3d(rand_x(eng), rand_y(eng), rand_z(eng));
  }

  const double res = grid.resolution;
  const double hole_margin = params.road_radius * res / 3.0;
  const int holed_lo = static_cast<int>((1.0 - params.connectivity) * params.num_nodes);
  const int holed_hi = static_cast<int>((1.0 + params.connectivity) * params.num_nodes);

  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) {
        const pcl::PointXYZ pt = gridPoint(grid, i * res, j * res, k * res);
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);

        double d1 = std::numeric_limits<double>::max();
        double d2 = std::numeric_limits<double>::max();
        int n1 = -1;
        int n2 = -1;
        for (int n = 0; n < params.num_nodes; ++n) {
          const double d = (nodes[n] - p).squaredNorm();
          if (d < d1) {
            d2 = d1;
            n2 = n1;
            d1 = d;
            n1 = n;
          } else if (d < d2) {
            d2 = d;
            n2 = n;
          }
        }
        d1 = std::sqrt(d1);
        d2 = std::sqrt(d2);

        // Voxels about equidistant from their two nearest nodes form the cell walls.
        if (d2 - d1 >= res) {
          continue;
        }
        // Walls selected by connectivity get a hole around the segment joining the nodes.
        const int pair = n1 + n2;
        if (
          pair > holed_lo && pair < holed_hi &&
          d1 + d2 - (nodes[n1] - nodes[n2]).norm() < hole_margin) {
          continue;
        }
        cloud.points.push_back(pt);
      }
    }
  }
}

}  // namespace asr_sdm_map_generator
