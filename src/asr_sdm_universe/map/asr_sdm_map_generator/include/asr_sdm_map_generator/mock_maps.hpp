#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>

// Map generators ported from mockamap (https://github.com/HKUST-Aerial-Robotics/mockamap).
// All maps are centered on the origin in x/y and start at z = 0; points sit at voxel centers.
namespace asr_sdm_map_generator
{

struct MapGrid
{
  double x_size{20.0};
  double y_size{20.0};
  double z_size{5.0};
  double resolution{0.1};
};

// Volumetric Perlin noise thresholded to a target fill ratio.
struct Perlin3dParams
{
  // Base noise frequency per voxel; larger is more complex (typical 0.0 ~ 0.5).
  double complexity{0.142857};
  // Fraction of voxels that are occupied (typical 0.0 ~ 0.4).
  double fill{0.38};
  // Number of octaves; larger adds detail.
  int fractal{1};
  // Amplitude falloff between octaves (typical 0.0 ~ 0.5).
  double attenuation{0.5};
};

// Hollow boxes of random footprint and height.
struct Post2dParams
{
  double width_min{0.6};
  double width_max{1.5};
  int obstacle_number{10};
};

// Recursive-division maze extruded to the full map height.
struct Maze2dParams
{
  double road_width{1.0};
  bool add_wall_x{false};
  bool add_wall_y{false};
};

// Voronoi-cell maze: walls lie on the bisector planes between random nodes.
struct Maze3dParams
{
  int num_nodes{10};
  // Fraction of walls that get a hole.
  double connectivity{0.5};
  // Hole size, in voxels.
  int road_radius{2};
};

void generatePerlin3d(
  const MapGrid & grid, const Perlin3dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud);

void generatePost2d(
  const MapGrid & grid, const Post2dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud);

void generateMaze2d(
  const MapGrid & grid, const Maze2dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud);

void generateMaze3d(
  const MapGrid & grid, const Maze3dParams & params, std::default_random_engine & eng,
  pcl::PointCloud<pcl::PointXYZ> & cloud);

}  // namespace asr_sdm_map_generator
