
/****************************************/
/*               Headers                */
/****************************************/

// Std Library
#include <iostream>
#include <iomanip>
#include <string>
#include <ios>
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdio> // rename, sprintf
#include <cstdlib>
#include <utility> // std::pair
#include <cmath>
#include <boost/filesystem.hpp>

// Pcl Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Project-Specific
#include "circular_array.hpp"
#include "grid.hpp"
#include "aux_types.h"
#include "util.h"

const bool VERBOSE = false;

/****************************************/
/*     PointCloud Helper Functions      */
/****************************************/

typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

PclPointCloud::Ptr read_pcd(std::string input_filename) {
  PclPointCloud::Ptr cloud(new PclPointCloud);
  int result = pcl::io::loadPCDFile<PclPoint>(input_filename, *cloud);
  if (result == -1) {
    PCL_ERROR(std::string(("Couldn't read file ") + input_filename).c_str());
    return cloud;
  }
  std::cout << "Loaded " << cloud->width * cloud->height
            << " points from " << input_filename << std::endl;
  return cloud;
}

void write_pcd(std::string output_filename, PclPointCloud::Ptr cloud) {
  const bool binary_mode = true;
  pcl::io::savePCDFile(output_filename, *cloud, binary_mode);
}


/****************************************/
/*      Point-related Computations      */
/****************************************/

/* Compute bounding box (in x-y plane) around all points) */
bbox compute_full_bbox(std::vector<PclPoint> points) {
  float minx = 0.0;
  float miny = 0.0;
  float maxx = 0.0;
  float maxy = 0.0;
  for (PclPoint p : points) {
    if (p.x < minx) minx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.x > maxx) maxx = p.x;
    if (p.y > maxy) maxy = p.y;
  }
  return bbox{minx, miny, maxx, maxy};
}

/* Adjust a point's position given the computed ground height */
void adjust_point(PclPoint *p, float bl_z, float br_z, float tl_z, float tr_z,
                   float x_ratio, float y_ratio, float grid_scale) {
  // interpolate floor z value
  float floor_z = lerp_2d(bl_z, br_z, tl_z, tr_z, x_ratio, y_ratio);
  p->z -= floor_z;
  // interpolate x/y angles of inclination
  float x_theta, y_theta;
  interp_angles(bl_z, br_z, tl_z, tr_z, x_ratio, y_ratio, grid_scale, &x_theta, &y_theta);
  // transform the point so these angles become zero
  //  - rotate by -x_theta about y axis
  //  - rotate by -y_theta about x axis
  // First x direction
  float dx = -1.0 * p->z * std::tan(x_theta);
  p->z = std::sqrt(p->z * p->z + dx * dx);
  p->x -= dx;
  // Then y direction
  float dy = -1.0 * p->z * std::tan(y_theta);
  p->z = std::sqrt(p->z * p->z + dy * dy);
  p->y -= dy;
}


/****************************************/
/*   Primary Function to Flatten PCD    */
/****************************************/

// Tunable parameters
const float GRID_SIDE_LEN = 20;
const int MIN_POINTS_PER_BLOCK = 100;

// Flatten function
void flatten_pcd(std::string full_input_filename, std::string full_output_filename) {
  std::cout << std::endl << "Now flattening " << full_input_filename << "..." << std::endl;
  // Read pointcloud
  PclPointCloud::Ptr cloud = read_pcd(full_input_filename);
  // TODO find a better way to convert aligned vector to normal vector
  std::vector<PclPoint> points;
  for (unsigned int i = 0; i < cloud->points.size(); i++) {
    points.push_back(cloud->points[i]);
  }
  
  // Find bbox for complete pointcloud
  bbox full_pcl_bbox = compute_full_bbox(points);
  if (VERBOSE) std::cout << "Full pointcloud bbox: " << bbox_to_str(full_pcl_bbox) << std::endl;

  // Create grid
  grid pcl_grid(GRID_SIDE_LEN);
  std::pair<int, int> grid_dim = pcl_grid.compute_grid(full_pcl_bbox);
  
  // Divide up the z's into grid blocks
  if (VERBOSE) std::cout << "Placing z's into grid blocks..." << std::endl;
  std::vector< std::vector< std::vector<float> > > z_arrays(grid_dim.first,
                std::vector< std::vector<float> >(grid_dim.second, std::vector<float>()));
  for (PclPoint const &p : points) {
    std::pair<int, int> grid_indices = pcl_grid.to_indices(p.x, p.y);
    z_arrays[grid_indices.first][grid_indices.second].push_back(p.z);
  }
  if (VERBOSE) {
    std::cout << "Grid blocks' sizes:" << std::endl;
    for (int col = 0; col < pcl_grid.h(); col++) {
      for (int row = 0; row < pcl_grid.w(); row++) {
        std::cout << std::setw(8) << z_arrays[col][row].size() << " ";
      }
      std::cout << std::endl;
    }
  }

  // Compute floor z for each block
  if (VERBOSE) std::cout << "Computing ground height per block..." << std::endl;
  std::vector< std::vector<float> > floor_zs(grid_dim.first,
                std::vector<float>(grid_dim.second));
  for (int y_idx = 0; y_idx < grid_dim.first; y_idx++) {
    for (int x_idx = 0; x_idx < grid_dim.second; x_idx++) {
      std::vector<float> z_vec = z_arrays[y_idx][x_idx];
      if (z_vec.size() > MIN_POINTS_PER_BLOCK) { // skip sections with very few points
        std::sort(z_vec.begin(), z_vec.end());
        int k = z_vec.size()/20; // <-- Parameter to be tuned!
        floor_zs[y_idx][x_idx] = z_vec[k];
      }
    }
  }
  if (VERBOSE) {
    std::cout << "Ground zs:" << std::endl;
    for (int col = 0; col < pcl_grid.h(); col++) {
      for (int row = 0; row < pcl_grid.w(); row++) {
        std::cout << std::setw(7) << std::setprecision(4) << floor_zs[col][row] << " ";
      }
      std::cout << std::endl;
    }
  }
  
  // Adjust each point based on floor height and angle with floor
  if (VERBOSE) std::cout << "Adjusting all points..." << std::endl;
  for (PclPoint &p : points) {
    std::pair<int, int> indices = pcl_grid.to_indices(p.x, p.y);
    std::pair<float, float> center = pcl_grid.center_coords(indices.first, indices.second);
    float dx_ratio = (p.x - center.first) / GRID_SIDE_LEN;
    float dy_ratio = (p.y - center.second) / GRID_SIDE_LEN;
    if (dy_ratio < 0) {
      indices.first--;
      dy_ratio += 1.0;
    }
    if (dx_ratio < 0) {
      indices.second--;
      dx_ratio += 1.0;
    }
    // get necessary values for four neighboring grid blocks
    int y_bot = int_clamp(indices.first, 0, pcl_grid.h() - 1);
    int y_top = int_clamp(indices.first + 1, 0, pcl_grid.h() - 1);
    int x_left = int_clamp(indices.second, 0, pcl_grid.w() - 1);
    int x_right = int_clamp(indices.second + 1, 0, pcl_grid.w() - 1);
    float z_bl = floor_zs[y_bot][x_left];
    float z_br = floor_zs[y_bot][x_right];
    float z_tl = floor_zs[y_top][x_left];
    float z_tr = floor_zs[y_top][x_right];
    // Finally, adjust z value for this point
    adjust_point(&p, z_bl, z_br, z_tl, z_tr, dx_ratio, dy_ratio, pcl_grid.s());
  }
  
  // Write points back to cloud->points vector
  for (unsigned int i = 0; i < cloud->points.size(); i++) {
    cloud->points[i] = points[i];
  }
  
  // Rewrite pcd
  std::cout << "Computations finished, writing output to "
            << full_output_filename << "..." << std::endl;
  write_pcd(full_output_filename, cloud);
  if (VERBOSE) std::cout << "Done." << std::endl << std::endl;
}

int main(int argc, char **argv) {
  // Parse command line parameters
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <input_directory>" << std::endl;
    return 0;
  }
  std::string input_path(argv[1]);
  std::string output_path = path_join(input_path, "flat_output");
  boost::filesystem::create_directory(output_path);
  std::vector<std::string> input_filenames;
  get_files_with_ext(input_path, ".pcd", input_filenames);
  for (std::string input_filename : input_filenames) {
    std::string output_basename = filename_append(basename(input_filename), "_flat");
    flatten_pcd(input_filename, path_join(output_path, output_basename));
  }  
  return 0;
}

