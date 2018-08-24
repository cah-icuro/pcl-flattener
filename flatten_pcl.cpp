#include <iostream>
#include <iomanip>
#include <string>
#include <ios>
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdio> // rename
#include <cstdlib>
#include <utility> // std::pair
#include <cmath>

#include "circular_array.hpp"
#include "grid.hpp"
#include "aux_types.h"
#include "util.h"


/* Read PCD to vector */
void read_pcd(std::string filename, std::vector<std::string> &headers, std::vector<lidar_point> &points) {
  std::ifstream ifs(filename);
  std::string line;
  int line_number = 1;
  int in_header = 1;
  while (std::getline(ifs, line)) {
    // if first char of line is a digit or minus sign, process as lidar data
    if (isdigit(line[0]) || (line[0] == '-')) {
      in_header = 0;
      float x, y, z;
      int intensity;
      std::istringstream iss(line);
      if (!(iss >> x >> y >> z >> intensity)) {
        std::cout << "Error parsing line " << line_number << std::endl;
      }
      points.push_back(lidar_point{x, y, z, intensity});
    }
    // otherwise, tread it as a header line
    else {
      if (in_header) headers.push_back(line);
    }
    // display progress
    if ((line_number % 10000) == 0) {
      std::cout << "\rLoaded " << format_number(line_number) << " lines     " << std::flush;
    }
    line_number++;
  }
  std::cout << std::endl;
  ifs.close();
}

/* Write PCD vector to file */
void write_pcd(std::string filename, std::vector<std::string> &headers, std::vector<lidar_point> &points) {
  std::ofstream ofs(filename, std::ios_base::out);
  int line_number = 1;
  for (std::string const &header_line : headers) {
    ofs << header_line << std::endl;
    line_number++;
  }
  for (lidar_point const &p : points) {
    ofs << p.x << " " << p.y << " " << p.z << " " << p.intensity << std::endl;
    // display progress
    if ((line_number % 10000) == 0) {
      std::cout << "\rWrote " << format_number(line_number) << " lines     " << std::flush;
    }
    line_number++;
  }
  std::cout << std::endl;
  ofs.close();
}

/* Compute bounding box (in x-y plane) around all points) */
bbox compute_full_bbox(std::vector<lidar_point> points) {
  float minx = 0.0;
  float miny = 0.0;
  float maxx = 0.0;
  float maxy = 0.0;
  for (lidar_point p : points) {
    if (p.x < minx) minx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.x > maxx) maxx = p.x;
    if (p.y > maxy) maxy = p.y;
  }
  return bbox{minx, miny, maxx, maxy};
}


void adjust_point(lidar_point *p, float bl_z, float br_z, float tl_z, float tr_z,
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

void flatten_pcd(std::string full_input_filename, std::string full_output_filename) {
  std::cout << std::endl << "Now flattening " << full_input_filename << "..." << std::endl;

  // Read pointcloud to vector
  std::cout << "Reading pointcloud:" << std::endl;
  std::vector<std::string> headers;
  std::vector<lidar_point> points;
  read_pcd(full_input_filename, headers, points);
  std::cout << "Total points read: " << points.size() << std::endl;
  
  // Find bbox for complete pointcloud
  bbox full_pcl_bbox = compute_full_bbox(points);
  std::cout << "Full pointcloud bbox: [ (" << full_pcl_bbox.minx << ", " << full_pcl_bbox.miny
                << "), (" << full_pcl_bbox.maxx << ", " << full_pcl_bbox.maxy << ") ]" << std::endl;

  // Create grid
  float GRID_SIDE_LEN = 20;
  grid pcl_grid(GRID_SIDE_LEN);
  std::pair<int, int> grid_dim = pcl_grid.compute_grid(full_pcl_bbox);
  
  // Divide up the z's into grid blocks
  std::cout << "Placing z's into grid blocks..." << std::endl;
  std::vector< std::vector< std::vector<float> > > z_arrays(grid_dim.first,
                std::vector< std::vector<float> >(grid_dim.second, std::vector<float>()));
  for (lidar_point const &p : points) {
    std::pair<int, int> grid_indices = pcl_grid.to_indices(p.x, p.y);
    z_arrays[grid_indices.first][grid_indices.second].push_back(p.z);
  }
  // Debugging
  std::cout << "Grid blocks' sizes:" << std::endl;
  for (int col = 0; col < pcl_grid.h(); col++) {
    for (int row = 0; row < pcl_grid.w(); row++) {
      std::cout << std::setw(8) << z_arrays[col][row].size() << " ";
    }
    std::cout << std::endl;
  }

  // Compute floor z for each block
  std::cout << "Computing ground height per block..." << std::endl;
  const int MIN_POINTS_PER_BLOCK = 100;
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
  std::cout << "Ground zs:" << std::endl;
  for (int col = 0; col < pcl_grid.h(); col++) {
    for (int row = 0; row < pcl_grid.w(); row++) {
      std::cout << std::setw(7) /*<< std::fixed*/ << std::setprecision(4) << floor_zs[col][row] << " ";
    }
    std::cout << std::endl;
  }
  
  // Adjust zs for each point
  std::cout << "Updating z values for all points..." << std::endl;
  for (lidar_point &p : points) {
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
  std::cout << "Computations finished." << std::endl;
  
  // Rewrite pcd
  std::cout << "Writing output to " << full_output_filename << ":" << std::endl;
  write_pcd(full_output_filename, headers, points);
  std::cout << "Done." << std::endl << std::endl;
}

int main(int argc, char **argv) {
  // Parse command line parameters
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <input_file.pcd>" << std::endl;
    return 0;
  }
  std::string input_filename = argv[1];
  std::string output_filename = filename_append(input_filename, "_flat");
  flatten_pcd(input_filename, output_filename);
  
  return 0;
}


/*
  std::cout << "Input file: " << input_filename << std::endl;
  std::cout << "Output file: " << output_filename << std::endl << std::endl;

  // Read pointcloud to vector
  std::cout << "Reading pointcloud:" << std::endl;
  std::vector<std::string> headers;
  std::vector<lidar_point> points;
  read_pcd(input_filename, headers, points);
  std::cout << "Total points read: " << points.size() << std::endl;
  
  // Find bbox for complete pointcloud
  bbox full_pcl_bbox = compute_full_bbox(points);
  std::cout << "Full pointcloud bbox: [ (" << full_pcl_bbox.minx << ", " << full_pcl_bbox.miny
                << "), (" << full_pcl_bbox.maxx << ", " << full_pcl_bbox.maxy << ") ]" << std::endl;

  // Create grid
  float GRID_SIDE_LEN = 20;
  grid pcl_grid(GRID_SIDE_LEN);
  std::pair<int, int> grid_dim = pcl_grid.compute_grid(full_pcl_bbox);
  
  // Divide up the z's into grid blocks
  std::cout << "Placing z's into grid blocks..." << std::endl;
  std::vector< std::vector< std::vector<float> > > z_arrays(grid_dim.first,
                std::vector< std::vector<float> >(grid_dim.second, std::vector<float>()));
  for (lidar_point const &p : points) {
    std::pair<int, int> grid_indices = pcl_grid.to_indices(p.x, p.y);
    z_arrays[grid_indices.first][grid_indices.second].push_back(p.z);
  }
  // Debugging
  std::cout << "Grid blocks' sizes:" << std::endl;
  for (int col = 0; col < pcl_grid.h(); col++) {
    for (int row = 0; row < pcl_grid.w(); row++) {
      std::cout << std::setw(8) << z_arrays[col][row].size() << " ";
    }
    std::cout << std::endl;
  }

  // Compute floor z for each block
  std::cout << "Computing ground height per block..." << std::endl;
  const int MIN_POINTS_PER_BLOCK = 100;
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
  std::cout << "Ground zs:" << std::endl;
  for (int col = 0; col < pcl_grid.h(); col++) {
    for (int row = 0; row < pcl_grid.w(); row++) {
      std::cout << std::setw(7) << std::setprecision(4) << floor_zs[col][row] << " ";
    }
    std::cout << std::endl;
  }
  
  // Adjust zs for each point
  std::cout << "Updating z values for all points..." << std::endl;
  for (lidar_point &p : points) {
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
  std::cout << "Computations finished." << std::endl;
  
  // Rewrite pcd
  std::cout << "Writing output to " << output_filename << ":" << std::endl;
  write_pcd(output_filename, headers, points);
  std::cout << "Done." << std::endl << std::endl;

  return 0;
*/

 /*

  // Set parameters
  const int CIRC_ARR_SIZE = 10000;
  const int UPDATE_INTER = 100;
  const int K = CIRC_ARR_SIZE / 50;
  float alpha = 0.5;
  float z_floor = 0;
  // Init vars and loop
  int line_number = 1;
  int points_read = 0;
  int early_exit_flag = 0;
  circular_array<float> recent_zs(CIRC_ARR_SIZE);
  std::string line;
  // Main processing loop
  while (std::getline(ifs, line)) {
    // if first char of line is a digit, process numerical fields
    if (isdigit(line[0]) || (line[0] == '-')) {
      float x, y, z;
      int intensity;
      std::istringstream iss(line);
      if (!(iss >> x >> y >> z >> intensity)) {
        std::cout << "Error parsing line " << line_number << std::endl;
      }
      // Insert z into circular array
      recent_zs.insert(z);
      // Update z_floor every UPDATE_INTER points processed
      if ((points_read >= CIRC_ARR_SIZE) && (points_read % UPDATE_INTER == 0)) {
        float recent_z_floor = recent_zs.kth_smallest_value(K);
        z_floor = alpha * recent_z_floor + (1.0 - alpha) * z_floor;
      }
      float adjusted_z = z - z_floor;
      ofs << x << " " << y << " " << adjusted_z << " " << intensity << std::endl;
      points_read++;
    }
    // else, first char is not a digit -> treat as a header line and rewrite
    else {
      ofs << line << std::endl;;
    }
    line_number++;
    if ((line_number % 10000) == 0) {
      std::cout << "\rProcessed " << format_number(line_number) << " lines     " << std::flush;
    }
    if ((EARLY_EXIT > 0) && (line_number > EARLY_EXIT)) {
      early_exit_flag = 1;
      break;
    }
  }
  std::cout << std::endl;
  ifs.close();
  ofs.close();

  // Fix points count and width (if there was an early exit)
  if (early_exit_flag) {
    char TEMP_FILENAME[] = "__temp_file.pcd";
    std::rename(output_filename.c_str(), TEMP_FILENAME);
    ifs = std::ifstream(TEMP_FILENAME);
    ofs = std::ofstream(output_filename, std::ios_base::out);
    while (std::getline(ifs, line)) {
      if (line.substr(0, 5) == "WIDTH") {
        ofs << "WIDTH " << points_read << std::endl;
      }
      else if (line.substr(0, 6) == "POINTS") {
        ofs << "POINTS " << points_read << std::endl;
      }
      else {
        ofs << line << std::endl;
      }
    }
    // Todo: delete TEMP_FILENAME file
  }

  std::cout << "Done. Results written to " << output_filename << "." << std::endl;

  return 0;
}
*/
