#ifndef UTIL_H
#define UTIL_H

#include <vector>

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 
#include <boost/filesystem.hpp>

#include "aux_types.h"

int ceiling_divide(float a, float b);

int int_clamp(int val, int low, int high);

// x, y in range [0, 1] expressing horizontal and vertical position of target point
float lerp_2d(float bl, float br, float tl, float tr, float x, float y);

void interp_angles(float bl, float br, float tl, float tr, float x, float y, float scale, float* x_theta, float* y_theta);

std::string string_slice(std::string s, int a, int b);

std::string filename_append(std::string input_filename, std::string suffix);

std::string format_number(int num);

std::string bbox_to_str(bbox b);

void get_files_with_ext(const ::boost::filesystem::path &root,
     const std::string &ext, std::vector<std::string> &ret);
     
std::string basename(std::string filename);

std::string path_join(std::string s1, std::string s2);

#endif // UTIL_H
