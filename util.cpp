#include <algorithm> // std::min, std::max
#include <sstream> // std::ostringstream
#include <string>
#include <cmath>
#include "util.h"

int ceiling_divide(float a, float b) {
  double ipart, fpart;
  fpart = modf(a/b, &ipart);
  return ((int) ipart) + (fpart > 0);
}

int int_clamp(int val, int low, int high) {
  return std::max(low, std::min(val, high));
}

float lerp(float a, float b, float r) {
  return a * (1-r) + b * r;
}

float lerp_2d(float bl, float br, float tl, float tr, float x, float y) {
  // first apply horizontal lerp
  float bot = lerp(bl, br, x);
  float top = lerp(tl, tr, x);
  // then apply vertical lerp
  return lerp(bot, top, y);
}

void interp_angles(float bl, float br, float tl, float tr, float x, float y, float scale, float* x_theta, float* y_theta) {
  // calculate x_theta
  float left_z = lerp(bl, tl, y);
  float right_z = lerp(br, tr, y);
  *x_theta = atan((right_z - left_z) / scale);
  // calculate y_theta
  float bot_z = lerp(bl, br, x);
  float top_z = lerp(tl, tr, x);
  *y_theta = atan((top_z - bot_z) / scale);
}


// Slices a string including the first element, excluding the ending element
std::string string_slice(std::string s, int a, int b) {
  int len = b - a;
  return s.substr(a, len);
}

std::string filename_append(std::string input_filename, std::string suffix) {
  int dot_index = input_filename.find('.');
  std::string base_filename = string_slice(input_filename, 0, dot_index);
  std::string extension = string_slice(input_filename, dot_index, input_filename.length());
  return base_filename + suffix + extension;
}

// e.g. 1234567 --> "1.2 M"
std::string format_number(int num) {
	// First multiply by 10 to get one decimal place
	num *= 10;
	char SUFFIXES[] = {'K', 'M', 'B'};
	int suff_idx = -1;
	while(num >= 10000) {
	  num /= 1000;
		suff_idx++;
	}
	int int_part = num/10;
	int dec_part = num%10;
	std::string s = std::to_string(int_part);
	if (dec_part && (s.length() < 3)) {
	  s += "." + std::to_string(dec_part);
  }
	if (suff_idx >= 0) {
	  s += std::string(" ") + SUFFIXES[suff_idx];
  }
	return s;
}

std::string bbox_to_str(bbox b) {
  std::ostringstream oss;
  oss << "[ (" << b.minx << ", " << b.miny << "), (" << b.maxx << ", " << b.maxy << ") ]";
  return oss.str();
}

// https://stackoverflow.com/questions/11140483/how-to-get-list-of-files-with-a-specific-extension-in-a-given-folder
namespace fs = ::boost::filesystem;
void get_files_with_ext(const fs::path &root, const std::string &ext,
                        std::vector<std::string> &ret) {
  if (!fs::exists(root) || !fs::is_directory(root)) return;
  fs::recursive_directory_iterator it(root);
  fs::recursive_directory_iterator endit;
  while (it != endit) {
    if (fs::is_regular_file(*it) && it->path().extension() == ext) {
      ret.push_back(it->path().string());
    }
    ++it;
  }
}

std::string basename(std::string filename) {
  ::boost::filesystem::path p(filename);
  return p.filename().string();
}

std::string path_join(std::string s1, std::string s2) {
  boost::filesystem::path p1(s1);
  boost::filesystem::path p2(s2);
  return (p1 / p2).string();
}
