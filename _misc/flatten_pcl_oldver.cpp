#include <iostream>
#include <string>
#include <ios>
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdio> // rename

#include "circular_array.hpp"

// Stop processing after this many lines
const int EARLY_EXIT = -1;

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

int main(int argc, char **argv) {
  // Parse command line parameters
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <input_file.pcd>" << std::endl;
    return 0;
  }
  std::string input_filename = argv[1];
  std::cout << "Reading pointcloud from " << input_filename << std::endl;
  std::ifstream ifs(input_filename);
  std::string output_filename = filename_append(input_filename, "_flat");
  std::cout << "Writing output to " << output_filename << std::endl;
  std::ofstream ofs(output_filename, std::ios_base::out);

  std::cout << std::endl << "Processing..." << std::endl << std::endl;
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
