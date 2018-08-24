#ifndef GRID_H
#define GRID_H

#include "aux_types.h"

#include <utility> // std::pair
#include "util.h" // int ceiling_divide(float, float);

class grid {

  private:
  float section_len;
  int _h, _w;
  float base_x, base_y;

  public:
  
  grid(float section_len) {
    this->section_len = section_len;
  }
  
  int h() {
    return this->_h;
  }
  
  int w() {
    return this->_w;
  }
  
  float s() {
    return this->section_len;
  }
  
  std::pair<int, int> compute_grid(bbox box) {
    float dx = box.maxx - box.minx;
    float dy = box.maxy - box.miny;
    this->_h = ceiling_divide(dy, section_len);
    this->_w = ceiling_divide(dx, section_len);
    this->base_x = box.minx;
    this->base_y = box.miny;
    return std::pair<int, int>(this->_h, this->_w);
  }
  
  std::pair<int, int> to_indices(float x, float y) {
    int x_idx = (int) ((x - base_x) / section_len);
    int y_idx = (int) ((y - base_y) / section_len);
    return std::pair<int, int>(y_idx, x_idx);
  }
  
  std::pair<float, float> center_coords(int y_idx, int x_idx) {
    float x = this->base_x + section_len * ( (float) x_idx + 0.5 );
    float y = this->base_y + section_len * ( (float) y_idx + 0.5 );
    return std::pair<float, float>(x, y);
  }
  
};
    
#endif // GRID_H
