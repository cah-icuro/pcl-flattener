#ifndef AUX_TYPES_H
#define AUX_TYPES_H

struct bbox {
  float minx, miny, maxx, maxy;
};

struct lidar_point {
  float x, y, z;
  int intensity;
};

#endif // AUX_TYPES_H
