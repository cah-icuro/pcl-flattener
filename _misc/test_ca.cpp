#include <iostream>
#include "circular_array.hpp"

int main() {
  circular_array<int> cir(10);
  for (int i = 0; i < 12; i++) {
    cir.insert(i);
  }
  std::vector<int> vec = cir.as_vec();
  for (int el: vec) {
    std::cout << el << ", ";
  }
  std::cout << std::endl;
  std::cout << "min: " << cir.min_value() << std::endl;
  return 0;
}
