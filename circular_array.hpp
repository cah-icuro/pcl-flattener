#ifndef CIRCULAR_ARRAY_H
#define CIRCULAR_ARRAY_H

#include <vector>
#include <algorithm>

template <typename T>
class circular_array {

  private:
  T* arr;
  int num_els;
  int cap;
  int start_index;

  public:

  circular_array(int cap) {
    this->arr = (T*) malloc(cap * sizeof(T));
    this->num_els = 0;
    this->cap = cap;
    this->start_index = 0;
  }

  void insert(T el) {
    // If at capacity, get rid of oldest element
    if (this->num_els == cap) {
      start_index++;
      num_els--;
    }
    // Compute index and insert
    int circular_index = (start_index + num_els) % (this->cap);
    (this->arr)[circular_index] = el;
    num_els++;
  }

  T get() {
    int circular_index = (start_index + num_els) % (this->cap);
    return (this->arr)[circular_index];
  }
  
  size_t size() {
    return num_els;
  }

  T min_value() {
    int circular_index;
    T min_val = (this->arr)[0];
    for (int i = 0; i < num_els; i++) {
      T el = (this->arr)[(start_index + i) % (this->cap)];
      min_val = (el < min_val) ? el : min_val;
    }
    return min_val;
  }

  std::vector<T> as_vec() {
    std::vector<T> vec;
    for (int i = 0; i < num_els; i++) {
      T el = (this->arr)[(start_index + i) % (this->cap)];
      vec.push_back(el);
    }
    return vec;
  }
  
  T kth_smallest_value(int k) {
    std::vector<T> vec = this->as_vec();
    std::sort(vec.begin(), vec.end());
    return vec[k];
  }
    
};

#endif // CIRCULAR_ARRAY_H
