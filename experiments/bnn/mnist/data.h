#ifndef DATA_H
#define DATA_H


#include <cstdint>
#include <iostream>
#include <vector>


namespace data {


struct Data {
  std::vector<std::vector<uint8_t>> images;
  std::vector<uint32_t> labels;
  uint32_t rows;
  uint32_t cols;
  uint32_t channels;
};


inline const uint32_t ToCacheAligned(uint32_t value) {
  return ((value / 64) + ((value % 64) ? 1 : 0)) * 64;
}


bool VerifyCache(const Data& data) {
  if (data.labels.capacity() % 64) {
    std::cout << "Labels not aligned." << std::endl;
    return false;
  }
  uint32_t a = data.images[0].capacity();
  if (a % 64) {
    std::cout << "Images not aligned." << std::endl;
    return false;
  }
  for (uint32_t i=1; i<data.images.size(); ++i) {
    if (data.images[i].capacity() != a) {
      std::cout << " Images has variation in image size." << std::endl;
      return false;
    }
  }
  return true;
}


} // namespace data


#endif // DATA_H
