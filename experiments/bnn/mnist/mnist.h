#ifndef MNIST_H
#define MNIST_H


#include <array>
#include <fstream>
#include <optional>
#include <vector>

#include "data.h"


namespace mnist {


uint32_t ReadUint32(std::ifstream& fin);


bool LoadImages(std::ifstream& fin, data::Data& data_to_fill) {
  char magic[4];
  fin.read(&magic[0], 4);
  if (magic[0] != 0 || magic[1] != 0 || magic[2] != 8 || magic[3] != 3) { return false; }
  uint32_t num_images = ReadUint32(fin);
  uint32_t rows = ReadUint32(fin);
  uint32_t cols = ReadUint32(fin);
  uint32_t total_bytes_per_image = rows * cols;
  data_to_fill.rows = rows;
  data_to_fill.cols = cols;
  data_to_fill.channels = 1;
  data_to_fill.images.reserve(num_images);
  for (uint32_t image_num=0; image_num<num_images; ++image_num) {
    data_to_fill.images.emplace_back(std::vector<uint8_t>{});
  }
  for (uint32_t image_num=0; image_num<num_images; ++image_num) {
    std::vector<uint8_t>& v = data_to_fill.images[image_num];
    v.reserve(data::ToCacheAligned(total_bytes_per_image));
    for (uint32_t i=0; i<total_bytes_per_image; ++i) {
      char c;
      fin.read(&c, 1);
      v.emplace_back(static_cast<uint8_t>(c));
    }
  }
  return true;
}


bool LoadLabels(std::ifstream& fin, data::Data& data_to_fill) {
  char magic[4];
  fin.read(&magic[0], 4);
  if (magic[0] != 0 || magic[1] != 0 || magic[2] != 8 || magic[3] != 1) { return false; }
  uint32_t num_items = ReadUint32(fin);
  data_to_fill.labels.reserve(data::ToCacheAligned(num_items));
  for (uint32_t item_num=0; item_num<num_items; ++item_num) {
    char c;
    fin.read(&c, 1);
    data_to_fill.labels.emplace_back(static_cast<uint32_t>(c));
  }
  return true;
}


bool LoadTesting(data::Data& data_to_fill) {
  std::ifstream fin_images("datasets/t10k-images-idx3-ubyte", std::ios::binary);
  std::ifstream fin_labels("datasets/t10k-labels-idx1-ubyte", std::ios::binary);
  if (!LoadImages(fin_images, data_to_fill)) { return false; }
  if (!LoadLabels(fin_labels, data_to_fill)) { return false; }
  if (data_to_fill.labels.size() != data_to_fill.images.size()) { return false; }
  return data::VerifyCache(data_to_fill);
}


bool LoadTraining(data::Data& data_to_fill) {
  std::ifstream fin_images("datasets/train-images-idx3-ubyte", std::ios::binary);
  std::ifstream fin_labels("datasets/train-labels-idx1-ubyte", std::ios::binary);
  if (!LoadImages(fin_images, data_to_fill)) { return false; }
  if (!LoadLabels(fin_labels, data_to_fill)) { return false; }
  if (data_to_fill.labels.size() != data_to_fill.images.size()) { return false; }
  return data::VerifyCache(data_to_fill);
}


inline uint32_t ReadUint32(std::ifstream& fin) {
  char uint32_bytes[4];
  fin.read(&uint32_bytes[0], 4);
  uint32_t x = (static_cast<uint8_t>(uint32_bytes[3])) +
               (static_cast<uint8_t>(uint32_bytes[2]) << 8) +
               (static_cast<uint8_t>(uint32_bytes[1]) << 16) +
               (static_cast<uint8_t>(uint32_bytes[0]) << 24);
  return x;
}


} // namespace mnist


#endif // MNIST_H
