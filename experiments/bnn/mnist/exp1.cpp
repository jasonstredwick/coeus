//g++ exp1.cpp -std=c++20 -O3 -march=native -mtune=native -mno-avx256-split-unaligned-load -o bin/exp1

#include <algorithm>
#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

#include "data.h"
#include "mnist.h"
#include "random.h"


constexpr const size_t INPUT_BITS = 28 * 28 * 8;
constexpr const size_t LAYER_BITS = 512;
constexpr const size_t OUTPUT_BITS = 10;
constexpr const size_t NN_TOTAL_BITS = (INPUT_BITS * LAYER_BITS) + (2 * (LAYER_BITS * LAYER_BITS)) + (LAYER_BITS * OUTPUT_BITS);
constexpr const size_t HALF_INPUT_BITS = INPUT_BITS / 2;
constexpr const size_t HALF_LAYER_BITS = LAYER_BITS / 2;

constexpr const size_t POP_SIZE = 128;
constexpr const size_t NUM_GENERATIONS = 10;
constexpr const size_t NUM_KEEP = 8;
constexpr const int64_t PICK_SCALE = 1'000'000;
constexpr const double POINT_MUTATION_RATE = (0.01 * static_cast<double>(NN_TOTAL_BITS)) / static_cast<double>(NN_TOTAL_BITS);


using input_t = std::bitset<INPUT_BITS>;
using output_t = std::bitset<OUTPUT_BITS>;


struct alignas(64) NN {
  alignas(64) std::array<input_t, LAYER_BITS> layer1;
  alignas(64) std::array<std::bitset<LAYER_BITS>, LAYER_BITS> layer2;
  alignas(64) std::array<std::bitset<LAYER_BITS>, LAYER_BITS> layer3;
  alignas(64) std::array<std::bitset<LAYER_BITS>, OUTPUT_BITS> layer4;
};


// Assumes total image bytes * 8 == N
template <size_t N>
std::bitset<N> InputToBitset(const std::vector<uint8_t>& img_bytes) noexcept {
  std::bitset<N> target_bitset;
  size_t bit_index = 0;
  for (uint8_t pixel : img_bytes) {
    std::bitset<8> pixel_bits{pixel};
    for (size_t i=0; i<8; ++i) {
      target_bitset[bit_index] = pixel_bits[i];
      bit_index++;
    }
  }
  return target_bitset;
}


// Assumes possible label value is <= N
template <size_t N>
std::vector<std::bitset<N>> InputLabelsToBitset(const std::vector<uint32_t>& labels) noexcept {
  std::vector<std::bitset<N>> one_hots(labels.size());
  for (auto label_id : labels) {
    std::bitset<N> one_hot{};
    one_hot.set(static_cast<size_t>(label_id), true);
    one_hots.emplace_back(one_hot);
  }
  return one_hots;
}


// Assumes num_bits less than equal to sizeof(uint_fast64_t) otherwise UB
void UpdateBits(const uint_fast64_t val, const size_t num_bits, auto& target_bitset, size_t& bit_index) noexcept {
  std::bitset<64> bits{val};
  for (size_t i=0; i<num_bits; ++i) {
    target_bitset.set(bit_index, bits[i]);
    bit_index++;
  }
  return;
}


void RandomizeLayer(auto& layer, auto& rng) noexcept {
  const size_t chunks_64 = layer[0].size() / 64;
  const size_t extra = layer[0].size() % 64;
  for (auto& bs : layer) {
    size_t bit_index = 0;
    for (size_t i=0; i<chunks_64; ++i) { UpdateBits(rng(), 64, bs, bit_index); }
    if (extra) { UpdateBits(rng(), extra, bs, bit_index); }
  }
  return;
}


void RandomizeNN(NN& nn, auto& rng) noexcept {
  RandomizeLayer(nn.layer1, rng);
  RandomizeLayer(nn.layer2, rng);
  RandomizeLayer(nn.layer3, rng);
  RandomizeLayer(nn.layer4, rng);
  return;
}


void CopyLayerWithMutation(const auto& src, auto& dst, auto& dist_bi_quantity, auto& dist_u_placement) noexcept {
  for (size_t i=0; i<src.size(); ++i) {
    dst[i] ^= dst[i]; // clear bits
    dst[i] |= src[i]; // copy src bits
    size_t num = static_cast<size_t>(dist_bi_quantity());
    for (size_t j=0; j<num; ++j) {
      size_t bit_index = dist_u_placement();
      dst[i].flip(bit_index);
    }
  }
  return;
}


void CopyWithMutation(const NN& src, NN& dst, auto& dist_bi_inputs, auto& dist_bi_layers, auto& dist_u_inputs, auto& dist_u_layers) noexcept {
  CopyLayerWithMutation(src.layer1, dst.layer1, dist_bi_inputs, dist_u_inputs);
  CopyLayerWithMutation(src.layer2, dst.layer2, dist_bi_layers, dist_u_layers);
  CopyLayerWithMutation(src.layer3, dst.layer3, dist_bi_layers, dist_u_layers);
  CopyLayerWithMutation(src.layer4, dst.layer4, dist_bi_layers, dist_u_layers);
  return;
}


double Evaluate(const NN& nn, const input_t& input, const output_t& one_hot_label) noexcept {
  double score = 0;
  std::bitset<LAYER_BITS> layer_scratch1{};
  std::bitset<LAYER_BITS> layer_scratch2{};
  std::bitset<OUTPUT_BITS> output{};
  for (size_t i=0; i<nn.layer1.size(); ++i) {
    const input_t& row = nn.layer1[i];
    const size_t count = static_cast<size_t>((~(input ^ row)).count());
    const bool sign = count >= HALF_INPUT_BITS;
    layer_scratch1.set(i, sign);
  }
  for (size_t i=0; i<nn.layer2.size(); ++i) {
    const std::bitset<LAYER_BITS>& row = nn.layer2[i];
    const size_t count = static_cast<size_t>((~(layer_scratch1 ^ row)).count());
    const bool sign = count >= HALF_LAYER_BITS;
    layer_scratch2.set(i, sign);
  }
  for (size_t i=0; i<nn.layer3.size(); ++i) {
    const std::bitset<LAYER_BITS>& row = nn.layer3[i];
    const size_t count = static_cast<size_t>((~(layer_scratch2 ^ row)).count());
    const bool sign = count >= HALF_LAYER_BITS;
    layer_scratch1.set(i, sign);
  }
  for (size_t i=0; i<nn.layer4.size(); ++i) {
    const std::bitset<LAYER_BITS>& row = nn.layer4[i];
    const size_t count = static_cast<size_t>((~(layer_scratch1 ^ row)).count());
    const bool sign = count >= HALF_LAYER_BITS;
    output.set(i, sign);
  }
  score = output == one_hot_label ? 1.0 : 0.0;
  return score;
}


void EvaluatePop(std::array<std::pair<double, size_t>, POP_SIZE>& results, const std::array<NN, POP_SIZE>& pop, const std::vector<input_t>& inputs, const std::vector<output_t>& one_hot_labels) noexcept {
  std::for_each(results.begin(), results.end(), [n=0](std::pair<double, size_t>& a) mutable { std::get<1>(a) = n++; return; });
  auto result = results.begin();
  for (const auto& nn : pop) {
    double acc = 0.0;
    for (size_t i=0; i<inputs.size(); ++i) { acc += Evaluate(nn, inputs[i], one_hot_labels[i]); }
    std::get<0>(*result) = acc / static_cast<double>(inputs.size());
    ++result;
  }
  return;
}


std::array<size_t, POP_SIZE> Pick(std::array<std::pair<double, size_t>, POP_SIZE>& results, const size_t num_keep, auto& rng) noexcept {
  std::array<size_t, POP_SIZE> picks{};
  std::array<int64_t, POP_SIZE> weights{};
  std::sort(
      results.begin(), results.end(),
      [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) -> bool { return std::get<0>(a) > std::get<0>(b); });
  std::ranges::transform(
      results.begin(), results.end(), weights.begin(),
      [](const std::pair<double, size_t>& a) -> int64_t {
        const int64_t val = static_cast<int64_t>(std::floor(static_cast<double>(PICK_SCALE) * std::get<0>(a)));
        return (val <= 0) ? 1 : val;
      });
  auto discrete_dist = std::discrete_distribution(weights.begin(), weights.end());
  auto dist = [&rng, &discrete_dist]() -> size_t { return discrete_dist(rng); };
  for (size_t i=0; i<num_keep; ++i) {
    picks[i] = std::get<1>(results[i]);
  }
  for (size_t i=num_keep; i<results.size(); ++i) {
    size_t pick = static_cast<size_t>(dist());
    picks[i] = std::get<1>(results[pick]);
  }
  return picks;
}


void Replicate(const std::array<NN, POP_SIZE>& src, std::array<NN, POP_SIZE>& dst, const std::array<size_t, POP_SIZE>& rep_ids, auto& rng) noexcept {
  auto dist_noop = []() -> int64_t { return 0; };
  std::binomial_distribution<int64_t> dist_bi_inputs(INPUT_BITS, POINT_MUTATION_RATE);
  std::binomial_distribution<int64_t> dist_bi_layers(LAYER_BITS, POINT_MUTATION_RATE);
  std::uniform_int_distribution<uint64_t> dist_u_inputs(0, INPUT_BITS-1);
  std::uniform_int_distribution<uint64_t> dist_u_layers(0, LAYER_BITS-1);
  auto dbi = [&rng, &dist_bi_inputs]() { return dist_bi_inputs(rng); };
  auto dbl = [&rng, &dist_bi_layers]() { return dist_bi_layers(rng); };
  auto dui = [&rng, &dist_u_inputs]() { return dist_u_inputs(rng); };
  auto dul = [&rng, &dist_u_layers]() { return dist_u_layers(rng); };
  auto ids_it = rep_ids.cbegin();
  auto ids_end = rep_ids.cend();
  auto dst_it = dst.begin();
  for (size_t i=0; i<NUM_KEEP; ++i, ++ids_it, ++dst_it) {
    CopyWithMutation(src[*ids_it], *dst_it, dist_noop, dist_noop, dist_noop, dist_noop);
  }
  for (; ids_it != ids_end; ++ids_it, ++dst_it) {
    CopyWithMutation(src[*ids_it], *dst_it, dbi, dbl, dui, dul);
  }
  return;
}


int Train(const std::vector<std::vector<uint8_t>>& imgs, const std::vector<uint32_t>& labels) noexcept {
  std::chrono::high_resolution_clock clock{};

  std::random_device rd{};
  std::seed_seq seed{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
  std::mt19937_64 rng{seed};

  auto t = clock.now();
  std::vector<input_t> training_inputs(imgs.size());
  std::ranges::transform(imgs, std::make_move_iterator(training_inputs.begin()),
                        [](const std::vector<uint8_t>& img) -> input_t { return InputToBitset<INPUT_BITS>(img); });
  std::vector<output_t> training_one_hots = InputLabelsToBitset<OUTPUT_BITS>(labels);
  auto dt = clock.now() - t;
  auto total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.0;
  std::cout << "Transform inputs (" << total << " sec)" << std::endl;

  t = clock.now();
  size_t current_pop_index = 0;
  std::unique_ptr<std::array<std::array<NN, POP_SIZE>, 2>> pops{new (std::nothrow) std::array<std::array<NN, POP_SIZE>, 2>{}};
  for (NN& nn : (*pops.get())[current_pop_index]) { RandomizeNN(nn, rng); }
  std::array<std::pair<double, size_t>, POP_SIZE> results;
  dt = clock.now() - t;
  total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.0;
  std::cout << "Initialize pop (" << total << " sec)" << std::endl;

  t = clock.now();
  for (size_t generation=0; generation<NUM_GENERATIONS; ++generation) {
    auto t1 = clock.now();
    size_t next_pop_index = (current_pop_index + 1) % 2;
    const std::array<NN, POP_SIZE>& pop = (*pops.get())[current_pop_index];
    std::array<NN, POP_SIZE>& next_pop = (*pops.get())[next_pop_index];
    EvaluatePop(results, pop, training_inputs, training_one_hots);
    std::array<size_t, POP_SIZE> rep_ids = Pick(results, NUM_KEEP, rng);
    Replicate(pop, next_pop, rep_ids, rng);
    current_pop_index = next_pop_index;
    auto dt1 = clock.now() - t1;
    double total1 = std::chrono::duration_cast<std::chrono::microseconds>(dt1).count() / 1000000.0;
    std::cout << "Gen " << generation << " complete  (" << total1 << " sec)" << std::endl;
  }
  dt = clock.now() - t;
  total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.0;
  std::cout << "Total train time (" << total << " sec)" << std::endl;

  return 0;
}


int main(void) {
  std::cout << "Load MNIST data ... ";
  data::Data training_data{};
  if (!mnist::LoadTraining(training_data)) {
    std::cout << std::endl;
    std::cout << "Failed to load training data." << std::endl;
    return 1;
  }
  data::Data testing_data{};
  if (!mnist::LoadTraining(testing_data)) {
    std::cout << std::endl;
    std::cout << "Failed to load testing data." << std::endl;
    return 1;
  }
  std::cout << "done." << std::endl;
  std::cout << "Training data size- " << training_data.images.size() << std::endl;
  std::cout << "Testing data size- " << testing_data.images.size() << std::endl;
  return Train(training_data.images, training_data.labels);
}
