//g++ exp3.cpp -std=c++20 -O3 -march=native -mtune=native -mno-avx256-split-unaligned-load -o bin/exp3
//g++ exp3.cpp -std=c++20 -O3 -fno-trapping-math -fno-math-errno -mprefer-vector-width=256 -march=native -mtune=native -o bin/exp3

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

#include <chrono>
#include <iostream>
#include <bitset>


namespace jms {
namespace bytes {


template <typename T>
concept Spannable_c = requires (T t) { {std::span{t}}; };


constexpr const size_t CACHE_SIZE = 64;
constexpr const size_t BITS_PER_BYTE = 8;


template <typename T>
constexpr const size_t TToBits(const size_t num_T) noexcept { return num_T * sizeof(T) * BITS_PER_BYTE; }
template <typename T>
constexpr const size_t BitsToT(const size_t num_bits) noexcept { return (num_bits / TToBits<T>(1)) + (num_bits % TToBits<T>(1) ? 1 : 0); }
constexpr std::tuple<size_t, size_t> BitToByteLoc(const size_t bit) noexcept { return {bit / 8, bit % 8}; }
constexpr size_t BitsToBytes(const size_t num_bits) noexcept { return BitsToT<std::byte>(num_bits); }
constexpr bool ByteHasBit(const std::byte byte_mask, const std::byte bit_mask) noexcept { return std::to_integer<bool>(byte_mask & bit_mask); }
constexpr size_t BytesToBits(const size_t num_bytes) noexcept { return TToBits<std::byte>(num_bytes); }


template <typename T, size_t num_bits=BytesToBits(sizeof(T))>
requires std::is_default_constructible_v<T>
constexpr T SetBit(const size_t bit_index) noexcept {
  if (bit_index >= num_bits) { return T{}; }
  std::array<std::byte, sizeof(T)> bytes{};
  const size_t byte_index = bit_index / 8;
  const size_t local_bit_index = bit_index % 8;
  std::byte b{0x80};
  if (local_bit_index) { b >>= local_bit_index; }
  bytes[byte_index] = b;
  return std::bit_cast<T>(bytes);
}


template <typename T, size_t num_bits=BytesToBits(sizeof(T))>
requires std::is_default_constructible_v<T>
constexpr const std::array<T, num_bits> GenBitMask() noexcept {
  std::array<T, num_bits> data{};
  std::ranges::for_each(data, [n=0](T& t) mutable { t = SetBit<T>(n++); });
  return data;
}


template <typename T, size_t num_bits=BytesToBits(sizeof(T))>
requires std::is_default_constructible_v<T>
constexpr const std::array<T, num_bits> GenByteMask() noexcept {
  std::array<T, num_bits> data{};
  std::ranges::generate(data, [n=0, val=T{}]() mutable { return val |= SetBit<T>(n++); });
  std::rotate(data.rbegin(), std::next(data.rbegin()), data.rend());
  return data;
}


// BitMask represents which bit is set; uses 0-based index
// 0x80, 0x40, ..., 0x01
constexpr const auto BitMask_byte = GenBitMask<std::byte>();
//constexpr const auto BitMask_uint16 = GenBitMask<uint16_t>();
//constexpr const auto BitMask_uint32 = GenBitMask<uint32_t>();
//constexpr const auto BitMask_uint64 = GenBitMask<uint64_t>();


// ByteMask represents all valid bits; index zero is all bits then proceeding left to right
// 0xff, 0x80, 0xc0 ..., 0xfe
constexpr const auto ByteMask_byte = GenByteMask<std::byte>();
//constexpr const auto ByteMask_uint16 = GenByteMask<uint16_t>();
//constexpr const auto ByteMask_uint32 = GenByteMask<uint32_t>();
//constexpr const auto ByteMask_uint64 = GenByteMask<uint64_t>();


// Returns the total bytes needed to encompass N bytes in terms of align_sized units and optionally
// enough room to ensure the data can be offset to start at a memory address aligned to the align_size.
struct alignas(32) AlignmentDetails {
  size_t align_size;
  size_t boundary_bytes; // front padding
  size_t num_bytes;
  size_t padding;        // back padding
  size_t total_bytes;

  AlignmentDetails() noexcept = default;
  AlignmentDetails(const size_t num_bytes_in, const size_t align_size_in, const bool ensure_boundary) noexcept
  : align_size{align_size_in ? align_size_in : 1},
    // could subtract 1 from align_size; more accurate but overall number of bytes allocated is better aligned without subtraction.
    // 256 bytes aligned on 64 byte boundary with ensure_boundary: (-1 -> 319 bytes) (-0 -> 320 bytes)
    boundary_bytes{ensure_boundary && num_bytes_in ? align_size : 0},
    num_bytes{num_bytes_in},
    padding{(num_bytes % align_size) ? (align_size - (num_bytes % align_size)) : 0},
    total_bytes{num_bytes + padding + boundary_bytes}
  {}
};


template <size_t align_size=1, bool ensure_boundary=false>
struct AlignedArray {
  using value_type = std::byte;

  std::unique_ptr<std::byte[]> buffer{};
  std::span<std::byte> bytes{};

  AlignedArray() noexcept = default;
  AlignedArray(const size_t num_bytes) noexcept {
    AlignmentDetails details{num_bytes, align_size, ensure_boundary};
    if (!details.total_bytes) { return; }
    buffer.reset(new (std::nothrow) std::byte[details.total_bytes]());
    if (buffer.get() == nullptr) { return; }
    std::byte* start{nullptr};
    if (ensure_boundary && details.boundary_bytes) {
      void* ptr = buffer.get(); // modified by std::align
      size_t total_bytes = details.total_bytes; // modified by std::align
      start = static_cast<std::byte*>(std::align(align_size, num_bytes, ptr, total_bytes));
    } else {
      start = buffer.get();
    }
    bytes = std::span{start, num_bytes};
  }
  // data/size methods are required to make this structure interchangeable with std::vector/other containers
  const std::byte* data() const noexcept { return std::assume_aligned<align_size>(bytes.data()); }
        std::byte* data()       noexcept { return std::assume_aligned<align_size>(bytes.data()); }
  size_t size() const noexcept { return bytes.size(); }
  constexpr const size_t Alignment() const noexcept { return align_size; }
  constexpr const bool EnsureBoundary() const noexcept { return ensure_boundary; }
};


// Cheating; using this instead of propagating a template from BitArray through main (messy).
// TODO: Cleanup if pursued.  At this point, AlignedArray is over 2x faster for layer sizes approximately or greater than a cache line.
using Buffer_t = AlignedArray<CACHE_SIZE, true>;
//using Buffer_t = AlignedArray<CACHE_SIZE, true>;
//using Buffer_t = std::vector<std::byte>;


struct BitArray {
  Buffer_t buffer{};
  size_t num_bits{0};

  BitArray() noexcept = default;
  BitArray(const size_t num_bits) noexcept : buffer{BitsToT<Buffer_t::value_type>(num_bits)}, num_bits(buffer.data() != nullptr ? num_bits : 0) {}
  bool operator!(void) const noexcept { return buffer.data() == nullptr; }
};


// Pass by value; constructor takes std::vector, std::array or other type and auto converts to view format.  Pass around/copy should be cheap so no references.
// https://godbolt.org/z/4vj38W3cK
struct BitViewWritable {
  std::span<std::byte> bytes{};
  size_t num_bits{0};

  BitViewWritable(void) noexcept = default;
  BitViewWritable(Spannable_c auto& i) noexcept : bytes{std::as_writable_bytes(std::span{i})}, num_bits(BytesToBits(bytes.size())) {}
  BitViewWritable(BitArray& i) noexcept : bytes{i.buffer.data(), i.buffer.size()}, num_bits{i.num_bits} {}
  BitViewWritable(std::byte* bytes, const size_t num_bits) noexcept : bytes{bytes, BitsToBytes(num_bits)}, num_bits(num_bits) {}
};


struct BitView {
  std::span<const std::byte> bytes{};
  size_t num_bits{0};

  BitView(void) noexcept = default;
  BitView(Spannable_c auto& i) noexcept : bytes{std::as_bytes(std::span{ i})}, num_bits(BytesToBits(bytes.size())) {}
  BitView(Spannable_c auto* i) noexcept : bytes{std::as_bytes(std::span{*i})}, num_bits(BytesToBits(bytes.size())) {}
  BitView(const BitArray& i) noexcept : bytes{i.buffer.AsBytes().subspan(0, BitsToBytes(i.num_bits))}, num_bits{i.num_bits} {}
  BitView(BitViewWritable i) noexcept : bytes{i.bytes}, num_bits{i.num_bits} {}
  BitView(const std::byte* bytes, const size_t num_bits) noexcept : bytes{bytes, BitsToBytes(num_bits)}, num_bits(num_bits) {}
};


struct BitMatrix {
  size_t input_dim_bits{0};
  size_t output_dim_bits{0};
  Buffer_t buffer{};
  std::vector<std::span<Buffer_t::value_type>> views{};

  BitMatrix() noexcept = default;
  BitMatrix(const size_t input_dim_bits_in, const size_t output_dim_bits_in) noexcept {
    if (!input_dim_bits_in || !output_dim_bits_in) { return; }
    const size_t num_units = BitsToT<Buffer_t::value_type>(input_dim_bits_in);
    const size_t total_unit_size = num_units * output_dim_bits_in;
    Buffer_t matrix{total_unit_size};
    if (matrix.data() == nullptr) { return; }
    std::vector<std::span<Buffer_t::value_type>> local_views{};
    local_views.reserve(output_dim_bits_in);
    for (size_t i=0; i<total_unit_size; i+=num_units) { local_views.emplace_back(matrix.view.subspan(i, num_units)); }
    input_dim_bits = input_dim_bits_in;
    output_dim_bits = output_dim_bits_in;
    buffer = std::move(matrix);
    views = std::move(local_views);
  }
  bool operator!(void) const noexcept { return buffer.data() == nullptr; }
};


/*
// Not really certain about the matrix views; if they do anything or not between writable or not; const BitViewWritable???
struct BitMatrixViewWritable {
  const std::span<BitViewWritable> views{};

  BitMatrixViewWritable() noexcept = default;
  BitMatrixViewWritable(BitMatrix& i) noexcept : views{i.views} {}
};


struct BitMatrixView {
  const std::span<const BitViewWritable> views{};

  BitMatrixView() noexcept = default;
  BitMatrixView(const BitMatrix& i) noexcept : views{i.views} {}
};
*/


template <typename T>
requires std::is_arithmetic_v<T>
struct alignas(CACHE_SIZE) CacheChunk {
  using value_type = std::remove_cv_t<T>;
  static_assert(sizeof(value_type) <= CACHE_SIZE && CACHE_SIZE % sizeof(value_type) == 0);

  constexpr static const size_t N = CACHE_SIZE / sizeof(value_type);
  value_type data[N];

        std::span<      std::byte> AsBytes(void)       noexcept { return std::as_writable_bytes(std::span{data}); }
  const std::span<const std::byte> AsBytes(void) const noexcept { return std::as_bytes(std::span{data}); }
        std::span<      value_type> AsSpan(void)       noexcept { return std::span{data}; }
  const std::span<const value_type> AsSpan(void) const noexcept { return std::span{data}; }
};


template <typename Rng_t>
class RandomByteGenerator {
  size_t n;
  Rng_t& rng;
  Rng_t::result_type value;
  std::span<const std::byte> bytes;

public:
  RandomByteGenerator(Rng_t& rng) noexcept : n{0}, rng{rng}, value{rng()}, bytes{std::as_bytes(std::span{&value, 1})} {}
  std::byte operator()(void) noexcept {
    std::byte v = bytes[n];
    n = (n + 1) % sizeof(typename Rng_t::result_type);
    if (!n) { value = rng(); }
    return v;
  }
};


bool operator==(BitView v1, BitView v2) noexcept {
  if (v1.num_bits != v2.num_bits || v1.bytes.size() != v2.bytes.size()) { return false; }
  const size_t remaining_bits = v1.num_bits % 8;
  for (size_t i=0; i<v1.bytes.size() - 1; ++i) { if (v1.bytes[i] != v2.bytes[i]) { return false; } } // TODO: Convert to find_if_not
  if ((v1.bytes.back() & ByteMask_byte[remaining_bits]) != (v2.bytes.back() & ByteMask_byte[remaining_bits])) { return false; }
  return true;
}


inline void Copy(BitView src, BitViewWritable dst) noexcept {
  for (size_t i=0; i<src.bytes.size() - 1; ++i) { dst.bytes[i] = src.bytes[i]; }
  const size_t remaining_bits = src.num_bits % 8;
  dst.bytes.back() = src.bytes.back() & ByteMask_byte[remaining_bits];
}


//inline void Copy(BitMatrixView src, BitMatrixViewWritable dst) noexcept {
inline void Copy(const BitMatrix& src, BitMatrix& dst) noexcept {
  //for (size_t i=0; i<src.views.size(); ++i) { Copy(src.views[i], dst.views[i]); }
  for (size_t i=0; i<src.views.size(); ++i) {
    std::span<const std::byte> src_bytes = std::as_bytes(src.views[i]).subspan(0, BitsToBytes(src.input_dim_bits));
    std::span<std::byte> dst_bytes = std::as_writable_bytes(dst.views[i]).subspan(0, BitsToBytes(dst.input_dim_bits));
    Copy(BitView{src_bytes.data(), src.input_dim_bits}, BitViewWritable{dst_bytes.data(), dst.input_dim_bits});
  }
}


std::tuple<const size_t, const std::byte> FindBit(BitView view, const size_t index) noexcept {
  const auto [byte_index, bit_index] = BitToByteLoc(index);
  // TODO there must be a better check.
  if (byte_index < view.bytes.size()) {
    // Only have to check bit inclusion for last byte.
    if (byte_index < view.bytes.size() - 1 || ByteHasBit(ByteMask_byte[view.num_bits % 8], BitMask_byte[bit_index])) {
      return {byte_index, BitMask_byte[bit_index]};
    }
  }
  return {view.bytes.size(), std::byte{0}};
}


std::optional<bool> Flip(BitViewWritable view, const size_t index) noexcept {
  const auto [byte_index, bit_mask] = FindBit(view, index);
  if (byte_index >= view.bytes.size()) { return std::nullopt; }
  std::byte& target_byte = view.bytes[byte_index];
  const bool flipped_value = !std::to_integer<bool>(target_byte & bit_mask);
  std::byte others = target_byte & ~bit_mask;
  if (!flipped_value) { target_byte = others; }
  else { target_byte = others | bit_mask; }
  return flipped_value;
}


std::optional<bool> Get(BitView view, const size_t index) noexcept {
  const auto [byte_index, bit_mask] = FindBit(view, index);
  if (byte_index >= view.bytes.size()) { return std::nullopt; }
  return std::to_integer<bool>(view.bytes[byte_index] & bit_mask);
}



void Randomize(BitViewWritable view, auto& rng) noexcept {
  if (!view.num_bits) { return; }
  RandomByteGenerator gen{rng};
  std::ranges::for_each(view.bytes, [&gen](std::byte& x) { x = gen(); });
  const size_t remaining_bits = view.num_bits % 8;
  view.bytes.back() &= ByteMask_byte[remaining_bits];
}


//inline void Randomize(BitMatrixViewWritable mat, auto& rng) noexcept {
inline void Randomize(BitMatrix& mat, auto& rng) noexcept {
  //std::ranges::for_each(mat.views, [&rng](BitViewWritable view) { Randomize(view, rng); });
  std::ranges::for_each(
      mat.views,
      [num_bits=mat.input_dim_bits, num_bytes=BitsToBytes(mat.input_dim_bits), &rng](std::span<Buffer_t::value_type> view) {
        std::span<std::byte> bytes = std::as_bytes(view).subspan(0, num_bytes);
        Randomize(BitViewWritable{bytes.data(), num_bits}, rng);
      });
}


std::optional<bool> Set(BitViewWritable view, const size_t index, bool value) noexcept {
  const auto [byte_index, bit_mask] = FindBit(view, index);
  if (byte_index >= view.bytes.size()) { return std::nullopt; }
  std::byte& target_byte = view.bytes[byte_index];
  std::byte others = target_byte & ~bit_mask;
  if (!value) { target_byte = others; }
  else { target_byte = others | bit_mask; }
  return value;
}


/*
void Eval(BitView input, const BitMatrix& mat) {
  if (input.num_bits != mat.input_dim_bits) { return; }
  
  return;
}
*/

void F(const size_t input_dim_bits, const size_t output_dim_bits, const std::vector<size_t>& layer_dim_bits) {
  struct LayerSize { size_t N, M; };
  auto all_dims_list = {{input_dim_bits}, layer_dim_bits, {output_dim_bits}};
  auto all_dims_range = all_dims_list | std::views::join;
  std::vector<size_t> all_dims(all_dims_range.begin(), all_dims_range.end());
  std::vector<LayerSize> layer_sizes(all_dims.size() - 1);
  std::ranges::transform(all_dims | std::ranges::views::take(layer_sizes.size()),
                         all_dims | std::ranges::views::drop(1),
                         layer_sizes.begin(),
                         [](const auto input_dim, const auto output_dim) -> LayerSize {
                           return {input_dim, output_dim};
                         });
  for (auto& i : layer_sizes) { std::cout << "(" << i.N << ", " << i.M << ")" << std::endl; }
}
void G(const size_t in_bits, const size_t out_bits) {
  const size_t chunks_512 = in_bits / 512; // 0 .. N
  size_t remaining_bits = in_bits % 512;
  const size_t chunk_256 = remaining_bits / 256; // 0 or 1
  remaining_bits = remaining_bits % 256;
  const size_t chunk_128 = remaining_bits / 128; // 0 or 1
  remaining_bits = remaining_bits % 128;
  const size_t chunk_64 = remaining_bits / 64; // 0 or 1
  remaining_bits = remaining_bits % 64;
  const size_t chunk_32 = remaining_bits / 32; // 0 or 1
  remaining_bits = remaining_bits % 32;
  const size_t chunk_16 = remaining_bits / 16; // 0 or 1
  remaining_bits = remaining_bits % 16;
  const size_t chunk_8 = (remaining_bits / 8); // 0 or 1
  remaining_bits = remaining_bits % 8;
  const size_t chunk_remaining = remaining_bits ? 1 : 0; // 0 or 1
  // mask = MASK[remaining_bits];

  // chunks_512 -> (chunks_512 * 512)
  // chunk_256 -> ((M / 2) * 512) + ((M % 2) ? 256 : 0)
  // chunk_128 -> ((M / 4) * 512) + ((M % 4) * 128)
  // chunk_64  -> ((M / 8) * 512) + ((M % 8) * 64)
  // chunk_32  -> ((M / 16) * 512) + ((M % 16) * 32)
  // chunk_16  -> ((M / 32) * 512) + ((M % 32) * 16)
  // chunk_8   -> ((M / 64) * 512) + ((M % 64) * 8)
  // chunk_remaining -> ((M / 64) * 512) + ((M % 64) * 8)
}

constexpr const uint64_t c1 = 0x5555555555555555;
constexpr const uint64_t c2 = 0x3333333333333333;
constexpr const uint64_t c4 = 0x0F0F0F0F0F0F0F0F;
constexpr const uint64_t c8 = 0x00FF00FF00FF00FF;
constexpr const uint64_t c16 = 0x0000FFFF0000FFFF;
constexpr const uint64_t c32 = 0x00000000FFFFFFFF;
uint64_t bitcount ( uint64_t x) {
  x = (x & c1) + ((x >> 1) & c1) ;
  x = (x & c2) + ((x >> 2) & c2) ;
  //return (x & c4) + ((x >> 4) & c4) ;
  x = (x & c4) + ((x >> 4) & c4) ;
  x = (x & c8) + ((x >> 8) & c8) ;
  x = (x & c16 ) + ((x >> 16) & c16) ;
  return (x & c32 ) + ((x >> 32) & c32) ;
}


int64_t XnorCountOnes(std::span<const Buffer_t::value_type> v1, std::span<const Buffer_t::value_type> v2, const size_t num_bits) noexcept {
  int count = 0;
  const size_t bits_per_unit = BytesToBits(sizeof(Buffer_t::value_type));
  const size_t num_units = num_bits / bits_per_unit;
  const size_t remaining_bits = num_bits % bits_per_unit;
  for (size_t i=0; i<num_units; ++i) {
    count += std::popcount(~(v1[i] ^ v2[i]));
  }
  if (remaining_bits) {
    std::span<const std::byte> bytes1 = std::as_bytes(v1).subspan(num_units * sizeof(Buffer_t::value_type));
    std::span<const std::byte> bytes2 = std::as_bytes(v2).subspan(num_units * sizeof(Buffer_t::value_type));
    if (remaining_bits % 8 == 0) {
      for (size_t i=0; i<bytes1.size(); ++i) { count += std::popcount(static_cast<uint8_t>(~(bytes1[i] ^ bytes2[i]))); }
    } else {
      for (size_t i=0; i<bytes1.size()-1; ++i) { count += std::popcount(static_cast<uint8_t>(~(bytes1[i] ^ bytes2[i]))); }
      count += std::popcount(static_cast<uint8_t>( (~(bytes1.back() ^ bytes2.back())) & ByteMask_byte[remaining_bits % 8]  ));
    }
  }
  return count;
}

/*
int64_t XnorCountOnes(BitView v1, BitView v2) noexcept {
  int64_t count = 0;
  CacheChunk<uint_fast16_t> scratch{};
  const size_t num_chunks = v1.bytes.size() / CACHE_SIZE;
  auto it1=v1.bytes.begin(), it2=v2.bytes.begin();
  for (auto _ : std::views::iota(static_cast<size_t>(0), num_chunks)) {
    for (std::byte& b : scratch.AsBytes()) { b = ~(*it1++ ^ *it2++); }
    for (uint_fast16_t& x : scratch.AsSpan()) { count += static_cast<int64_t>(std::popcount(x)); }
  }
  for (; it1 != v1.bytes.end(); ++it1, ++it2) {
    count += static_cast<int64_t>(std::popcount(static_cast<uint8_t>(~(*it1 ^ *it2))));
  }
  const size_t remaining_bits = v1.num_bits % 8;
  const std::byte last_byte_mask = ByteMask_byte[remaining_bits];
  if (v1.bytes.size() && last_byte_mask != std::byte{0xff}) {
    std::byte final_xnor = ~(v1.bytes.back() ^ v2.bytes.back());
    std::byte final_xnor_masked = final_xnor & last_byte_mask;
    count -= static_cast<int64_t>(std::popcount(static_cast<uint8_t>(final_xnor)) - std::popcount(static_cast<uint8_t>(final_xnor_masked)));
  }
  return count;
}
*/


//void Evaluate(BitView input, BitMatrixView mat, BitViewWritable out) noexcept {
void Evaluate(BitView input, const BitMatrix& mat, BitViewWritable out) noexcept {
  const int64_t half_bits = static_cast<int64_t>(input.num_bits / 2);
  auto views_it = mat.views.begin();
  for (std::byte& target_byte : out.bytes) {
    std::byte b{0};
    for (size_t bit_index=0; bit_index < 8 && views_it != mat.views.end(); ++bit_index, ++views_it) {
      //b |= BitMask_byte[bit_index] & (XnorCountOnes(input, *views_it) >= half_bits ? std::byte{0xff} : std::byte{0});
      b |= BitMask_byte[bit_index] & (XnorCountOnes(input, *views_it) >= half_bits ? std::byte{0xff} : std::byte{0});
    }
    target_byte = b;
  }
}


} // namespace bytes


namespace nn {


std::vector<jms::bytes::BitArray> InputLabelsToOneHot(const std::vector<uint32_t>& labels) noexcept {
  std::vector<jms::bytes::BitArray> one_hots(labels.size());
  const uint32_t max_label = std::ranges::max(labels);
  const size_t num_bits = static_cast<const size_t>(max_label + 1);
  std::ranges::transform(labels, std::make_move_iterator(one_hots.begin()), [num_bits](auto label_id) -> jms::bytes::BitArray {
    jms::bytes::BitArray v{num_bits};
    jms::bytes::Set(v, static_cast<const size_t>(label_id), true);
    return v;
  });
  return one_hots;
}


class Dataset {
public:
  using data_t = std::tuple<std::span<const std::byte>, jms::bytes::BitView>;

private:
  std::vector<jms::bytes::BitArray> one_hot_labels;
  std::vector<data_t> data;
  std::span<data_t> span_data;
  size_t span_index{0};

public:
  Dataset(const std::vector<std::vector<uint8_t>>& images, const std::vector<uint32_t>& labels) noexcept
  : one_hot_labels{InputLabelsToOneHot(labels)}, data(images.size()) {
    //if (images.size() != labels.size()) { throw std::runtime_error("Dataset images and labels not the same size."); }
    std::ranges::transform(images, one_hot_labels, std::make_move_iterator(data.begin()),
                           [](const std::vector<uint8_t>& img, jms::bytes::BitView label) -> data_t {
                             return data_t{std::as_bytes(std::span{img}), label};
                           });
    span_data = std::span<data_t>{data};
  }

  std::optional<std::span<data_t>> Batch(const size_t batch_size) noexcept {
    if (span_index >= span_data.size()) { return std::nullopt; }
    size_t num = (span_index + batch_size > span_data.size()) ? span_data.size() - span_index : batch_size;
    // Temporary for training on the same images each step.
    //std::span<data_t> batch{span_data.subspan(span_index, num)};
    std::span<data_t> batch{span_data.subspan(0, batch_size)};
    span_index += num;
    return batch;
  }
  size_t InputDim(void) const noexcept { return span_data.empty() ? 0 : std::get<0>(span_data.front()).size() * 8; }
  size_t OutputDim(void) const noexcept { return one_hot_labels.empty() ? 0 : one_hot_labels[0].num_bits; }
  void ResetBatches(void) noexcept { span_index = 0; }
  void Shuffle(auto& rng) noexcept { std::ranges::shuffle(data, rng); }
};


struct DenseBitLayer {
  size_t input_dim_bits{0};
  size_t output_dim_bits{0};
  jms::bytes::BitArray output{};
  jms::bytes::BitMatrix matrix{};

  DenseBitLayer() noexcept = default;
  DenseBitLayer(const size_t input_dim_bits_in, const size_t output_dim_bits_in) noexcept {
    jms::bytes::BitArray v{output_dim_bits_in};
    jms::bytes::BitMatrix m{input_dim_bits_in, output_dim_bits_in};
    if (!v || !m) { return; }
    input_dim_bits = input_dim_bits_in;
    output_dim_bits = output_dim_bits_in;
    output = std::move(v);
    matrix = std::move(m);
  }
  bool operator!(void) const noexcept { return !output || !matrix; }
};


// TODO: Need to decide later on dynamic sizing and amount of bounds checking.
class BNN {
  std::vector<DenseBitLayer> layers;

public:
  BNN(const size_t input_dim_bits, const size_t output_dim_bits, const std::vector<size_t>& layer_dim_bits) noexcept
  : layers(layer_dim_bits.size() + 1) {
    auto all_dims_list = {{input_dim_bits}, layer_dim_bits, {output_dim_bits}};
    auto all_dims_range = all_dims_list | std::views::join;
    std::vector<size_t> all_dims(all_dims_range.begin(), all_dims_range.end());
    std::ranges::transform(all_dims | std::ranges::views::take(layers.size()),
                           all_dims | std::ranges::views::drop(1),
                           layers.begin(),
                           [](const auto input_dim, const auto output_dim) -> DenseBitLayer {
                             return {input_dim, output_dim};
                           });
  }

  bool operator!(void) const noexcept {
    return std::ranges::find_if(layers, [](const DenseBitLayer& layer) -> bool { return !layer; }) != layers.end();
  }

  void Copy(const BNN& src) noexcept {
    for (size_t i=0; i<layers.size(); ++i) { jms::bytes::Copy(src.layers[i].matrix, layers[i].matrix); }
    //std::ranges::copy(src.layers, layers,
    //                  [](const DenseBitLayer& lhs, DenseBitLayer& rhs) { jms::bytes::Copy(lhs.matrix, rhs.matrix); });
  }

  void CopyWithMutation(const BNN& src, double mutation_rate, auto& rng) noexcept {
    Copy(src);
    if (mutation_rate <= 0.0) { return; }
    for (DenseBitLayer& layer : layers) {
      std::binomial_distribution<uint64_t> bin_dist{layer.input_dim_bits, mutation_rate};
      std::uniform_int_distribution<uint64_t> u_dist{0, layer.input_dim_bits};
      auto dist_b = [&rng, &bin_dist]() mutable -> uint64_t { return bin_dist(rng); };
      auto dist_u = [&rng, &u_dist]() mutable -> uint64_t { return u_dist(rng); };
      for (auto& view : layer.matrix.views) {
        const uint64_t how_many = dist_b();
        for (size_t i=0; i<how_many; ++i) {
          jms::bytes::Flip(view, dist_u());
        }
      }
    }
  }

  size_t DimInput(void) const noexcept { return layers.front().input_dim_bits; }
  size_t DimOutput(void) const noexcept { return layers.back().output_dim_bits; }

  jms::bytes::BitView Evaluate(jms::bytes::BitView input) noexcept {
    jms::bytes::BitView view{input};
    for (DenseBitLayer& layer : layers) {
      jms::bytes::Evaluate(view, layer.matrix, layer.output);
      view = layer.output;
    }
    return view;
  }

  void Randomize(auto& rng) noexcept {
    for (DenseBitLayer& layer : layers) { jms::bytes::Randomize(layer.matrix, rng); }
  }

  double Score(jms::bytes::BitView input, jms::bytes::BitView one_hot_label) noexcept {
    jms::bytes::BitView output = Evaluate(input);
    double score = output == one_hot_label ? 1.0 : 0.0;
    return score;
  }
};


} // namespace nn


namespace evolve {


std::vector<double> EvaluatePop(std::span<jms::nn::BNN> pop, std::span<typename jms::nn::Dataset::data_t> batch) noexcept {
  std::vector<double> results{};
  results.reserve(pop.size());
  for (jms::nn::BNN& nn : pop) {
    double acc = std::accumulate(batch.begin(), batch.end(), 0.0,
        [&nn](double total, typename jms::nn::Dataset::data_t& data) mutable -> double {
          auto [img, one_hot_label] = data;
          return total + nn.Score(img, one_hot_label);
        });
    results.push_back(acc / static_cast<double>(batch.size()));
  }
  return results;
}


std::vector<size_t> Pick(std::vector<std::tuple<size_t, double>>& results, const size_t num_keep,
                         const size_t pick_scale, auto& rng) noexcept {
  std::vector<size_t> picks(results.size());
  std::vector<int64_t> weights(results.size());
  std::ranges::sort(results, [](const auto& a, const auto& b) -> bool { return std::get<1>(a) > std::get<1>(b); });
  std::ranges::transform(
      results, weights.begin(),
      [pick_scale](const auto& a) -> int64_t {
        const auto [_, score] = a;
        const int64_t val = static_cast<int64_t>(std::floor(static_cast<double>(pick_scale) * score));
        return (val <= 0) ? 1 : val;
      });
  auto discrete_dist = std::discrete_distribution(weights.begin(), weights.end());
  auto dist = [&rng, &discrete_dist]() mutable -> uint64_t { return discrete_dist(rng); };
  std::span<size_t> span_out{picks};
  std::span<std::tuple<size_t, double>> span_in{results};
  std::ranges::transform(span_in.subspan(0, num_keep), span_out.subspan(0, num_keep).begin(),
                         [](const auto& a) -> size_t { return std::get<0>(a); });
  std::ranges::generate(span_out.subspan(num_keep), [&dist, &results]() mutable -> size_t {
    size_t pick = static_cast<size_t>(dist());
    return std::get<0>(results[pick]);
  });
  return picks;
}


void Replicate(std::span<const jms::nn::BNN> src, std::span<jms::nn::BNN> dst, std::span<const size_t> rep_ids,
               const size_t num_keep, const double mutation_rate, auto& rng) noexcept {
  std::vector<int> dummy_out(dst.size());
  std::ranges::transform(dst.subspan(0, num_keep), rep_ids.subspan(0, num_keep), dummy_out.begin(),
                         [&src](auto& nn, size_t i) -> int {
                           nn.Copy(src[i]); return i;
                          });
  std::ranges::transform(dst.subspan(num_keep), rep_ids.subspan(num_keep), dummy_out.begin(),
                         [&src, mutation_rate, &rng](auto& nn, size_t i) mutable -> int {
                           nn.CopyWithMutation(src[i], mutation_rate, rng); return i;
                         });
}


} // namespace evolve
} // namespace jms


#include "data.h"
#include "mnist.h"


constexpr const size_t LAYER_BITS = 512;//512;// * 32;

constexpr const size_t POP_SIZE = 128;
constexpr const size_t NUM_GENERATIONS = 1;//50;//10;
constexpr const size_t BATCH_SIZE = 256;//60000;
constexpr const size_t NUM_KEEP = 8;//16;//10;
constexpr const int64_t PICK_SCALE = 1'000'000;
constexpr const double POINT_MUTATION_RATE = 0.01;


int Train(const std::vector<std::vector<uint8_t>>& imgs, const std::vector<uint32_t>& labels) noexcept {
  std::chrono::high_resolution_clock clock{};

  std::random_device rd{};
  std::seed_seq seed{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
  std::mt19937_64 rng{seed};

  auto t = clock.now();
  jms::nn::Dataset dataset{imgs, labels};
  for (int i=0; i<50; ++i) { dataset.Shuffle(rng); }
  auto dt = clock.now() - t;
  auto total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.0;
  std::cout << "Transform inputs (" << total << " sec)" << std::endl;

  t = clock.now();
  size_t current_pop_index = 0;
  std::vector<size_t> layer_sizes{LAYER_BITS, LAYER_BITS, LAYER_BITS};
  std::vector<std::vector<jms::nn::BNN>> pops(2);
  pops[0].reserve(POP_SIZE);
  pops[1].reserve(POP_SIZE);
  for (size_t i=0; i<POP_SIZE; ++i) {
    pops[0].emplace_back(dataset.InputDim(), dataset.OutputDim(), layer_sizes);
    pops[1].emplace_back(dataset.InputDim(), dataset.OutputDim(), layer_sizes);
  }
  for (auto& nn : pops[current_pop_index]) { nn.Randomize(rng); }
  double m_rate = POINT_MUTATION_RATE;
  dt = clock.now() - t;
  total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.0;
  std::cout << "Initialize pop (" << total << " sec)" << std::endl;

  double eval_time = 0.0;
  double pick_time = 0.0;
  double repl_time = 0.0;
  t = clock.now();
  for (size_t generation=0; generation<NUM_GENERATIONS; ++generation) {
    size_t counter = 0;
    for (auto opt_batch=dataset.Batch(BATCH_SIZE);
         opt_batch.has_value();
         opt_batch=dataset.Batch(BATCH_SIZE)) {
      auto t1 = clock.now();
      size_t next_pop_index = (current_pop_index + 1) % 2;
      auto& pop = pops[current_pop_index];
      auto& next_pop = pops[next_pop_index];
      auto t2 = clock.now();
      std::vector<double> scores = jms::evolve::EvaluatePop(pop, opt_batch.value());
      auto dt2 = clock.now() - t2;
      eval_time += std::chrono::duration_cast<std::chrono::microseconds>(dt2).count();
      t2 = clock.now();
      std::vector<std::tuple<size_t, double>> results(scores.size());
      std::ranges::transform(scores, std::views::iota(static_cast<size_t>(0), scores.size()),
                             std::make_move_iterator(results.begin()),
                             [](double score, size_t i) -> std::tuple<size_t, double> { return {i, score}; });
      std::vector<size_t> rep_ids = jms::evolve::Pick(results, NUM_KEEP, PICK_SCALE, rng);
      dt2 = clock.now() - t2;
      pick_time += std::chrono::duration_cast<std::chrono::microseconds>(dt2).count();
      t2 = clock.now();
      jms::evolve::Replicate(pop, next_pop, rep_ids, NUM_KEEP, m_rate, rng);
      dt2 = clock.now() - t2;
      repl_time += std::chrono::duration_cast<std::chrono::microseconds>(dt2).count();
      current_pop_index = next_pop_index;
      counter++;
      auto dt1 = clock.now() - t1;
      double total1 = std::chrono::duration_cast<std::chrono::microseconds>(dt1).count() / 1000000.0;
      //if (counter % 10 == 0) {
        //std::cout << "Gen " << generation << " - batch " << counter << "  with best acc: " << std::ranges::max(scores) << "  (" << total1 << " sec)" << std::endl;
        std::cout << "Totals: eval(" << eval_time << ")  pick(" << pick_time << ")  repl(" << repl_time << ")" << std::endl;
        std::cout << "Gen " << generation << " - batch " << counter << "  with best acc: " << std::ranges::max(scores) << " " << scores[0] << " " << scores[1] << " | " << m_rate << "  (" << total1 << " sec)" << std::endl;
      //}
    }
    //if (m_rate > 0.02) { m_rate -= 0.005; }
    //if (m_rate > 0.00001) { m_rate *= 0.85; }
    //dataset.Shuffle(rng);
    dataset.ResetBatches();
  }
  dt = clock.now() - t;
  total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count() / 1000000.0;
  std::cout << "Total train time (" << total << " sec)" << std::endl;

  return 0;
}


int main(void) {
  std::cout << "Load training data ... ";
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
