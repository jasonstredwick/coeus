#ifndef JMS_BASE_ARRAY_H
#define JMS_BASE_ARRAY_H


#include <algorithm>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>


namespace jms {
namespace base {
namespace array {


template <typename T, size_t align_size=alignof(std::remove_cvref_t<T>)> class AlignedDynamic;
template <typename T, size_t N, size_t align_size=alignof(std::remove_cvref_t<T>)> class AlignedStatic;


template <typename T>
using AlignedDynamic64_t = AlignedDynamic<T, 64>;


template <typename T, size_t align_size>
class AlignedDynamic {
public:
  using value_type      = std::remove_cvref_t<T>;
  using reference       = value_type&;
  using const_reference = const value_type&;
  using pointer         = value_type*;
  using const_pointer   = const value_type*;
  using iterator        = pointer;
  using const_iterator  = const_pointer;
  using riterator       = std::reverse_iterator<iterator>;
  using const_riterator = std::reverse_iterator<const_iterator>;
  using difference_type = ptrdiff_t;
  using size_type       = size_t;

  AlignedDynamic(size_t N) noexcept { Init(N); return; }
  AlignedDynamic(const AlignedDynamic& from_array) noexcept { CopyConstruct(from_array); return; }
  AlignedDynamic(AlignedDynamic&&) noexcept = default;
  ~AlignedDynamic(void) noexcept = default;
  AlignedDynamic& operator=(const AlignedDynamic& from_array) noexcept { Copy(from_array); return *this; }
  AlignedDynamic& operator=(AlignedDynamic&&) noexcept = default;

  reference operator[](size_t index) noexcept { return array_data[index]; }
  const_reference operator[](size_t index) const noexcept { return array_data[index]; }
  reference back(void) noexcept { return array_data[array_size-1]; }
  const_reference back(void) const noexcept { return array_data[array_size-1]; }
  reference front(void) noexcept { return array_data[0]; }
  const_reference front(void) const noexcept { return array_data[0]; }

  pointer data(void) noexcept { return array_data; }
  const_pointer data(void) const noexcept { return array_data; }

  void resize(size_t new_size, bool shrink_to_fit=false) { Resize(new_size, shrink_to_fit); return; }
  void shrink_to_fit(void) { Resize(array_size, true); return; }

  bool empty(void) const noexcept { return array_size == 0; }
  size_t size(void) const noexcept { return array_size; }
  size_t max_size(void) const noexcept { return array_size; }
  size_t total_bytes(void) const noexcept { return raw_total_bytes; }

  iterator begin(void) noexcept { return array_data; }
  const_iterator begin(void) const noexcept { return array_data; }
  riterator rbegin(void) noexcept { return riterator(end()); }
  const_riterator rbegin(void) const noexcept { return const_riterator(end()); }

  iterator end(void) noexcept { return array_data + array_size; }
  const_iterator end(void) const noexcept { return array_data + array_size; }
  riterator rend(void) noexcept { return riterator(begin()); }
  const_riterator rend(void) const noexcept { return const_riterator(begin()); }

  const_iterator cbegin(void) const noexcept { return begin(); }
  const_riterator crbegin(void) const noexcept { return rbegin(); }
  const_iterator cend(void) const noexcept { return end(); }
  const_riterator crend(void) const noexcept { return rend(); }

private:
  static_assert((align_size == sizeof(value_type)) ||
                (align_size > sizeof(value_type) && align_size % sizeof(value_type) == 0) ||
                (align_size < sizeof(value_type) && sizeof(value_type) % align_size == 0),
                "AlignedDynamic requires type size and align_size to be divisible.");
  std::unique_ptr<std::byte[]> raw;
  pointer_type array_data {nullptr};
  size_t array_size {0};
  size_t raw_total_bytes {0};

  size_t TotalBytesNeeded(size_t N) const noexcept {
    size_t num_bytes = N * sizeof(value_type);
    size_t delta = num_bytes % align_size;
    size_t padding = (delta > 0) ? align_size - delta : 0;
    // extra align_size allows for aligning pointer to start of data to align_size memory boundary
    return num_bytes + padding + align_size;
  }

  void Copy(const AlignedDynamic& from_array) noexcept {
    // total bytes needed for N=0 is 0; so false for N=0 in all cases
    if (raw_total_bytes < TotalBytesNeeded(from_array.array_size)) {
      Init(from_array.array_size);
    } else {
      array_size = from_array.array_size;
      if (array_size == 0) { array_data = nullptr; return; }
    }
    std::copy(from_array.cbegin(), from_array.cend(), begin());
    return;
  }

  void CopyConstruct(const AlignedDynamic& from_array) noexcept {
    Init(from_array.array_size);
    if (array_size == 0) { return; }
    std::copy(from_array.cbegin(), from_array.cend(), begin());
    return;
  }

  void Init(size_t N) noexcept {
    array_size = 0;
    array_data = nullptr;
    raw_total_bytes = 0;
    raw.reset();
    if (!N) { return; }
    size_t total_bytes_needed = TotalBytesNeeded(N);
    raw.reset(new (std::nothrow) std::byte[total_bytes_needed]());
    void* pt = raw.get();
    if (pt == nullptr) { return; }
    raw_total_bytes = total_bytes_needed;
    array_data = std::assume_aligned<align_size>(static_cast<pointer_type>(std::align(align_size, sizeof(value_type) * N, pt, total_bytes_needed)));
    array_size = N;
    return;
  }

  // NOTE: Does not initialize data outside of the N copied units if resize reuses previous raw data.
  // NOTE: If resize fails to create new raw data (as needed) then size and total bytes will be zero.
  // NOTE: Returns original array_size and raw_total_bytes to verify change if necessary.
  std::pair<size_t, size_t> Resize(size_t N, bool shrink_to_fit=false) noexcept {
    std::pair<size_t, size_t> old_sizes = std::make_pair(array_size, raw_total_bytes);
    size_t total_bytes_needed = TotalBytesNeeded(N);
    if (total_bytes_needed == raw_total_bytes || (!shrink_to_fit && total_bytes_needed < raw_total_bytes)) { array_size = N; return old_sizes; }
    std::unique_ptr<std::byte> tmp_raw {raw.release()};
    pointer_type tmp_array_data = std::assume_aligned<align_size>(array_data);
    Init(N);
    if (!array_size && N) {
      // Failed to allocate new data; reset to original data as if this function was not called.
      raw.reset(tmp_raw.release());
      raw_total_bytes = old_sizes.second;
      array_data = std::assume_aligned<align_size>(tmp_array_data);
      array_size = old_sizes.first;
      return old_sizes;
    }
    size_t copy_size = std::min(array_size, old_sizes.first);
    if (copy_size) { std::copy(tmp_array_data, tmp_array_data + copy_size, array_data); }
    return old_sizes;
  }
};


template <typename T, size_t N, size_t align_size>
class AlignedStatic {
public:
  using value_type      = std::remove_cvref_t<T>;
  using reference       = value_type&;
  using const_reference = const value_type&;
  using pointer         = value_type*;
  using const_pointer   = const value_type*;
  using iterator        = pointer;
  using const_iterator  = const_pointer;
  using riterator       = std::reverse_iterator<iterator>;
  using const_riterator = std::reverse_iterator<const_iterator>;
  using difference_type = ptrdiff_t;
  using size_type       = size_t;

private:
  static_assert((align_size == sizeof(value_type)) ||
                (align_size > sizeof(value_type) && align_size % sizeof(value_type) == 0) ||
                (align_size < sizeof(value_type) && sizeof(value_type) % align_size == 0),
                "AlignedStatic requires type size and align_size to be divisible.");
  static_assert(N > 0, "AlignedStatic of fixed size N cannot be zero.");

  constexpr static size_t num_bytes_v = N * sizeof(value_type);
  constexpr static size_t remaining_v = num_bytes_v % align_size;
  constexpr static size_t padding_v = (remaining_v > 0) ? align_size - remaining_v : 0;
  constexpr static size_t total_bytes_v = num_bytes_v + padding_v;

  alignas(align_size) value_type array_data[N]{};

public:
  constexpr AlignedStatic(void) noexcept = default;
  constexpr AlignedStatic(AlignedStatic&&) noexcept = default;
  constexpr ~AlignedStatic(void) noexcept = default;
  constexpr AlignedStatic& operator=(AlignedStatic&&) noexcept = default;

  constexpr reference operator[](size_t index) noexcept { return array_data[index]; }
  constexpr const_reference operator[](size_t index) const noexcept { return array_data[index]; }
  constexpr reference back(void) noexcept { return array_data[N-1]; }
  constexpr const_reference back(void) const noexcept { return array_data[N-1]; }
  constexpr reference front(void) noexcept { return array_data[0]; }
  constexpr const_reference front(void) const noexcept { return array_data[0]; }

  constexpr pointer data(void) noexcept { return array_data; }
  constexpr const_pointer data(void) const noexcept { return array_data; }

  constexpr bool empty(void) const noexcept { return false; }
  constexpr size_t size(void) const noexcept { return N; }
  constexpr size_t max_size(void) const noexcept { return N; }
  constexpr size_t total_bytes(void) const noexcept { return AlignedStatic<T, N, align_size>::total_bytes_v; }

  constexpr iterator begin(void) noexcept { return array_data; }
  constexpr const_iterator begin(void) const noexcept { return array_data; }
  constexpr riterator rbegin(void) noexcept { return riterator(end()); }
  constexpr const_riterator rbegin(void) const noexcept { return const_riterator(end()); }

  constexpr iterator end(void) noexcept { return &array_data[N-1]; }
  constexpr const_iterator end(void) const noexcept { return &array_data[N-1]; }
  constexpr riterator rend(void) noexcept { return riterator(begin()); }
  constexpr const_riterator rend(void) const noexcept { return const_riterator(begin()); }

  constexpr const_iterator cbegin(void) const noexcept { return begin(); }
  constexpr const_riterator crbegin(void) const noexcept { return rbegin(); }
  constexpr const_iterator cend(void) const noexcept { return end(); }
  constexpr const_riterator crend(void) const noexcept { return rend(); }
};


} // namespace array
} // namespace base
} // namespace jms


#endif // JMS_BASE_ARRAY_H
