#ifndef JMS_MEMORY_ALIGN_H
#define JMS_MEMORY_ALIGN_H


#include <algorithm>
#include <memory>


namespace jms {


// Returns the total bytes needed to encompass N bytes in terms of align_sized units and optionally
// enough room to ensure the data can be offset to start at a memory address aligned to the align_size.
struct alignas(32) AlignedBytesDetails {
  size_t align_size;
  size_t boundry_bytes;
  size_t num_bytes;
  size_t padding;
  size_t total_bytes;
  AlignedBytesDetails(const size_t num_bytes, const size_t align_size, const bool ensure_boundary=false) noexcept
  : align_size(align_size ? align_size : 1),
    boundry_bytes(ensure_boundary && num_bytes ? align_size - 1 : 0),
    num_bytes(num_bytes),
    padding((num_bytes % align_size > 0) ? align_size - (num_bytes % align_size) : 0),
    total_bytes(num_bytes + padding + boundry_bytes) {
  }
  AlignedBytesDetails(const AlignedBytesDetails&) noexcept = default;
  AlignedBytesDetails(AlignedBytesDetails&&) noexcept = default;
  ~AlignedBytesDetails(void) noexcept = default;
  AlignedBytesDetails& operator=(const AlignedBytesDetails&) noexcept = default;
  AlignedBytesDetails& operator=(AlignedBytesDetails&&) noexcept = default;
};


struct AlignedBytes {
  AlignedBytesDetails details;
  std::unique_ptr<std::byte[]> buffer;

  AlignedBytes(const size_t num_bytes, const size_t align_size, const bool ensure_boundary=false) noexcept
  : details{num_bytes, align_size, ensure_boundary},
    buffer{details.total_bytes ? new (std::nothrow) std::byte[details.total_bytes]() : nullptr} {
    return;
  }
  AlignedBytes(const AlignedBytes& chunk) noexcept
  : details(details), buffer{details.total_bytes ? new (std::nothrow) std::byte[details.total_bytes]() : nullptr} {
    if (buffer.get()) { std::copy_n(chunk.buffer.get(), details.total_bytes, buffer.get()); }
    return;
  }
  AlignedBytes(AlignedBytes&&) noexcept = default;
  ~AlignedBytes(void) noexcept = default;
  AlignedBytes(const AlignedBytes&&) noexcept = delete;
  AlignedBytes& operator=(AlignedBytes&&) noexcept = default;

  bool operator!(void) const noexcept { return buffer.get() == nullptr; }
};


} // namespace jms


#endif // JMS_MEMORY_ALIGN_H
