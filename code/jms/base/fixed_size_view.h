#ifndef JMS_BASE_FIXED_SIZE_VIEW_H
#define JMS_BASE_FIXED_SIZE_VIEW_H


namespace jms {
namespace base {


template <typename T>
struct FixedSizeView_i {
  using value_type      = std::remove_cv_t<T>;
  using reference       = T&;
  using const_reference = const T&;
  using pointer         = T*;
  using const_pointer   = const T*;
  /*
  using iterator        = Iter;
  using const_iterator  = const Iter;
  using riterator       = std::reverse_iterator<Iter>;
  using const_riterator = std::reverse_iterator<const_iterator>;
  */
  using difference_type = ptrdiff_t;
  using size_type       = size_t;

  virtual constexpr ~FixedSizeView_i(void) noexcept { return; }

  virtual constexpr reference operator[](size_t index) noexcept = 0;
  virtual constexpr const_reference operator[](size_t index) const noexcept = 0;
  virtual constexpr reference back(void) noexcept = 0;
  virtual constexpr const_reference back(void) const noexcept = 0;
  virtual constexpr reference front(void) noexcept = 0;
  virtual constexpr const_reference front(void) const noexcept = 0;

  virtual constexpr pointer data(void) noexcept = 0;
  virtual constexpr const_pointer data(void) const noexcept = 0;

  virtual constexpr bool empty(void) const noexcept = 0;
  virtual constexpr size_t size(void) const noexcept = 0;
  virtual constexpr size_t max_size(void) const noexcept = 0;

  /*
  virtual constexpr iterator begin(void) noexcept = 0;
  virtual constexpr const_iterator begin(void) const noexcept = 0;
  virtual constexpr riterator rbegin(void) noexcept = 0;
  virtual constexpr const_riterator rbegin(void) const noexcept = 0;

  virtual constexpr iterator end(void) noexcept = 0;
  virtual constexpr const_iterator end(void) const noexcept = 0;
  virtual constexpr riterator rend(void) noexcept = 0;
  virtual constexpr const_riterator rend(void) const noexcept = 0;

  virtual constexpr const_iterator cbegin(void) const noexcept = 0;
  virtual constexpr const_riterator crbegin(void) const noexcept = 0;
  virtual constexpr const_iterator cend(void) const noexcept = 0;
  virtual constexpr const_riterator crend(void) const noexcept = 0;
  */
};


template <Array_c Array_t>
class FixedSizeView : public FixedSizeView_i<typename Array_t::value_type> {
public:
  using Interface_t = FixedSizeView_i<typename Array_t::value_type>;

private:
  Array_t& ref;

public:
  virtual constexpr ~FixedSizeView(void) noexcept { return; }

  constexpr Interface_t::reference operator[](size_t index) noexcept override { return ref[index]; }
  constexpr Interface_t::const_reference operator[](size_t index) const noexcept override { return ref[index]; }
  constexpr Interface_t::reference back(void) noexcept override { return ref.back(); }
  constexpr Interface_t::const_reference back(void) const noexcept override { return ref.back(); }
  constexpr Interface_t::reference front(void) noexcept override { return ref.front(); }
  constexpr Interface_t::const_reference front(void) const noexcept override { return ref.front(); }

  constexpr Interface_t::pointer data(void) noexcept override { return ref.data(); }
  constexpr Interface_t::const_pointer data(void) const noexcept override { return ref.data(); }

  constexpr bool empty(void) const noexcept override { return ref.empty(); }
  constexpr size_t size(void) const noexcept override { return ref.size(); }
  constexpr size_t max_size(void) const noexcept override { return ref.max_size();; }

  /*
  constexpr Interface_t::iterator begin(void) noexcept override { return ref.begin(); }
  constexpr Interface_t::const_iterator begin(void) const noexcept override { return ref.begin(); }
  constexpr Interface_t::riterator rbegin(void) noexcept override { return ref.rbegin(); }
  constexpr Interface_t::const_riterator rbegin(void) const noexcept override { return ref.rbegin(); }

  constexpr Interface_t::iterator end(void) noexcept override { return ref.end(); }
  constexpr Interface_t::const_iterator end(void) const noexcept override { return ref.end(); }
  constexpr Interface_t::riterator rend(void) noexcept override { return ref.rend(); }
  constexpr Interface_t::const_riterator rend(void) const noexcept override { return ref.rend(); }

  constexpr Interface_t::const_iterator cbegin(void) const noexcept override { return ref.cbegin(); }
  constexpr Interface_t::const_riterator crbegin(void) const noexcept override { return ref.crbegin(); }
  constexpr Interface_t::const_iterator cend(void) const noexcept override { return ref.cend(); }
  constexpr Interface_t::const_riterator crend(void) const noexcept override { return ref.crend(); }
  */
};


} // namespace base
} // namespace jms


#endif // JMS_BASE_FIXED_SIZE_VIEW_H
