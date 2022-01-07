#ifndef JMS_BASE_CONCEPTS_H
#define JMS_BASE_CONCEPTS_H


namespace jms {
namespace base {


template <typename Array_t>
concept Array_c = requires(Array_t a, size_t n) {
  {a[n]} -> std::convertible_to<typename Array_t::reference>;
  {a.back()} -> std::convertible_to<typename Array_t::reference>;
  {a.front()} -> std::convertible_to<typename Array_t::reference>;
  {a.data()} -> std::convertible_to<typename Array_t::pointer>;
  {a.empty()} -> std::convertible_to<bool>;
  {a.size()} -> std::convertible_to<size_t>;
  {a.max_size()} -> std::convertible_to<size_t>;
  {a.begin()} -> std::random_access_iterator;
  {a.cbegin()} -> const std::random_access_iterator;
  {a.end()} -> std::random_access_iterator;
  {a.cend()} -> const std::random_access_iterator;
  {a.rbegin()} -> std::reverse_iterator;
  {a.crbegin()} -> const std::reverse_iterator;
  {a.rend()} -> std::reverse_iterator;
  {a.crend()} -> const std::reverse_iterator;
};


template <typename F_t, typename V_t>
concept FT2T_c = std::is_invocable_v<F_t, V_t> && requires(F_t F, V_t v) {
  {F(v)} -> std::convertible_to<V_t>;
};


} // namespace jms
} // namespace jms


#endif // JMS_BASE_CONCEPTS_H
