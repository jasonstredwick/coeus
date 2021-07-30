#ifndef JMS_UTILS_BARRIER_H
#define JMS_UTILS_BARRIER_H


#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <type_traits>


namespace jms {
namespace utils {


template <typename T>
concept arrival_token_c = std::is_move_constructible_v<T> && std::is_move_assignable_v<T> && std::is_destructible_v<T>;
template <typename T>
concept CompletionFunction_c = std::is_move_constructible_v<T> && std::is_destructible_v<T> && std::is_nothrow_invocable_v<T&>;


struct Noop final { void operator()() noexcept { return; }; };


template <CompletionFunction_c CompletionFunction_t=Noop>
class alignas(64) barrier {
public:
  class arrival_token final {
  public:
    constexpr arrival_token(arrival_token&&) noexcept = default;
    constexpr ~arrival_token(void) noexcept = default;
    constexpr arrival_token& operator=(arrival_token&&) noexcept = default;
    constexpr auto operator<=>(const arrival_token&) const noexcept = default;
  private:
    uint8_t phase{0};
    constexpr explicit arrival_token(uint8_t phase) noexcept : phase(phase) { return; }
    constexpr void NextPhase(void) noexcept { phase = (phase + 1) % 2; return; }
    constexpr uint8_t Phase(void) const noexcept { return phase; }
    friend class barrier;
  };

private:
  std::mutex mutex;
  std::condition_variable condition;
  std::ptrdiff_t expected;
  std::ptrdiff_t count;
  arrival_token token; // used to prevent spurious wakeup checking if the value has changed (when count is zero)
  CompletionFunction_t CompletionFunction;

public:
  constexpr explicit barrier(std::ptrdiff_t expected, CompletionFunction_t CompletionFunction=CompletionFunction_t()) noexcept
  : expected(expected), count(expected), token{0}, CompletionFunction(CompletionFunction)
  { return; }
  barrier(const barrier&) = delete;
  barrier& operator=(const barrier&) = delete;

  static constexpr std::ptrdiff_t max(void) noexcept { return PTRDIFF_MAX; }

  [[nodiscard]] constexpr arrival_token arrive(std::ptrdiff_t n=1) {
    std::unique_lock<std::mutex> lock(mutex);
    return Update(n);
  }

  constexpr void wait(arrival_token&& arrival) {
    std::unique_lock<std::mutex> lock(mutex);
    const auto phase = arrival.Phase();
    if (phase == 7) { return; } // arrival was created at the same time as a notify; so don't wait.
    condition.wait(lock, [this, phase] { return phase != this->token.Phase(); });
    return;
  }

  constexpr void arrive_and_wait(void) {
    wait(arrive());
    return;
  }

  constexpr void arrive_and_drop(void) {
    std::unique_lock<std::mutex> lock(mutex);
    expected--;
    Update();
    return;
  }

private:
  constexpr arrival_token Update(std::ptrdiff_t n=1) noexcept {
    count -= n;
    if (!count) {
      count = expected;
      CompletionFunction();
      token.NextPhase();
      condition.notify_all();
      return arrival_token{7};
    }
    return arrival_token{token.Phase()};
  }
};


} // namespace utils
} // namespace jms


#endif // JMS_UTILS_BARRIER_H
