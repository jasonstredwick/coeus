#ifndef JMS_AGENT_MUTATION_HELPER_H
#define JMS_AGENT_MUTATION_HELPER_H


#include <limits>

#include "jms/base/concepts.h"


namespace jms {
namespace agent {


template <typename T>
struct MutateCallable {
  virtual ~MutateCallable(void) noexcept { return; }
  const T operator()(const T) noexcept = 0;
};


struct ThresholdCallable {
  virtual ~ThresholdCallable(void) noexcept { return; }
  const bool operator()(void) noexcept = 0;
};


template <typename T, typename Rng_t, typename Dist_t>
class ChanceThreshold : public ThresholdCallable {
private:
  Rng_t& rng;
  Dist_t dist;
  T threshold;

public:
  ChanceThreshold(T threshold, Rng_t& rng, Dist_t dist) noexcept : threshold(threshold), rng(rng), dist(dist) { return; }
  virtual ~ChanceThreshold(void) noexcept { return; }
  const bool operator()(void) noexcept override { return dist(rng) >= threshold; }
};


template <typename T, typename Rng_t, typename Dist_t, FT2T_c ChangeFunc>
class ChanceChange : public MutateCallable<T> {
private:
  Rng_t& rng;
  Dist_t dist;
  T threshold;
  ChangeFunc F;

public:
  ChanceChange(T threshold, Rng_t& rng, Dist_t dist, ChangeFunc F) noexcept : threshold(threshold), rng(rng), dist(dist), F(F) { return; }
  virtual ~ChanceChange(void) noexcept { return; }
  const bool operator()(const T t) noexcept override { return dist(rng) >= threshold ? F(t); }
};


} // namespace agent
} // namespace jms

#endif // JMS_AGENT_MUTATION_HELPER_H
