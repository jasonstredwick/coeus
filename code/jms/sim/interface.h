#ifndef SIM_INTERFACE_H
#define SIM_INTERFACE_H


namespace jms {
namespace sim {


class Interface {
public:
  virtual ~Interface() {}
  virtual void Step(void) noexcept = 0;
  virtual void StepN(int32_t num_steps) noexcept = 0;
};


} // namespace sim
} // namespace jms


#endif // SIM_INTERFACE_H
