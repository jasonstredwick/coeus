#ifndef SIM_INTERFACE_H
#define SIM_INTERFACE_H


namespace jms {
namespace sim {


class Interface {
public:
  virtual ~Interface() {}
  virtual void Step(void) = 0;
};


} // namespace sim
} // namespace jms


#endif // SIM_INTERFACE_H
