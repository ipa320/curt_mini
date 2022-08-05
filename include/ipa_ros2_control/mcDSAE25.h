#pragma once

#include "ipa_ros2_control/enums.h"
#include "soem_ros2/osal.h"
#include <soem_ros2/soem.h>
#include <rclcpp/logger.hpp>
#include <vector>
namespace ipa_ros2_control
{

int mcDSAE25_PO2SOparam(uint16 slave);

class mcDSAE25
{
public:
  mcDSAE25() = default;
  mcDSAE25(ec_slavet* slave);
  ~mcDSAE25();

  void setSlave(ec_slavet* slave, uint16 slave_nr);

  template <class Func>
  uint16 sdoReadInit(uint16_t slave, Func&& ec_SDOread) const
  {
    uint16 state = 0;
    int uint16size = sizeof(state);  // SDOread requires buffer size as pointer...
    ec_SDOread(slave, 0x6041, 0x00, FALSE, &uint16size, &state, EC_TIMEOUTSAFE);  // Aktualisiere State-wert
    return state;
  };

  InternalState status();

  void setCommand(const InternalCommand& command);

  bool changeStatus(const InternalState& targetStatus);

  int32 getRPM() const;

  void setRPM(int16 rpm) const;
  
private:
  ec_slavet* slave_;
  uint16 slave_nr_;
};
}  // namespace ipa_ros2_control
