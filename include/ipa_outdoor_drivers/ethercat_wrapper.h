#include <soem_ros2/soem.h>
#include <string>
#include <tl_expected/expected.hpp>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#define EC_TIMEOUTMON 500

class EthercatWrapper
{
public:
  EthercatWrapper();

  ~EthercatWrapper();

  auto initializeEthercat(const char* ifname) -> tl::expected<std::string, std::string>;
  auto setupSlaves() -> tl::expected<std::string, std::string>;
  auto statecheck_callback() -> std::optional<std::string>;
  void datacycle_callback();
  void close();

private:
  volatile int expectedWKC_;
  volatile int wkc_;
  bool pdo_transfer_active_ = false;  // wird aktiviert, wenn alles im OP
};