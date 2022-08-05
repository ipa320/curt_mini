#include "ipa_ros2_control/mcDSAE25.h"
#include "soem_ros2/ethercatcoe.h"
#include "soem_ros2/ethercattype.h"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <thread>

namespace ipa_ros2_control {

int mcDSAE25_PO2SOparam(uint16 slave) {
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "mcDSAE25setup für slave %d named %s aufgerufen\n", slave,
              ec_slave[slave].name);
  int8 i8buf;
  int32 i32buf;
  uint16 u16buf;
  uint16 u16buf2 = 0x66;

  int uint16size = sizeof(u16buf);

  // Steuerwort - Betrieb sperren
  u16buf = 0x07;
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Steuerwort - Betrieb sperren");
  ec_SDOwrite(slave, 0x6040, 0x00, FALSE, sizeof(u16buf), &u16buf,
                        EC_TIMEOUTSAFE);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Fehler löschen
  u16buf = 0x8F;
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Fehler loeschen");
  ec_SDOwrite(slave, 0x6040, 0x00, FALSE, sizeof(u16buf), &u16buf,
                        EC_TIMEOUTSAFE);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  i8buf = 2; // Betriebsart Velocity 2
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Velocity Betriebsart gesetzt = %d",
              ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(i8buf), &i8buf,
                          EC_TIMEOUTSAFE));
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Betriebsart 0x6060 warten");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "warten vorbei");

  u16buf = 0x8001; // Motortyp BLDC - Dieses Register wird mit
                   // miControl-Software nicht verwendet
  ec_SDOwrite(slave, 0x6402, 0x00, FALSE, sizeof(u16buf), &u16buf,
              EC_TIMEOUTSAFE);
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Motortyp 0x6402 warten");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "warten vorbei");

  i32buf = 64; // Drehzahldimnesionsfaktor - Zaehler = 64
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Drehzahldimnesionsfaktor - Zaehler gesetzt = %d",
              ec_SDOwrite(slave, 0x604C, 0x01, FALSE, sizeof(i32buf), &i32buf,
              EC_TIMEOUTSAFE));
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
              "Dimensionsfaktor Zaehler 0x604C warten");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "warten vorbei");

  ec_SDOread(slave, 0x6402, 0x00, FALSE, &uint16size, &u16buf2,
             EC_TIMEOUTSAFE); // Aktualisiere State-wert
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Motortyp 0x6402 = %x",
              u16buf2);

  return 1;
}

mcDSAE25::mcDSAE25(ec_slavet *slave) : slave_(slave) {
  slave_->PO2SOconfig = &mcDSAE25_PO2SOparam;
}

mcDSAE25::~mcDSAE25() {}

void mcDSAE25::setSlave(ec_slavet *slave, uint16 slave_nr) {
  slave_ = slave;
  slave_nr_ = slave_nr;
  slave_->PO2SOconfig = &mcDSAE25_PO2SOparam;
}

InternalState mcDSAE25::status() {
  uint16 u16buf;
  int uint16size = sizeof(u16buf);

  ec_SDOread(slave_nr_, 0x6041, 0x00, FALSE, &uint16size, &u16buf,
             EC_TIMEOUTSAFE);
  switch (0b0000000000001111 & u16buf) {
  case 0b0000:
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "StatusWord: %d",
                u16buf);
    if (0b0000000001000000 & u16buf) {
      RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                  "Zustand Switch_On_Disabled");
      return Switch_On_Disabled;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                  "Slave %d\nZustand Not_Ready_To_Switch_On", slave_nr_);
      return Not_Ready_To_Switch_On;
    }
  case 0b0001:
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "Ready_To_Switch_On");
    return Ready_To_Switch_On;
  case 0b0011:
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "CiA402-Status: Switched_On");
    return Switched_On;
  case 0b0111:
    if ((0b0000000001100000 & u16buf) == 0b0000000000100000) {
      RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                  "CiA402-Status: Operation_Enabled");
      return Operation_Enabled;
    } else {
      return Quick_Stop_Active;
    }
  case 0b1111:
    return Fault_Reaction_Active;
  case 0b1000:
    return Fault;
  }
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Return unknown");
  return Unknown;
}

void mcDSAE25::setCommand(const InternalCommand &command) {
  uint16 u16buf;
  switch (command) {
  case Shutdown:
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Shutdown %s",
                slave_->name);
    u16buf = 0x06;
    break;
  case Switch_On:
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Switch_ON %s",
                slave_->name);
    u16buf = 0x07;
    break;
  case Disable_Voltage:
    u16buf = 0x00;
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "Disable_Voltage on %s", slave_->name);
    break;
  case Quick_Stop:
    u16buf = 0x02;
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Quick_Stop %s",
                slave_->name);
    break;
  case Disable_Operation:
    u16buf = 0x07;
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "Disable_Operation of %s", slave_->name);
    break;
  case Enable_Operation:
    u16buf = 0x0F;
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"),
                "Enable_Operation of %s", slave_->name);
    break;
  case Faulut_Reset:
    u16buf = 0x8F;
    RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Fault Reset %s",
                slave_->name);
    break;
  }
  ec_SDOwrite(slave_nr_, 0x6040, 0x00, FALSE, sizeof(u16buf), &u16buf,
              EC_TIMEOUTSAFE);
}

bool mcDSAE25::changeStatus(const InternalState &targetStatus) {
  // printf("changeStatus for %s\n", slave_->name);
  switch (targetStatus) {
  case Operation_Enabled:
    switch (status()) {
    case Operation_Enabled:
      return 1; // ist schon im Zielzustand
    case Start:
      return 0; // Gerät wird noch Initialisiert
    case Not_Ready_To_Switch_On:
      return 0; // Gerät wird noch Initialisiert
    case Switch_On_Disabled:
      setCommand(Shutdown);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Ready_To_Switch_On:
      setCommand(Switch_On);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Switched_On:
      setCommand(Enable_Operation);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Quick_Stop_Active:
      setCommand(Disable_Voltage);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Fault_Reaction_Active:
      return 0; // oder warten
    case Fault:
      setCommand(Faulut_Reset);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    }
    break;

  case Switched_On:
    switch (status()) {
    case Operation_Enabled:
      setCommand(Disable_Operation); // ist schon im Zielzustand
      std::this_thread::sleep_for(std::chrono::seconds(1));
      break;
    case Start:
      return 0; // Gerät wird noch Initialisiert
    case Not_Ready_To_Switch_On:
      return 0; // Gerät wird noch Initialisiert
    case Switch_On_Disabled:
      setCommand(Shutdown);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Ready_To_Switch_On:
      setCommand(Switch_On);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Switched_On:
      return 1;
    case Quick_Stop_Active:
      setCommand(Disable_Voltage);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    case Fault_Reaction_Active:
      return 0; // oder warten
    case Fault:
      setCommand(Faulut_Reset);
      // std::this_thread::sleep_for (std::chrono::seconds(1));
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
  return 0; // for  ohne Erreichen des Zielzustands durchlaufen
}

int32 mcDSAE25::getRPM() const {
  int32 i32buf;
  int size = sizeof(i32buf);
  ec_SDOread(slave_nr_, 0x3A04, 0x01, FALSE, &size, &i32buf, EC_TIMEOUTSAFE);
  return static_cast<int32>(i32buf);
}

void mcDSAE25::setRPM(int16 rpm) const {
  int16 i16buf = static_cast<int16>(rpm);
  ec_SDOwrite(slave_nr_, 0x6042, 0x00, FALSE, sizeof(i16buf), &i16buf,
              EC_TIMEOUTSAFE);
  RCLCPP_INFO(rclcpp::get_logger("MvpHardwareInterface"), "Commanded RPM: %d",
              rpm);
}

} // namespace ipa_ros2_control
