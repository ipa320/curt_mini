
#include "ipa_outdoor_drivers/ethercat_wrapper.h"
#include <functional>
#include <memory>
#include <sstream>

#define EC_TIMEOUTMON 500

EthercatWrapper::EthercatWrapper() {}

EthercatWrapper::~EthercatWrapper() {
  pdo_transfer_active_ = false;
  /* request INIT state for all slaves */
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  /* stop SOEM, close socket */
  ec_close();
}

auto EthercatWrapper::initializeEthercat(const char *ifname)
    -> tl::expected<std::string, std::string> {
  /* initialise SOEM, bind socket to ifname */
  std::stringstream logger;
  if (!ec_init(ifname)) {
    logger << "No socket connection on " << ifname << ".\n";
    return tl::unexpected(logger.str());
  }
  if (ec_config_init(FALSE) <= 0) // setup mailboxes, alle Slaves in PRE_OP
  {
    logger << "No slaves found";
    return tl::unexpected(logger.str());
  }
  std::stringstream out;
  out << "Ethercat initialized on " << ifname << ". " << ec_slavecount
      << " slaves found";
  return out.str();
}

auto EthercatWrapper::setupSlaves() -> tl::expected<std::string, std::string> {
  std::stringstream logger;
  char IOmap[4096];
  ec_config_map(
      &IOmap); // Map all PDOs from slaves to IOmap with Outputs/Inputs in
               // sequential order (legacy SOEM way)., danach SAFE_OP

  ec_configdc(); // Locate Distributed Clock slaves, measure propagation delays.

  logger << "output bytes= " << ec_slave[0].Obytes
         << "\n"; // number of output-Bytes. Maximal das zum Pointer
                  // ec_slave[1].outputs addieren um objekt durch zu gehen.
  logger << "input bytes= " << ec_slave[0].Ibytes << "\n";

  logger << "Slaves mapped, state to SAFE_OP.\n";
  /* wait for all slaves to reach SAFE_OP state */
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  logger << "segments : " << ec_group[0].nsegments << " "
         << ec_group[0].IOsegment[0] << " " << ec_group[0].IOsegment[1] << " "
         << ec_group[0].IOsegment[2] << " " << ec_group[0].IOsegment[3] << "\n";

  logger << "Request operational state for all slaves\n";
  expectedWKC_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  logger << "Calculated workcounter " << expectedWKC_ << "\n";
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  /* request OP state for all slaves */
  ec_writestate(0);

  /* wait for all slaves to reach OP state */
  int chk = 40;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state !=
                     EC_STATE_OPERATIONAL)); // wird 40 mal ausgeführt
  if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
    logger << "Operational state reached for all slaves.\n";
    pdo_transfer_active_ = true;
    /* Check if slaves are found in the expected order */
    return logger.str();
  }

  logger << "Not all slaves reached operational state.\n";
  ec_readstate();
  for (int i = 1; i <= ec_slavecount; i++) {
    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
      logger << "Slave" << i << " State=0x" << ec_slave[i].state
             << "StatusCode=0x" << ec_slave[i].ALstatuscode << " : "
             << ec_ALstatuscode2string(ec_slave[i].ALstatuscode) << "\n";
    }
  }
  return tl::unexpected(logger.str());
}

auto EthercatWrapper::statecheck_callback() -> std::optional<std::string> {
  uint8 currentgroup = 0;

  if (!(pdo_transfer_active_ &&
        ((wkc_ < expectedWKC_) || ec_group[currentgroup].docheckstate))) {
    return std::nullopt;
  }
  std::stringstream logger;
  /* one ore more slaves are not responding */
  ec_group[currentgroup].docheckstate = FALSE;
  ec_readstate();
  for (int slave = 1; slave <= ec_slavecount; slave++) {
    if ((ec_slave[slave].group == currentgroup) &&
        (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
      ec_group[currentgroup].docheckstate = TRUE;
      if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
        logger << "ERROR : slave " << slave
               << " is in SAFE_OP + ERROR, attempting ack."
               << "\n";
        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
        ec_writestate(slave);
      } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
        logger << "slave " << slave
               << " is in SAFE_OP, change to OPERATIONAL.\n";
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
      } else if (ec_slave[slave].state > 0) {
        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
          ec_slave[slave].islost = FALSE;
          logger << "MESSAGE : slave " << slave << " reconfigured\n";
        }
      } else if (!ec_slave[slave].islost) {
        /* re-check state */
        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (!ec_slave[slave].state) {
          ec_slave[slave].islost = TRUE;
          logger << "slave " << slave << " is lost!\n";
        }
      }
    }
    if (ec_slave[slave].islost) {
      if (!ec_slave[slave].state) {
        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
          ec_slave[slave].islost = FALSE;
          logger << "MESSAGE : slave " << slave << " recovered.\n";
        }
      } else {
        ec_slave[slave].islost = FALSE;
        logger << "MESSAGE : slave " << slave << " found\n";
      }
    }
  }
  if (!ec_group[currentgroup].docheckstate) {
    logger << "Recovered from not responding slaves : all slaves resumed "
              "OPERATIONAL.";
  }
  return logger.str();
}

void EthercatWrapper::datacycle_callback() {
  if (pdo_transfer_active_) {
    ec_send_processdata();
    wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
  }
}

void EthercatWrapper::close() {
  pdo_transfer_active_ = false;
  // request INIT state for all slaves
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  // stop SOEM, close socket
  ec_close();
}