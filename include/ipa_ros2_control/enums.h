#pragma once

namespace ipa_ros2_control
{
enum InternalState
{
  Unknown = 0,
  Start = 0,
  Not_Ready_To_Switch_On = 1,
  Switch_On_Disabled = 2,
  Ready_To_Switch_On = 3,
  Switched_On = 4,
  Operation_Enabled = 5,
  Quick_Stop_Active = 6,
  Fault_Reaction_Active = 7,
  Fault = 8,
};

enum InternalCommand
{
  Shutdown,
  Switch_On,
  Disable_Voltage,
  Quick_Stop,
  Disable_Operation,
  Enable_Operation,
  Faulut_Reset
};
}  // namespace ipa_ros2_control