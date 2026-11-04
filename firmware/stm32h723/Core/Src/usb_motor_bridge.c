#include "usb_motor_bridge.h"

#include "motor_manager.h"
#include "usb_protocol.h"

void usb_motor_bridge_process(void)
{
  const Motor_Command *cmd;

  if (usb_protocol_take_motor_command_ready() != 0U) {
    cmd = usb_protocol_get_motor_commands();
    motor_manager_apply_commands(cmd);
  }

  motor_manager_process();
}
