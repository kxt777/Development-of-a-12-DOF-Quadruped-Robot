#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

#include "motor_manager.h"
#include <stdint.h>

void usb_protocol_process(void);
uint8_t usb_protocol_take_motor_command_ready(void);
const Motor_Command *usb_protocol_get_motor_commands(void);
uint8_t usb_protocol_take_global_command(uint8_t *cmd);

#endif /* USB_PROTOCOL_H */
