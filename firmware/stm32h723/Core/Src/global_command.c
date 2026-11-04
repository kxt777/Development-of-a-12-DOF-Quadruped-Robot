#include "global_command.h"

#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"
#include "motor_manager.h"
#include "usb_protocol.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#define GLOBAL_COMMAND_ENABLE_ALL 0x01U
#define GLOBAL_COMMAND_DISABLE_ALL 0x02U
#define GLOBAL_COMMAND_DAMPING_ALL 0x03U
#define GLOBAL_COMMAND_FIRST_MOTOR_ID 0x01U
#define GLOBAL_COMMAND_STEP_INTERVAL_MS 10U

typedef enum
{
  GLOBAL_COMMAND_IDLE = 0,
  GLOBAL_COMMAND_SEND_ACK,
  GLOBAL_COMMAND_SEND_MOTOR_MSG,
  GLOBAL_COMMAND_START_COMMAND,
  GLOBAL_COMMAND_EXECUTE_COMMAND
} GlobalCommandState;

static GlobalCommandState global_state = GLOBAL_COMMAND_IDLE;
static uint8_t active_command = 0U;
static uint8_t active_motor = 0U;
static uint32_t command_tick = 0U;
static char ack_msg[40];
static char motor_msg[32];

static const char *global_command_name(uint8_t cmd)
{
  switch (cmd) {
    case GLOBAL_COMMAND_ENABLE_ALL:
      return "ENABLE_ALL";
    case GLOBAL_COMMAND_DISABLE_ALL:
      return "DISABLE_ALL";
    case GLOBAL_COMMAND_DAMPING_ALL:
      return "DAMPING_ALL";
    default:
      return "UNKNOWN";
  }
}

static uint8_t global_command_is_supported(uint8_t cmd)
{
  return ((cmd == GLOBAL_COMMAND_ENABLE_ALL) ||
          (cmd == GLOBAL_COMMAND_DISABLE_ALL) ||
          (cmd == GLOBAL_COMMAND_DAMPING_ALL)) ? 1U : 0U;
}

static void global_command_prepare(uint8_t cmd)
{
  active_command = cmd;
  active_motor = 0U;

  snprintf(ack_msg,
           sizeof(ack_msg),
           "GLOBAL CMD OK\r\n%s\r\n",
           global_command_name(cmd));

  global_state = GLOBAL_COMMAND_SEND_ACK;
}

static void global_command_prepare_motor_msg(void)
{
  const char *action = "MOTOR";

  if (active_command == GLOBAL_COMMAND_ENABLE_ALL) {
    action = "ENABLE MOTOR";
  } else if (active_command == GLOBAL_COMMAND_DISABLE_ALL) {
    action = "DISABLE MOTOR";
  } else if (active_command == GLOBAL_COMMAND_DAMPING_ALL) {
    action = "DAMPING MOTOR";
  }

  snprintf(motor_msg,
           sizeof(motor_msg),
           "%s %u\r\n",
           action,
           (unsigned int)(active_motor + 1U));
}

static void global_command_start_execute(void)
{
  active_motor = 0U;
  command_tick = HAL_GetTick();
  global_state = GLOBAL_COMMAND_EXECUTE_COMMAND;
}

static void global_command_execute_step(void)
{
  uint32_t now;
  uint16_t motor_id;

  if (active_command != GLOBAL_COMMAND_ENABLE_ALL) {
    switch (active_command) {
      case GLOBAL_COMMAND_DISABLE_ALL:
        motor_manager_disable_all();
        break;
      case GLOBAL_COMMAND_DAMPING_ALL:
        motor_manager_damping_all();
        break;
      default:
        break;
    }

    global_state = GLOBAL_COMMAND_IDLE;
    return;
  }

  if (active_motor >= MOTOR_MANAGER_COUNT) {
    global_state = GLOBAL_COMMAND_IDLE;
    return;
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - command_tick) < GLOBAL_COMMAND_STEP_INTERVAL_MS) {
    return;
  }

  motor_id = (uint16_t)(GLOBAL_COMMAND_FIRST_MOTOR_ID + active_motor);
  enable_motor_mode(&hfdcan1, motor_id);
  active_motor++;
  command_tick = now;
}

static void global_command_execute(void)
{
  switch (active_command) {
    case GLOBAL_COMMAND_ENABLE_ALL:
    case GLOBAL_COMMAND_DISABLE_ALL:
    case GLOBAL_COMMAND_DAMPING_ALL:
      global_command_start_execute();
      break;
    default:
      global_state = GLOBAL_COMMAND_IDLE;
      break;
  }
}

void global_command_process(void)
{
  uint8_t cmd;

  if (global_state == GLOBAL_COMMAND_SEND_ACK) {
    if (CDC_Transmit_HS((uint8_t *)ack_msg, (uint16_t)strlen(ack_msg)) == USBD_OK) {
      global_command_prepare_motor_msg();
      global_state = GLOBAL_COMMAND_SEND_MOTOR_MSG;
    }
    return;
  }

  if (global_state == GLOBAL_COMMAND_SEND_MOTOR_MSG) {
    if (CDC_Transmit_HS((uint8_t *)motor_msg, (uint16_t)strlen(motor_msg)) == USBD_OK) {
      active_motor++;
      if (active_motor >= MOTOR_MANAGER_COUNT) {
        global_state = GLOBAL_COMMAND_START_COMMAND;
      } else {
        global_command_prepare_motor_msg();
      }
    }
    return;
  }

  if (global_state == GLOBAL_COMMAND_START_COMMAND) {
    global_command_execute();
    return;
  }

  if (global_state == GLOBAL_COMMAND_EXECUTE_COMMAND) {
    global_command_execute_step();
    return;
  }

  if (usb_protocol_take_global_command(&cmd) == 0U) {
    return;
  }

  if (global_command_is_supported(cmd) == 0U) {
    (void)CDC_Transmit_HS((uint8_t *)"GLOBAL CMD ERROR\r\n",
                          (uint16_t)strlen("GLOBAL CMD ERROR\r\n"));
    return;
  }

  global_command_prepare(cmd);
}
