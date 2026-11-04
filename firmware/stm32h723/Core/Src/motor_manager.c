#include "motor_manager.h"

#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"

static uint16_t pending_motor_mask = 0U;
static uint8_t next_motor_index = 0U;

static uint8_t motor_index_from_id(uint8_t id, uint8_t *index)
{
  if ((id < 1U) || (id > MOTOR_MANAGER_COUNT)) {
    return 0U;
  }

  *index = (uint8_t)(id - 1U);
  return 1U;
}

void motor_manager_init(void)
{
  dm_motor_init();
}

void motor_manager_enable_all(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    dm_motor_clear_err(&hfdcan1, &motor[i]);
    HAL_Delay(1);
  }

  HAL_Delay(20);

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    dm_motor_enable(&hfdcan1, &motor[i]);
    HAL_Delay(1);
  }
}

void motor_manager_disable_all(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    dm_motor_disable(&hfdcan1, &motor[i]);
    HAL_Delay(1);
  }
}

void motor_manager_damping_all(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    motor[i].ctrl.mode = mit_mode;
    motor[i].ctrl.pos_set = 0.0f;
    motor[i].ctrl.vel_set = 0.0f;
    motor[i].ctrl.kp_set = 0.0f;
    motor[i].ctrl.kd_set = 1.0f;
    motor[i].ctrl.tor_set = 0.0f;
    dm_motor_ctrl_send(&hfdcan1, &motor[i]);
    HAL_Delay(1);
  }
}

void motor_manager_apply_commands(const Motor_Command motor_cmd[MOTOR_MANAGER_COUNT])
{
  uint8_t i;
  uint8_t index;

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    if (motor_cmd[i].valid == 0U) {
      continue;
    }

    if (motor_index_from_id(motor_cmd[i].id, &index) == 0U) {
      continue;
    }

    if (motor_cmd[i].mode != MOTOR_CMD_MODE_MIT) {
      continue;
    }

    motor[index].ctrl.mode = mit_mode;
    motor[index].ctrl.pos_set = motor_cmd[i].position;
    motor[index].ctrl.vel_set = motor_cmd[i].velocity;
    motor[index].ctrl.kp_set = motor_cmd[i].kp;
    motor[index].ctrl.kd_set = motor_cmd[i].kd;
    motor[index].ctrl.tor_set = motor_cmd[i].torque;
    pending_motor_mask = (uint16_t)(pending_motor_mask | (uint16_t)(1U << index));
  }
}

void motor_manager_process(void)
{
  uint8_t checked = 0U;
  uint8_t index;

  while ((pending_motor_mask != 0U) &&
         (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) &&
         (checked < MOTOR_MANAGER_COUNT)) {
    index = next_motor_index;
    next_motor_index = (uint8_t)((next_motor_index + 1U) % MOTOR_MANAGER_COUNT);
    checked++;

    if ((pending_motor_mask & (uint16_t)(1U << index)) == 0U) {
      continue;
    }

    dm_motor_ctrl_send(&hfdcan1, &motor[index]);
    pending_motor_mask = (uint16_t)(pending_motor_mask & (uint16_t)~(uint16_t)(1U << index));
  }
}
