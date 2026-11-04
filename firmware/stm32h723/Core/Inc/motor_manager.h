#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <stdint.h>

#define MOTOR_MANAGER_COUNT 12U

typedef enum
{
  MOTOR_CMD_MODE_MIT = 1U
} Motor_Command_Mode;

typedef struct
{
  uint8_t valid;
  uint8_t id;
  uint8_t mode;
  float position;
  float velocity;
  float kp;
  float kd;
  float torque;
} Motor_Command;

void motor_manager_init(void);
void motor_manager_enable_all(void);
void motor_manager_disable_all(void);
void motor_manager_damping_all(void);
void motor_manager_apply_commands(const Motor_Command motor_cmd[MOTOR_MANAGER_COUNT]);
void motor_manager_process(void);

#endif /* MOTOR_MANAGER_H */
