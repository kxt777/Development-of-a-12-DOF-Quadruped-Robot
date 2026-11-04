#include "usb_protocol.h"

#include "motor_manager.h"
#include "usbd_cdc_if.h"
#include <float.h>
#include <string.h>

#define USB_PROTO_MAGIC_0 'D'
#define USB_PROTO_MAGIC_1 'M'
#define USB_PROTO_MAGIC_2 '1'
#define USB_PROTO_MAGIC_3 '2'
#define USB_PROTO_TYPE_MOTOR_CONTROL 0x10U
#define USB_PROTO_TYPE_GLOBAL_COMMAND 0x20U
#define USB_PROTO_RECORD_SIZE 23U
#define USB_PROTO_PACKET_SIZE (5U + (MOTOR_MANAGER_COUNT * USB_PROTO_RECORD_SIZE))
#define USB_PROTO_GLOBAL_COMMAND_SIZE 6U

static uint8_t stream_rx_buf[USB_CMD_BUF_SIZE];
static uint16_t stream_rx_len = 0U;
static uint8_t cdc_snapshot_buf[USB_CMD_BUF_SIZE];
static Motor_Command motor_cmd[MOTOR_MANAGER_COUNT];
static volatile uint8_t motor_cmd_ready = 0U;
static volatile uint8_t global_cmd_ready = 0U;
static uint8_t global_cmd = 0U;

static void usb_protocol_reply(const char *text)
{
  (void)CDC_Transmit_HS((uint8_t *)text, (uint16_t)strlen(text));
}

static float read_float_le(const uint8_t *data)
{
  float value;
  uint8_t bytes[4];

  bytes[0] = data[0];
  bytes[1] = data[1];
  bytes[2] = data[2];
  bytes[3] = data[3];
  memcpy(&value, bytes, sizeof(value));
  return value;
}

static uint8_t float_is_finite(float value)
{
  return ((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX)) ? 1U : 0U;
}

static uint8_t validate_motor_control_packet(const uint8_t *data)
{
  uint8_t i;
  uint16_t offset = 5U;
  uint16_t id_mask = 0U;
  uint8_t valid;
  uint8_t id;
  uint8_t mode;
  float position;
  float velocity;
  float kp;
  float kd;
  float torque;
  uint16_t bit;

  if ((data[0] != USB_PROTO_MAGIC_0) ||
      (data[1] != USB_PROTO_MAGIC_1) ||
      (data[2] != USB_PROTO_MAGIC_2) ||
      (data[3] != USB_PROTO_MAGIC_3) ||
      (data[4] != USB_PROTO_TYPE_MOTOR_CONTROL)) {
    return 0U;
  }

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    valid = data[offset];
    id = data[offset + 1U];
    mode = data[offset + 2U];
    position = read_float_le(&data[offset + 3U]);
    velocity = read_float_le(&data[offset + 7U]);
    kp = read_float_le(&data[offset + 11U]);
    kd = read_float_le(&data[offset + 15U]);
    torque = read_float_le(&data[offset + 19U]);

    if ((valid != 0U) && (valid != 1U)) {
      return 0U;
    }

    if (valid == 0U) {
      offset = (uint16_t)(offset + USB_PROTO_RECORD_SIZE);
      continue;
    }

    if ((id < 1U) || (id > MOTOR_MANAGER_COUNT)) {
      return 0U;
    }

    if (mode != MOTOR_CMD_MODE_MIT) {
      return 0U;
    }

    bit = (uint16_t)(1U << (id - 1U));
    if ((id_mask & bit) != 0U) {
      return 0U;
    }
    id_mask = (uint16_t)(id_mask | bit);

    if ((float_is_finite(position) == 0U) ||
        (float_is_finite(velocity) == 0U) ||
        (float_is_finite(kp) == 0U) ||
        (float_is_finite(kd) == 0U) ||
        (float_is_finite(torque) == 0U) ||
        (kp < 0.0f) || (kp > 500.0f) ||
        (kd < 0.0f) || (kd > 5.0f)) {
      return 0U;
    }

    offset = (uint16_t)(offset + USB_PROTO_RECORD_SIZE);
  }

  return 1U;
}

static void parse_motor_control_packet(const uint8_t *data)
{
  uint8_t i;
  uint16_t offset = 5U;

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    motor_cmd[i].valid = data[offset];
    motor_cmd[i].id = data[offset + 1U];
    motor_cmd[i].mode = data[offset + 2U];
    motor_cmd[i].position = read_float_le(&data[offset + 3U]);
    motor_cmd[i].velocity = read_float_le(&data[offset + 7U]);
    motor_cmd[i].kp = read_float_le(&data[offset + 11U]);
    motor_cmd[i].kd = read_float_le(&data[offset + 15U]);
    motor_cmd[i].torque = read_float_le(&data[offset + 19U]);
    offset = (uint16_t)(offset + USB_PROTO_RECORD_SIZE);
  }

  motor_cmd_ready = 1U;
}

static void parse_global_command_packet(const uint8_t *data)
{
  global_cmd = data[5];
  global_cmd_ready = 1U;
}

static void trim_ascii(uint8_t *data, uint16_t *len)
{
  while ((*len > 0U) &&
         ((data[*len - 1U] == '\r') || (data[*len - 1U] == '\n') ||
          (data[*len - 1U] == ' ') || (data[*len - 1U] == '\t'))) {
    data[*len - 1U] = '\0';
    (*len)--;
  }
}

static void process_ascii_command(uint8_t *data, uint16_t len)
{
  trim_ascii(data, &len);
  data[len] = '\0';

  if (strcmp((char *)data, "PING") == 0) {
    usb_protocol_reply("PONG\r\n");
  } else if (strcmp((char *)data, "DM12") == 0) {
    usb_protocol_reply("PROTO OK\r\n");
  } else {
    usb_protocol_reply("ERR UNKNOWN_CMD\r\n");
  }
}

static void stream_drop(uint16_t len)
{
  if (len >= stream_rx_len) {
    stream_rx_len = 0U;
    return;
  }

  memmove(stream_rx_buf, &stream_rx_buf[len], (uint16_t)(stream_rx_len - len));
  stream_rx_len = (uint16_t)(stream_rx_len - len);
}

static int16_t stream_find_header(void)
{
  uint16_t i;

  if (stream_rx_len < 4U) {
    return -1;
  }

  for (i = 0U; i <= (uint16_t)(stream_rx_len - 4U); i++) {
    if ((stream_rx_buf[i] == USB_PROTO_MAGIC_0) &&
        (stream_rx_buf[i + 1U] == USB_PROTO_MAGIC_1) &&
        (stream_rx_buf[i + 2U] == USB_PROTO_MAGIC_2) &&
        (stream_rx_buf[i + 3U] == USB_PROTO_MAGIC_3)) {
      return (int16_t)i;
    }
  }

  return -1;
}

static int16_t stream_find_newline(uint16_t limit)
{
  uint16_t i;

  if (limit > stream_rx_len) {
    limit = stream_rx_len;
  }

  for (i = 0U; i < limit; i++) {
    if ((stream_rx_buf[i] == '\n') || (stream_rx_buf[i] == '\r')) {
      return (int16_t)i;
    }
  }

  return -1;
}

static void stream_process_ascii_line(uint16_t line_len)
{
  uint8_t ascii_buf[USB_CMD_BUF_SIZE];

  if (line_len >= USB_CMD_BUF_SIZE) {
    line_len = USB_CMD_BUF_SIZE - 1U;
  }

  memcpy(ascii_buf, stream_rx_buf, line_len);
  ascii_buf[line_len] = '\0';
  process_ascii_command(ascii_buf, line_len);

  while ((line_len < stream_rx_len) &&
         ((stream_rx_buf[line_len] == '\r') || (stream_rx_buf[line_len] == '\n'))) {
    line_len++;
  }

  stream_drop(line_len);
}

static void stream_append(const uint8_t *data, uint16_t len)
{
  uint16_t free_len;

  if (len > USB_CMD_BUF_SIZE) {
    data = &data[len - USB_CMD_BUF_SIZE];
    len = USB_CMD_BUF_SIZE;
  }

  free_len = (uint16_t)(USB_CMD_BUF_SIZE - stream_rx_len);
  if (len > free_len) {
    stream_drop((uint16_t)(len - free_len));
  }

  memcpy(&stream_rx_buf[stream_rx_len], data, len);
  stream_rx_len = (uint16_t)(stream_rx_len + len);
}

static void stream_parse(void)
{
  int16_t header_pos;
  int16_t newline_pos;

  for (;;) {
    header_pos = stream_find_header();

    if (header_pos < 0) {
      newline_pos = stream_find_newline(stream_rx_len);

      if (newline_pos >= 0) {
        stream_process_ascii_line((uint16_t)newline_pos);
        continue;
      }

      if (stream_rx_len > 3U) {
        stream_drop((uint16_t)(stream_rx_len - 3U));
      }
      return;
    }

    if (header_pos > 0) {
      newline_pos = stream_find_newline((uint16_t)header_pos);

      if (newline_pos >= 0) {
        stream_process_ascii_line((uint16_t)newline_pos);
      } else {
        stream_drop((uint16_t)header_pos);
      }
      continue;
    }

    if (stream_rx_len < 5U) {
      return;
    }

    if (stream_rx_buf[4] == USB_PROTO_TYPE_GLOBAL_COMMAND) {
      if (stream_rx_len < USB_PROTO_GLOBAL_COMMAND_SIZE) {
        return;
      }

      parse_global_command_packet(stream_rx_buf);
      stream_drop(USB_PROTO_GLOBAL_COMMAND_SIZE);
      continue;
    }

    if (stream_rx_buf[4] != USB_PROTO_TYPE_MOTOR_CONTROL) {
      stream_drop(1U);
      continue;
    }

    if (stream_rx_len < USB_PROTO_PACKET_SIZE) {
      return;
    }

    if (validate_motor_control_packet(stream_rx_buf) != 0U) {
      parse_motor_control_packet(stream_rx_buf);
      stream_drop(USB_PROTO_PACKET_SIZE);
    } else {
      usb_protocol_reply("MOTOR CMD ERROR\r\n");
      stream_drop(1U);
    }
  }
}

void usb_protocol_process(void)
{
  uint16_t len;

  if (usb_cmd_ready == 0U) {
    stream_parse();
    return;
  }

  __disable_irq();
  len = rx_accumulated_len;
  if (len > USB_CMD_BUF_SIZE) {
    len = USB_CMD_BUF_SIZE;
  }
  memcpy(cdc_snapshot_buf, (const void *)usb_cmd_buf, len);
  rx_accumulated_len = 0U;
  usb_cmd_len = 0U;
  usb_cmd_ready = 0U;
  __enable_irq();

  if (len > 0U) {
    stream_append(cdc_snapshot_buf, len);
  }
  stream_parse();
}

uint8_t usb_protocol_take_motor_command_ready(void)
{
  uint8_t ready;

  __disable_irq();
  ready = motor_cmd_ready;
  motor_cmd_ready = 0U;
  __enable_irq();

  return ready;
}

const Motor_Command *usb_protocol_get_motor_commands(void)
{
  return motor_cmd;
}

uint8_t usb_protocol_take_global_command(uint8_t *cmd)
{
  uint8_t ready;

  __disable_irq();
  ready = global_cmd_ready;
  if (ready != 0U) {
    *cmd = global_cmd;
    global_cmd_ready = 0U;
  }
  __enable_irq();

  return ready;
}
