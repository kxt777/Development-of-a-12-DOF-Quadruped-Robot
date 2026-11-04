#include "usb_feedback.h"

#include "dm_motor_ctrl.h"
#include "motor_manager.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define USB_FB_MAGIC_0 'D'
#define USB_FB_MAGIC_1 'M'
#define USB_FB_MAGIC_2 '1'
#define USB_FB_MAGIC_3 '2'
#define USB_FB_TYPE_MOTOR_FEEDBACK 0x11U
#define USB_FB_RECORD_SIZE 23U
#define USB_FB_PACKET_SIZE (5U + (MOTOR_MANAGER_COUNT * USB_FB_RECORD_SIZE))
#define USB_FB_PERIOD_MS 20U

static uint8_t feedback_packet[USB_FB_PACKET_SIZE];
static uint32_t last_feedback_tick = 0U;

static void write_float_le(uint8_t *data, float value)
{
  uint8_t bytes[4];

  memcpy(bytes, &value, sizeof(value));
  data[0] = bytes[0];
  data[1] = bytes[1];
  data[2] = bytes[2];
  data[3] = bytes[3];
}

static void usb_feedback_build_packet(void)
{
  uint8_t i;
  uint16_t offset = 5U;

  feedback_packet[0] = USB_FB_MAGIC_0;
  feedback_packet[1] = USB_FB_MAGIC_1;
  feedback_packet[2] = USB_FB_MAGIC_2;
  feedback_packet[3] = USB_FB_MAGIC_3;
  feedback_packet[4] = USB_FB_TYPE_MOTOR_FEEDBACK;

  for (i = 0U; i < MOTOR_MANAGER_COUNT; i++) {
    feedback_packet[offset] = 1U;
    feedback_packet[offset + 1U] = (uint8_t)motor[i].id;
    feedback_packet[offset + 2U] = (uint8_t)motor[i].para.state;
    write_float_le(&feedback_packet[offset + 3U], motor[i].para.pos);
    write_float_le(&feedback_packet[offset + 7U], motor[i].para.vel);
    write_float_le(&feedback_packet[offset + 11U], motor[i].para.tor);
    write_float_le(&feedback_packet[offset + 15U], motor[i].para.Tmos);
    write_float_le(&feedback_packet[offset + 19U], motor[i].para.Tcoil);
    offset = (uint16_t)(offset + USB_FB_RECORD_SIZE);
  }
}

void usb_feedback_process(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - last_feedback_tick) < USB_FB_PERIOD_MS) {
    return;
  }

  last_feedback_tick = now;
  usb_feedback_build_packet();
  (void)CDC_Transmit_HS(feedback_packet, USB_FB_PACKET_SIZE);
}
