#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

#define CAN_CLASS   0
#define CAN_FD_BRS  1

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9

extern volatile uint32_t can_tx_fail_count;
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can_bus_off_count;
extern volatile uint32_t can_error_warning_count;
extern volatile uint16_t last_rec_id;
extern volatile uint8_t last_rx_data[8];
extern volatile uint32_t last_hal_error;
extern volatile uint8_t can_last_tx_ret;
extern volatile uint32_t can_last_hal_error;
extern volatile uint32_t can_tx_fifo_free;

void bsp_can_init(void);
void can_filter_init(void);
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#endif /* __BSP_FDCAN_H_ */
