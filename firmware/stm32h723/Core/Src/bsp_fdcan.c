#include "bsp_fdcan.h"

volatile uint32_t can_tx_fail_count = 0;
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_bus_off_count = 0;
volatile uint32_t can_error_warning_count = 0;
volatile uint16_t last_rec_id = 0;
volatile uint8_t last_rx_data[8] = {0};
volatile uint32_t last_hal_error = 0;
volatile uint8_t can_last_tx_ret = 0;
volatile uint32_t can_last_hal_error = 0;
volatile uint32_t can_tx_fifo_free = 0;

void bsp_can_init(void)
{
	can_filter_init();

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1,
	                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE
	                                     | FDCAN_IT_TX_COMPLETE
	                                     | FDCAN_IT_TX_FIFO_EMPTY
	                                     | FDCAN_IT_BUS_OFF
	                                     | FDCAN_IT_ARB_PROTOCOL_ERROR
	                                     | FDCAN_IT_DATA_PROTOCOL_ERROR
	                                     | FDCAN_IT_ERROR_PASSIVE
	                                     | FDCAN_IT_ERROR_WARNING,
	                                   0) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
	}
}

void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;

	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fdcan_filter.FilterID1 = 0x00;
	fdcan_filter.FilterID2 = 0x00;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
	}
	if (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
	}
}

void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
	uint32_t nom_brp = 1, nom_seg1 = 59, nom_seg2 = 20, nom_sjw = 20;
	uint32_t dat_brp = 1, dat_seg1 = 29, dat_seg2 = 10, dat_sjw = 10;
	(void)mode;

	switch (baud) {
		case CAN_BR_125K: nom_brp = 4; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break;
		case CAN_BR_200K: nom_brp = 2; nom_seg1 = 174; nom_seg2 = 25; nom_sjw = 25; break;
		case CAN_BR_250K: nom_brp = 2; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break;
		case CAN_BR_500K: nom_brp = 1; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break;
		case CAN_BR_1M:
		default:          nom_brp = 1; nom_seg1 = 59;  nom_seg2 = 20; nom_sjw = 20; break;
	}

	if (HAL_FDCAN_DeInit(hfdcan) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(hfdcan);
	}

	hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan->Init.NominalPrescaler = nom_brp;
	hfdcan->Init.NominalTimeSeg1 = nom_seg1;
	hfdcan->Init.NominalTimeSeg2 = nom_seg2;
	hfdcan->Init.NominalSyncJumpWidth = nom_sjw;
	hfdcan->Init.DataPrescaler = dat_brp;
	hfdcan->Init.DataTimeSeg1 = dat_seg1;
	hfdcan->Init.DataTimeSeg2 = dat_seg2;
	hfdcan->Init.DataSyncJumpWidth = dat_sjw;

	if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
		last_hal_error = HAL_FDCAN_GetError(hfdcan);
	}
}

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef pTxHeader;

	if (len > 8) {
		can_tx_fail_count++;
		can_last_tx_ret = 1;
		can_last_hal_error = HAL_FDCAN_GetError(hfdcan);
		can_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
		return can_last_tx_ret;
	}

	pTxHeader.Identifier = id;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength = len;
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) {
		can_tx_fail_count++;
		last_hal_error = HAL_FDCAN_GetError(hfdcan);
		can_last_tx_ret = 1;
		can_last_hal_error = last_hal_error;
		can_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
		return can_last_tx_ret;
	}
	can_last_tx_ret = 0;
	can_last_hal_error = HAL_FDCAN_GetError(hfdcan);
	can_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
	return can_last_tx_ret;
}

uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len = 0;
	uint8_t i;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK) {
		*rec_id = pRxHeader.Identifier;
		can_rx_count++;
		last_rec_id = *rec_id;
		for (i = 0; i < 8; i++) {
			last_rx_data[i] = buf[i];
		}
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8) {
			len = pRxHeader.DataLength;
		}
		return len;
	}
	return 0;
}

__weak void fdcan1_rx_callback(void)
{
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	(void)RxFifo0ITs;
	if (hfdcan == &hfdcan1) {
		fdcan1_rx_callback();
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (ErrorStatusITs & FDCAN_IR_BO) {
		can_bus_off_count++;
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
	if (ErrorStatusITs & (FDCAN_IR_EP | FDCAN_IR_EW)) {
		can_error_warning_count++;
		last_hal_error = HAL_FDCAN_GetError(hfdcan);
	}
}
