#include "dm_motor_drv.h"
#include "fdcan.h"

void dm_motor_enable(hcan_t* hcan, motor_t *motor)
{
	enable_motor_mode(hcan, motor->id);
}

void dm_motor_disable(hcan_t* hcan, motor_t *motor)
{
	disable_motor_mode(hcan, motor->id);
	dm_motor_clear_para(motor);
}

void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor)
{
	switch (motor->ctrl.mode) {
		case mit_mode:
			mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set,
			         motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case pos_mode:
			pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case spd_mode:
			spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case psi_mode:
			psi_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
			break;
	}
}

void dm_motor_clear_para(motor_t *motor)
{
	motor->ctrl.kd_set = 0;
	motor->ctrl.kp_set = 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}

void dm_motor_clear_err(hcan_t* hcan, motor_t *motor)
{
	clear_err(hcan, motor->id);
}

void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = rx_data[0] & 0x0F;
	motor->para.state = rx_data[0] >> 4;
	motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
	motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	motor->para.Tmos = (float)rx_data[6];
	motor->para.Tcoil = (float)rx_data[7];
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

static float clamp_float(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	fdcanx_send_data(hcan, motor_id, data, 8);
}

void disable_motor_mode(hcan_t* hcan, uint16_t motor_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	fdcanx_send_data(hcan, motor_id, data, 8);
}

void save_pos_zero(hcan_t* hcan, uint16_t motor_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
	fdcanx_send_data(hcan, motor_id, data, 8);
}

void clear_err(hcan_t* hcan, uint16_t motor_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
	fdcanx_send_data(hcan, motor_id, data, 8);
}

void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	pos = clamp_float(pos, -motor->tmp.PMAX, motor->tmp.PMAX);
	vel = clamp_float(vel, -motor->tmp.VMAX, motor->tmp.VMAX);
	tor = clamp_float(tor, -motor->tmp.TMAX, motor->tmp.TMAX);
	kp = clamp_float(kp, KP_MIN, KP_MAX);
	kd = clamp_float(kd, KD_MIN, KD_MAX);

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);

	data[0] = pos_tmp >> 8;
	data[1] = pos_tmp;
	data[2] = vel_tmp >> 4;
	data[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = kd_tmp >> 4;
	data[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;
	fdcanx_send_data(hcan, motor_id + MIT_MODE, data, 8);
}

void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel)
{
	uint8_t data[8];
	uint8_t *pbuf = (uint8_t*)&pos;
	uint8_t *vbuf = (uint8_t*)&vel;

	data[0] = pbuf[0]; data[1] = pbuf[1]; data[2] = pbuf[2]; data[3] = pbuf[3];
	data[4] = vbuf[0]; data[5] = vbuf[1]; data[6] = vbuf[2]; data[7] = vbuf[3];
	fdcanx_send_data(hcan, motor_id + POS_MODE, data, 8);
}

void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
	uint8_t data[4];
	uint8_t *vbuf = (uint8_t*)&vel;

	data[0] = vbuf[0];
	data[1] = vbuf[1];
	data[2] = vbuf[2];
	data[3] = vbuf[3];
	fdcanx_send_data(hcan, motor_id + SPD_MODE, data, 4);
}

void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint8_t data[8];
	uint8_t *pbuf = (uint8_t*)&pos;
	uint16_t u16_vel = (uint16_t)(vel * 100);
	uint16_t u16_cur = (uint16_t)(cur * 10000);
	uint8_t *vbuf = (uint8_t*)&u16_vel;
	uint8_t *ibuf = (uint8_t*)&u16_cur;

	data[0] = pbuf[0]; data[1] = pbuf[1]; data[2] = pbuf[2]; data[3] = pbuf[3];
	data[4] = vbuf[0]; data[5] = vbuf[1];
	data[6] = ibuf[0]; data[7] = ibuf[1];
	fdcanx_send_data(hcan, motor_id + PSI_MODE, data, 8);
}

void read_motor_data(uint16_t id, uint8_t rid)
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}

void read_motor_ctrl_fbdata(uint16_t id)
{
	uint8_t can_id_l = id & 0xFF;
	uint8_t can_id_h = (id >> 8) & 0x07;
	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}

void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0xFF;
	uint8_t can_id_h = (id >> 8) & 0x07;
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 8);
}

void save_motor_data(uint16_t id, uint8_t rid)
{
	uint8_t can_id_l = id & 0xFF;
	uint8_t can_id_h = (id >> 8) & 0x07;
	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, rid};
	fdcanx_send_data(&hfdcan1, 0x7FF, data, 4);
}
