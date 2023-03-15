/*
    Copyright 2019 - 2023 Mitch Lustig
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "realtimedata.h"

#include "vesc_c_if.h"

#include <math.h>

// Can
#define MAX_CAN_AGE		0.1
#define MAX_CAN_DEVS	2

// Mathses
#define DEG2RAD_f(deg)		((deg) * (float)(M_PI / 180.0))
#define RAD2DEG_f(rad) 		((rad) * (float)(180.0 / M_PI))

void realtimedata_update(RealtimeData *data, balance_config *balance_conf) {

	// Set "last" values to previous loops values
	data->last_pitch_angle = data->pitch_angle;
	data->last_gyro_y = data->gyro[1];

	// Get the values we want
	data->motor_current = VESC_IF->mc_get_tot_current_directional_filtered();
	data->motor_position = VESC_IF->mc_get_pid_pos_now();
	data->pitch_angle = RAD2DEG_f(VESC_IF->imu_get_pitch());
	data->roll_angle = RAD2DEG_f(VESC_IF->imu_get_roll());
	data->abs_roll_angle = fabsf(data->roll_angle);
	data->abs_roll_angle_sin = sinf(DEG2RAD_f(data->abs_roll_angle));
	VESC_IF->imu_get_gyro(data->gyro);
	data->duty_cycle = VESC_IF->mc_get_duty_cycle_now();
	data->abs_duty_cycle = fabsf(data->duty_cycle);
	data->erpm = VESC_IF->mc_get_rpm();
	data->abs_erpm = fabsf(data->erpm);
	if (balance_conf->multi_esc) {
		data->avg_erpm = data->erpm;
		for (int i = 0;i < MAX_CAN_DEVS;i++) {
			can_status_msg *msg = VESC_IF->can_get_status_msg_index(i);
			if (msg->id >= 0 && VESC_IF->ts_to_age_s(msg->rx_time) < MAX_CAN_AGE) {
				data->avg_erpm += msg->rpm;
			}
		}

		data->avg_erpm = data->avg_erpm / 2.0; // Assume 2 motors, i don't know how to steer 3 anyways
	}

	data->adc1 = VESC_IF->io_read_analog(VESC_PIN_ADC1);
	data->adc2 = VESC_IF->io_read_analog(VESC_PIN_ADC2); // Returns -1.0 if the pin is missing on the hardware
	if (data->adc2 < 0.0) {
		data->adc2 = 0.0;
	}

	// Calculate switch state from ADC values
	if (balance_conf->fault_adc1 == 0 && balance_conf->fault_adc2 == 0){ // No Switch
		data->switch_state = ON;
	} else if (balance_conf->fault_adc2 == 0) { // Single switch on ADC1
		if (data->adc1 > balance_conf->fault_adc1) {
			data->switch_state = ON;
		} else {
			data->switch_state = OFF;
		}
	} else if (balance_conf->fault_adc1 == 0) { // Single switch on ADC2
		if (data->adc2 > balance_conf->fault_adc2) {
			data->switch_state = ON;
		} else {
			data->switch_state = OFF;
		}
	} else { // Double switch
		if (data->adc1 > balance_conf->fault_adc1 && data->adc2 > balance_conf->fault_adc2) {
			data->switch_state = ON;
		} else if (data->adc1 > balance_conf->fault_adc1 || data->adc2 > balance_conf->fault_adc2) {
			if (balance_conf->fault_is_dual_switch) {
				data->switch_state = ON;
			} else {
				data->switch_state = HALF;
			}
		} else {
			data->switch_state = OFF;
		}
	}
}
