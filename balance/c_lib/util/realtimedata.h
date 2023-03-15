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
#ifndef REALTIMEDATA_H_
#define REALTIMEDATA_H_

#include "../conf/datatypes.h"

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

// Runtime values read from elsewhere
typedef struct{
	float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle, abs_roll_angle_sin, last_gyro_y;
	float gyro[3];
	float duty_cycle, abs_duty_cycle;
	float erpm, abs_erpm, avg_erpm;
	float motor_current;
	float motor_position;
	float adc1, adc2;
	SwitchState switch_state;
} RealtimeData;

void realtimedata_update(RealtimeData *data, balance_config *balance_conf);

#endif /* REALTIMEDATA_H_ */
