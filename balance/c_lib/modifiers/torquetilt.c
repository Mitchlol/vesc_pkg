/*
    Copyright 2023 Mitch Lustig
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
#include "torquetilt.h"
#include "../util/biquad.h"
#include "../util/realtimedata.h"
#include "../util/mathmacros.h"

#include <math.h>

void torquetilt_configure(Torquetilt *torquetilt, balance_config *balance_conf){
	torquetilt->on_step_size = balance_conf->torquetilt_on_speed / balance_conf->hertz;
	torquetilt->off_step_size = balance_conf->torquetilt_off_speed / balance_conf->hertz;

	if (balance_conf->torquetilt_filter > 0) { // Torquetilt Current Biquad
		float Fc = balance_conf->torquetilt_filter / balance_conf->hertz;
		biquad_config(&torquetilt->current_biquad, BQ_LOWPASS, Fc);
	}
}

void torquetilt_reset(Torquetilt *torquetilt){
	torquetilt->target = 0;
	torquetilt->interpolated = 0;
	torquetilt->filtered_current = 0;
	biquad_reset(&torquetilt->current_biquad);
}

float torquetilt_update(Torquetilt *torquetilt, RealtimeData *realtimedata, balance_config *balance_conf) {
	// Filter current (Biquad)
	if (balance_conf->torquetilt_filter > 0) {
		torquetilt->filtered_current = biquad_process(&torquetilt->current_biquad, realtimedata->motor_current);
	} else {
		torquetilt->filtered_current = realtimedata->motor_current;
	}

	// Wat is this line O_o
	// Take abs motor current, subtract start offset, and take the max of that with 0 to get the current above our start threshold (absolute).
	// Then multiply it by "power" to get our desired angle, and min with the limit to respect boundaries.
	// Finally multiply it by sign motor current to get directionality back
	torquetilt->target = fminf(fmaxf((fabsf(torquetilt->filtered_current) - balance_conf->torquetilt_start_current), 0) *
			balance_conf->torquetilt_strength, balance_conf->torquetilt_angle_limit) * SIGN(torquetilt->filtered_current);

	float step_size;
	if ((torquetilt->interpolated - torquetilt->target > 0 && torquetilt->target > 0) ||
			(torquetilt->interpolated - torquetilt->target < 0 && torquetilt->target < 0)) {
		step_size = torquetilt->off_step_size;
	} else {
		step_size = torquetilt->on_step_size;
	}

	torquetilt->interpolated = STEP_TOWARDS(torquetilt->interpolated, torquetilt->target, step_size);

	return torquetilt->interpolated;
}
