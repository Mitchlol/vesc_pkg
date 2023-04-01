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
#include "turntilt.h"
#include "../conf/datatypes.h"
#include "../util/realtimedata.h"
#include "../util/mathmacros.h"

#include <math.h>

void turntilt_configure(Turntilt *turntilt, balance_config *balance_conf){
	turntilt->step_size = balance_conf->turntilt_speed / balance_conf->hertz;
}

void turntilt_reset(Turntilt *turntilt){
	turntilt->target = 0;
	turntilt->interpolated = 0;
}

float turntilt_update(Turntilt *turntilt, RealtimeData *realtimedata, balance_config *balance_conf){
	// Calculate desired angle
	turntilt->target = realtimedata->abs_roll_angle_sin * balance_conf->turntilt_strength;

	// Apply cutzone
	if (realtimedata->abs_roll_angle < balance_conf->turntilt_start_angle) {
		turntilt->target = 0;
	}

	// Disable below erpm threshold otherwise add directionality
	if (realtimedata->abs_erpm < balance_conf->turntilt_start_erpm) {
		turntilt->target = 0;
	} else {
		turntilt->target *= SIGN(realtimedata->erpm);
	}

	// Apply speed scaling
	if (realtimedata->abs_erpm < balance_conf->turntilt_erpm_boost_end) {
		turntilt->target *= 1 + ((balance_conf->turntilt_erpm_boost / 100.0f) *
				(realtimedata->abs_erpm / balance_conf->turntilt_erpm_boost_end));
	} else {
		turntilt->target *= 1 + (balance_conf->turntilt_erpm_boost / 100.0f);
	}

	// Limit angle to max angle
	if (turntilt->target > 0) {
		turntilt->target = fminf(turntilt->target, balance_conf->turntilt_angle_limit);
	} else {
		turntilt->target = fmaxf(turntilt->target, -balance_conf->turntilt_angle_limit);
	}

	// Move towards target limited by max speed
	turntilt->interpolated = STEP_TOWARDS(turntilt->interpolated, turntilt->target, turntilt-> step_size);

	return turntilt->interpolated;
}
