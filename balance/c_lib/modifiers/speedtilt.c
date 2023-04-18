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

#include "speedtilt.h"
#include "../util/realtimedata.h"
#include "../util/mathmacros.h"

#include <math.h>

void speedtilt_configure(Speedtilt *speedtilt, balance_config *balance_conf){
	speedtilt->step_size = balance_conf->noseangling_speed / balance_conf->hertz;

	// Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
	speedtilt->degrees_per_erpm = balance_conf->tiltback_variable / 1000;
}
void speedtilt_reset(Speedtilt *speedtilt){
	speedtilt->interpolated = 0;
}

float speedtilt_update(Speedtilt *speedtilt, RealtimeData *realtimedata, balance_config *balance_conf){
	float speedtilt_target = 0;

	// Speed based tilt
	speedtilt_target = speedtilt->degrees_per_erpm * realtimedata->erpm;
	speedtilt_target = fminf(fabsf(speedtilt_target), fabsf(balance_conf->tiltback_variable_max)) * SIGN(speedtilt_target);

	speedtilt->interpolated = STEP_TOWARDS(speedtilt->interpolated, speedtilt_target, speedtilt->step_size);

	return speedtilt->interpolated;
}

