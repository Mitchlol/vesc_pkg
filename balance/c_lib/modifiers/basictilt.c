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

#include "basictilt.h"
#include "../util/realtimedata.h"
#include "../util/mathmacros.h"

#include <math.h>

void basictilt_configure(Basictilt *basictilt, balance_config *balance_conf){
	basictilt->step_size = balance_conf->noseangling_speed / balance_conf->hertz;
}
void basictilt_reset(Basictilt *basictilt){
	basictilt->interpolated = 0;
}

float basictilt_update(Basictilt *basictilt, RealtimeData *realtimedata, balance_config *balance_conf){
	float basictilt_target = 0;

	if (realtimedata->erpm > balance_conf->tiltback_constant_erpm) {
		basictilt_target = balance_conf->tiltback_constant;
	} else if (realtimedata->erpm < -balance_conf->tiltback_constant_erpm){
		basictilt_target = -balance_conf->tiltback_constant;
	}

	basictilt->interpolated = STEP_TOWARDS(basictilt->interpolated, basictilt_target, basictilt->step_size);

	return basictilt->interpolated;
}

