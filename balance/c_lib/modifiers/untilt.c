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

#include "untilt.h"
#include "../util/realtimedata.h"
#include "../util/mathmacros.h"

#include <math.h>

void untilt_configure(Untilt *untilt, balance_config *balance_conf){
	untilt->step_size = balance_conf->startup_speed / balance_conf->hertz;
}
void untilt_reset(Untilt *untilt, RealtimeData *realtimedata){
	untilt->interpolated = realtimedata->pitch_angle;
}

float untilt_update(Untilt *untilt){
	untilt->interpolated = STEP_TOWARDS(untilt->interpolated, 0, untilt->step_size);

	return untilt->interpolated;
}

