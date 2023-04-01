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
#ifndef TORQUETILT_H_
#define TORQUETILT_H_

#include "../conf/datatypes.h"
#include "../util/biquad.h"
#include "../util/realtimedata.h"

typedef struct{
	// Computed config values
	float on_step_size, off_step_size;
	// State
	float filtered_current, target, interpolated;
	Biquad current_biquad;
} Torquetilt;

void torquetilt_configure(Torquetilt *torquetilt, balance_config *balance_conf);
void torquetilt_reset(Torquetilt *torquetilt);
float torquetilt_update(Torquetilt *torquetilt, RealtimeData *realtimedata, balance_config *balance_conf);


#endif /* TORQUETILT_H_ */
