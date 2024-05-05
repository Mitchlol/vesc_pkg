// This file is autogenerated by VESC Tool

#ifndef CONFPARSER_H_
#define CONFPARSER_H_

#include "datatypes.h"
#include <stdint.h>
#include <stdbool.h>

// Constants
#define BALANCE_CONFIG_SIGNATURE		2790061407

// Functions
int32_t confparser_serialize_balance_config(uint8_t *buffer, const balance_config *conf);
bool confparser_deserialize_balance_config(const uint8_t *buffer, balance_config *conf);
void confparser_set_defaults_balance_config(balance_config *conf);

// CONFPARSER_H_
#endif
