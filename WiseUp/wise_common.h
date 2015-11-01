/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 */

#pragma once

#include <stdint.h>

typedef struct {
	uint32_t uuid;
	uint8_t  id;
} item_address_t;

void
print_buffer (uint8_t * buffer, uint8_t size) {
	for (uint8_t i = 0; i < size; i++) {
		Serial.print (buffer[i], HEX);
	} Serial.println ();
}