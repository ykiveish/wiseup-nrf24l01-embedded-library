/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 */

#pragma once

#include <stdint.h>

#define DATA_PACKAGE_SIZE           7
#define RFCOMM_VERSION              0x1

/* Data type */
#define SENSOR_INFO_DATA_NO_AUTH_TYPE   0x0
#define SENSOR_INFO_DATA_TYPE           0x1
#define SENSOR_CMD_DATA_TYPE            0x2
#define DEVICE_PROT_DATA_TYPE           0x3

/* Sensor command type */
#define SENSOR_CMD_RELAY		    0x1
#define SENSOR_CMD_RELAY_RGB	    0x2

/* Device protocol commands */
#define DEVICE_PROT_CONNECT_REQ     0x1
#define DEVICE_PROT_CONNECT_ADDR    0x2
#define DEVICE_PROT_CONNECT_CHK     0x99

/* Sizes */
#define SENSOR_INFO_DATA_SIZE	    0x5
#define DEVICE_PROT_CONN_DATA_SIZE  0x1
#define SENSOR_CMD_DATA_TYPE_SIZE	0x6

/* Sensors type */
#define TEMPERATURE_SENSOR_TYPE     0x1
#define LUMINANCE_SENSOR_TYPE 	    0x2
#define PIR_SENSOR_TYPE             0x3
#define RELAY_SENSOR_TYPE		    0x4
#define RGB_LED_SENSOR_TYPE		    0x5

/* Sender types */
#define SENDER_SENSOR_LOCAL_HUB    	0x1
#define SENDER_SENSOR_WIRELESS_HUB  0x2

struct device_version {
	uint8_t major	:4;
	uint8_t minor	:4;
};

/*struct rfcomm_sensor_info {
    uint8_t    	sensor_address;
    uint8_t    	sensor_type;
	uint16_t	sensor_update_interval;
    uint8_t    	sensor_data_len;
};*/

struct rfcomm_individual_sensor_info {
	uint8_t 	sensor_address[6];
	uint8_t		sensor_type;
	uint16_t	sensor_update_interval;
	uint8_t		sensor_data_len;
};

struct rfcomm_sensor_command {
	uint8_t 	sensor_address;
	uint8_t		command_type;
    uint8_t     command_data[4];
};

struct rfcomm_device_prot {
    uint8_t 	device_cmd;
    uint8_t     device_data[8];
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct rfcomm_sensor_info {
    uint8_t    	port;
    uint8_t    	type;
    uint8_t    	data_len;
};

enum DATA_TYPE {
	SERVER_ADDRESS_BROADCAST,
	CLIENT_ID_REQUEST,
	CLIENT_ID_RESPONSE,
	CLIENT_SENSOR_DATA
};

struct rfcomm_data {
	struct control { 
		uint16_t is_fragmeneted	: 1;
		uint16_t version		: 3;
		uint16_t is_broadcast	: 1;
		uint16_t is_ack			: 1;
		uint16_t reserved		: 10;
	};
	
	struct data_info {
		uint16_t data_size		: 5;
		uint16_t data_type		: 11;
	};
	
	struct sender_info {
		uint16_t sender_type	: 5;
		uint16_t reserved		: 11;
	};
	
	union data_package {
		struct _unframeneted {
			uint8_t 	data[DATA_PACKAGE_SIZE];
		} unframeneted;
		
		struct _fragmented {
			uint8_t		frame_number;
			uint8_t		data[DATA_PACKAGE_SIZE - 1];
		} fragmented;
	};
	
	uint16_t		magic_number;		// 2  byte
	control 		control_flags;		// 1  byte
	data_info		data_information;	// 2  byte
	sender_info		sender_information; // 2  byte
	data_package 	data_frame;			// 7  byte
	uint16_t		packet_id;			// 2  byte
};

void
build_packet (uint8_t * buffer, enum DATA_TYPE type) {
	rfcomm_data * packet = (rfcomm_data *) buffer;

	memcpy (buffer, 0x0, 32);
	packet->magic_number = 0xAABB;
	packet->control_flags.version = RFCOMM_VERSION;

	switch (type) {
		case SERVER_ADDRESS_BROADCAST:
			packet->data_information.data_size = 0x5;
			packet->data_information.data_type = SERVER_ADDRESS_BROADCAST;
			break;
		case CLIENT_ID_REQUEST:
			packet->control_flags.is_ack 	   = 0x1;
			packet->data_information.data_size = 0x4;
			packet->data_information.data_type = CLIENT_ID_REQUEST;
			break;
		case CLIENT_ID_RESPONSE:
			packet->data_information.data_size = 0x1;
			packet->data_information.data_type = CLIENT_ID_RESPONSE;
			break;
		case CLIENT_SENSOR_DATA:
            packet->data_information.data_size = 0x1;
			packet->data_information.data_type = CLIENT_SENSOR_DATA;
			break;
	}
}