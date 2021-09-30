
#ifndef __COMBIADAPTER_H__
#define __COMBIADAPTER_H__
#include "mbed.h"
#include <cstdint>


enum combi_command_t {
    cmd_brd_fwversion	= 0x20,
    cmd_brd_adcfilter	= 0x21,
    cmd_brd_adc			= 0x22,
    cmd_brd_egt			= 0x23,
    cmd_can_open		= 0x80,
    cmd_can_bitrate		= 0x81,
    cmd_can_frame		= 0x82,
    cmd_can_txframe		= 0x83,
    cmd_can_ecuconnect	= 0x89,
    cmd_can_readflash	= 0x8a,
    cmd_can_writeflash	= 0x8b,
    cmd_term_ack		= 0x00,
    cmd_term_nack		= 0xff
};

struct packet_t {
    uint8_t cmd_code;	// command code
    uint16_t data_len;	// data block length
    uint8_t *data;		// optional data block
    uint8_t term;		// terminator
};

struct combiCan_t {
    uint32_t id;
    uint8_t length;
	uint64_t data;
    uint8_t is_extended;
	uint8_t is_remote;
};

extern void combi_thread();

#endif