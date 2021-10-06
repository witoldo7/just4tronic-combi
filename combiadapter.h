
#ifndef __COMBIADAPTER_H__
#define __COMBIADAPTER_H__
#include <cstdint>

enum combi_command_t {
    cmd_brd_fwversion     = 0x20,
    cmd_brd_adcfilter     = 0x21,
    cmd_brd_adc           = 0x22,
    cmd_brd_egt           = 0x23,
    cmd_bdm_stop_chip     = 0x40,
    cmd_bdm_reset_chip    = 0x41,
    cmd_bdm_run_chip      = 0x42,
    cmd_bdm_step_chip     = 0x43,
    cmd_bdm_restart_chip  = 0x44,
    cmd_bdm_mem_read      = 0x45,
    cmd_bdm_mem_write     = 0x46,
    cmd_bdm_sysreg_read   = 0x47,
    cmd_bdm_sysreg_write  = 0x48,
    cmd_bdm_adreg_read    = 0x49,
    cmd_bdm_adreg_write   = 0x4A,
    cmd_bdm_read_flash    = 0x4B,
    cmd_bdm_erase_flash   = 0x4C,
    cmd_bdm_write_flash   = 0x4D,
    cmd_bdm_pinstate      = 0x4E,
    cmd_can_open          = 0x80,
    cmd_can_bitrate       = 0x81,
    cmd_can_frame         = 0x82,
    cmd_can_txframe       = 0x83,
    cmd_can_ecuconnect    = 0x89,
    cmd_can_readflash     = 0x8a,
    cmd_can_writeflash    = 0x8b,
    cmd_term_ack          = 0x00,
    cmd_term_nack         = 0xff
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