// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

#ifndef __LA9310_RFNM_LALIB_H__
#define __LA9310_RFNM_LALIB_H__


#define TCML_START 0x1c000000
#define HIF_OFFSET 0x1C000
#define HIF_START  ( TCML_START + HIF_OFFSET )
#define HIF_SIZE   0x4000

#define CCSR_START 0x18000000
#define CCSR_SIZE  0x10000000


#pragma pack(push)




#define RF_SWCMD_DATA_SIZE    12







#define RF_SWCMD_TIMEOUT_SECS    30
#define RF_SWCMD_TIMEOUT_RETRIES 5
#define RFDEV_NAME_LEN 32

#define SET_PARAM(x) data->x = x;
#define GET_PARAM(x) *x = data->x;

#define RF_API_PRE(type, cmdid) \
    type * data; \
    struct rf_sw_cmd_desc sw_cmd = {0}; \
    int ret; \
    sw_cmd.cmd = cmdid; \
    data = (type *)&sw_cmd.data[0]; \

#define RF_API_SEND(type) \
    ret = rf_send_swcmd(rfdev, &sw_cmd, sizeof(type));

#define RF_API_END \
    rf_free_cmd(rfdev, &sw_cmd); \
    return ret; \

struct rf_host_stats {
    int sw_cmds_tx;
    int sw_cmds_failed;
    int sw_cmds_timed_out;
    int sw_cmds_desc_busy;
};

typedef enum modem_endianness {
    MOD_BE,
    MOD_LE
} mod_endian_t;

typedef enum modem_type {
    MOD_GEUL,
    MOD_LA9310,
} mod_type_t;

struct rfdevice {
    char name[RFDEV_NAME_LEN];
    void *hif_p;
    void *rfic_p;
    void *ccsr_p;
    void *scr_p;
    void *scrmodshare_p;
    void *cal_p;
    void *iqdata_p;
    mod_endian_t endian;
    mod_type_t type;
    int act_modem_cnt;
    void *mil_ptr;
	struct rf_host_stats host_stats;
};


#pragma pack(pop)

void rfnm_la9310_quiesce(void);
void rfnm_la9310_stream_regate(void);
int rfnm_la9310_tdd(uint32_t period_chunks, uint32_t duty_chunks);

// TX health snapshot: walker counters from the TCM request ring + the AXIQ/gate
// registers docs-5q proved host-visible. 0xffffffff fields = MMIO fenced / ring unmapped.
struct rfnm_tx_health {
	uint32_t txn_prod;
	uint32_t txn_cons;
	uint32_t txn_executed;
	uint32_t txn_missed;
	uint32_t txn_rejected;
	uint32_t txn_qs_stuck;
	uint32_t axiq_sr1;	// GPIN1: TX0 nibble bits[19:16] = {ERROVER, ERRUNDER, NOTFULL, ENABLED}
	uint32_t axiq_cr3;	// GPOUT7: bit0 = TX0 fifo enable command
	uint32_t c11_sc;	// TX_ALLOWED comparator SC readback: bit31 = live output level
};
void rfnm_la9310_tx_health(struct rfnm_tx_health *h);

#define LA9310_RF_SW_CMD_MSG_UNIT_BIT    ( 1 )

#endif
