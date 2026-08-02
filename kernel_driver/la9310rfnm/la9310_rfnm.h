// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

#ifndef __LA9310_RFNM_H__
#define __LA9310_RFNM_H__

#define LA9310_DEVICE_NAME_MAX_LEN	(12)
extern void ipc_test_usecase(const char *name);
extern volatile uint64_t num_of_bytes_tx;
extern volatile uint64_t num_of_bytes_rx;
int rfnm_schedule_restart_sm(void);
void rfnm_register_lalib_quiesce_cb(void (*cb)(void));

// /dev/rfnm_data_ep zero-copy protocol - values mirrored in librfnm rfnm_fw_api.h,
// keep the two in sync. The RX/TX packet pools are mmap'd from the chardev at the
// offsets below; the ioctls move pool indices only, never payload.
#define RFNM_LOCAL_RX_GET 2	// writes u32 pool index of the next filled RX packet to *arg
#define RFNM_LOCAL_RX_PUT 3	// arg = index: return the packet to the free pool
#define RFNM_LOCAL_TX_ACQ 4	// writes u32 pool index of a claimed free TX slot to *arg
#define RFNM_LOCAL_TX_SUB 5	// arg = index: submit the filled slot for transmission
#define RFNM_LOCAL_MMAP_RX_OFFSET 0x0UL
#define RFNM_LOCAL_MMAP_TX_OFFSET 0x10000000UL
#endif
