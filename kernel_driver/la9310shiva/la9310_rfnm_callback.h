// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

#ifndef __LA9310_HOST_RFNM_CALLBACK_H__
#define __LA9310_HOST_RFNM_CALLBACK_H__

/*Call back function prototype*/
//typedef void (*rfnm_callback_t)(struct sk_buff *skb_ptr, void *cookie);

int register_rfnm_callback( void* callbackfunc, int irqid);
int unregister_rfnm_callback(void);

#endif
