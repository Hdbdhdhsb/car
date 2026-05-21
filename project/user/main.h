#ifndef _MAIN_H_
#define _MAIN_H_

#include "zf_common_headfile.h"
#include "Contact_master/contact_master.h"

#define UART2_TX UART2_TX_P11
#define UART2_RX UART2_RX_P10
#define CTRL_PIN IO_P14

extern uint32 g_sysTick_ms;   /* 系统心跳 */

void sys_tick_handler(void);

#endif 