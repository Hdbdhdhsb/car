#ifndef _MAIN_H_
#define _MAIN_H_

#include "zf_common_headfile.h"
#include "Contact_master/contact_master.h"

#define UART4_TX UART4_TX_P03
#define UART4_RX UART4_RX_P02
#define UART2_TX UART2_TX_P11
#define UART2_RX UART2_RX_P10
#define CTRL_PIN IO_P14

#define DEBUG_DATA_UART         UART_4
#define LOWER_CTRL_UART         UART_4

#define WIFI_CMD_RX_BUF_SIZE        64U
#define WIFI_CMD_LINE_SIZE          32U
#define WIFI_CTRL_SEND_PERIOD_MS    50U
#define WIFI_CTRL_COMMAND_TIMEOUT_MS 800U

/* Default control values sent while the GUI keeps the car in run mode. */
#define LOWER_START_SPEED0          5.0f
#define LOWER_START_SPEED1          5.0f
#define LOWER_START_TURN            0.0f

/* Stop command required by the user request. */
#define LOWER_STOP_SPEED0           0.0f
#define LOWER_STOP_SPEED1           0.0f
#define LOWER_STOP_TURN             0.0f


extern uint32 g_sysTick_ms;   /* 系统心跳 */

void sys_tick_handler(void);

#endif 
