#ifndef CONTACT_MASTER_H
#define CONTACT_MASTER_H

#include "zf_common_headfile.h"
#include "PID/PID.h"

#define FRAME_HEAD       0xAA
#define RX_BUF_SIZE      128               /* 最大数据长度 */
#define FRAME_CRC_LEN    1                /* CRC8 宽度 */

/* 发送缓冲区最大尺寸 */
#define MAX_PAYLOAD_LEN  (1 + 2 * sizeof(pid_ctrl_parameter_t))  /* 载荷最大长度 */
#define MAX_TX_BUF_SIZE  (1 + 1 + MAX_PAYLOAD_LEN + FRAME_CRC_LEN)

typedef enum {
    CMD_ERROR_IN        = 0x01,
    CMD_CONTROL_IN      = 0x02,
    CMD_PID_DEBUG_IN    = 0x03
} cmd_type_t;

typedef enum {
    PID_SPEED0 = 1,     /* 速度 PID */
    PID_SPEED1 = 2,     /* 速度 PID */
    PID_TURN  = 3       /* 转向 PID */
} pid_debug_type_t;

typedef struct {
    pid_debug_type_t type;   
    pid_ctrl_parameter_t pid_cfg;
} pid_debug_data_t;

/**
 * @brief 发送误差数据帧（上位机调试用）
 * @param error_input 当前误差值
 * @param target_speed 当前目标速度
 * @param uart 发送使用的 UART 索引（uart_index_enum 枚举）
 */
void send_error_data(float error_input, float target_speed, uart_index_enum uart);

/**
 * @brief 直接指令控制(上位机调试用)
 * @param target_speed0 目标速度0（例如左轮）
 * @param target_speed1 目标速度1（例如右轮）
 * @param target_turn 目标转向角度
 * @param uart 发送使用的 UART 索引
 */
void send_ctrl_data(float target_speed0, float target_speed1, float target_turn, uart_index_enum uart);

/**
 * @brief PID参数调节数据发送（上位机调试用）
 * @param pid_speed PID类型
 * @param pid_turn  PID 参数结构体指针
 * @param uart 发送使用的 UART 索引
 */
void send_pid_debug_data(pid_debug_type_t type, const pid_ctrl_parameter_t *param, uart_index_enum uart);
/**
 * @brief 使能下位机
 * @param ctrl_pin 使能控制引脚，传入对应的 gpio_pin_enum 枚举值
 */
void enable_control(gpio_pin_enum ctrl_pin);

#endif