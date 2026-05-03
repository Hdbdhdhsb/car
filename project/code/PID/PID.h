#ifndef PID_H
#define PID_H

#include "zf_common_typedef.h"

#define PID_OK       0   /* 成功 */
#define PID_ERR_INV  1   /* 参数错误 */
#define PID_ERR_MEM  2   /* 内存不足 */

/* PID 计算类型 */
typedef enum {
    PID_CAL_TYPE_INCREMENTAL,   /* 增量式 */
    PID_CAL_TYPE_POSITIONAL     /* 位置式 */
} pid_calculate_type_t;

/* PID 控制块句柄（不透明指针） */
typedef struct pid_ctrl_block_t *pid_ctrl_block_handle_t;

/* PID 参数 */
typedef struct {
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float kd;                   /* 微分系数 */
    float max_output;           /* 输出上限 */
    float min_output;           /* 输出下限 */
    float max_integral;         /* 积分上限 */
    float min_integral;         /* 积分下限 */
    pid_calculate_type_t cal_type; /* 计算类型 */
} pid_ctrl_parameter_t;




/**
 * @brief 创建新的 PID 控制块
 * @param config PID 初始化配置结构体指针（包含 PID 参数与计算类型）
 * @param ret_pid 用于返回创建的 PID 控制块句柄的指针
 * @return uint8 PID_OK(0) 表示创建成功，PID_ERR_INV(1) 表示参数无效，PID_ERR_MEM(2) 表示内存不足
 */
uint8 pid_new_control_block(const pid_ctrl_parameter_t *init_param,
                            pid_ctrl_block_handle_t *ret_pid);

/**
 * @brief 删除指定的 PID 控制块，释放其资源
 * @param pid 需要删除的 PID 控制块句柄（由 pid_new_control_block 创建）
 * @return uint8 PID_OK(0) 表示删除成功，PID_ERR_INV(1) 表示句柄无效
 */
uint8 pid_del_control_block(pid_ctrl_block_handle_t pid);

/**
 * @brief 更新指定 PID 控制块的参数（Kp, Ki, Kd, 限幅、计算类型等）
 * @param pid 目标 PID 控制块句柄
 * @param params 指向新的 PID 参数结构体的指针
 * @return uint8 PID_OK(0) 表示更新成功，PID_ERR_INV(1) 表示参数无效
 */
uint8 pid_update_parameters(pid_ctrl_block_handle_t pid,
                            const pid_ctrl_parameter_t *params);

/**
 * @brief 执行 PID 计算，得到控制输出
 * @param pid PID 控制块句柄
 * @param input_error 本次误差（设定值 - 实际值）
 * @param ret_result 用于存储计算得到的控制输出（float 指针）
 * @return uint8 PID_OK(0) 表示计算成功，PID_ERR_INV(1) 表示参数无效
 */
uint8 pid_compute(pid_ctrl_block_handle_t pid, float input_error,
                  float *ret_result);

/**
 * @brief 重置 PID 控制块的历史状态（积分、前次误差等清零）
 * @param pid 目标 PID 控制块句柄
 * @return uint8 PID_OK(0) 表示重置成功，PID_ERR_INV(1) 表示句柄无效
 */
uint8 pid_reset_ctrl_block(pid_ctrl_block_handle_t pid);

#endif