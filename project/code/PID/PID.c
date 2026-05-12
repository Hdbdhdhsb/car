#include "PID.h"

#define MAX_PID_BLOCKS  4          /* 最多同时存在的控制块数量 */

#define PID_MIN(a, b) ((a) < (b) ? (a) : (b))
#define PID_MAX(a, b) ((a) > (b) ? (a) : (b))
/* 函数指针类型 */
typedef float (*pid_cal_func_t)(struct pid_ctrl_block_t *pid, float error);
typedef unsigned char bool;
#define false 0
#define true 1
/* PID 控制块结构体 */
struct pid_ctrl_block_t {
    float Kp;
    float Ki;
    float Kd;
    float previous_err1;
    float previous_err2;
    float integral_err;
    float last_output;
    float max_output;
    float min_output;
    float max_integral;
    float min_integral;
    pid_cal_func_t calculate_func;
};

/* 静态内存池 */
static struct pid_ctrl_block_t pid_pool[MAX_PID_BLOCKS];
static bool pid_used[MAX_PID_BLOCKS] = { false };

/* 位置式 PID */
static float pid_calc_positional(struct pid_ctrl_block_t *pid, float error)
{
    float output;
    pid->integral_err += error;
    pid->integral_err = PID_MIN(pid->integral_err, pid->max_integral);
    pid->integral_err = PID_MAX(pid->integral_err, pid->min_integral);

    output = error * pid->Kp +
             (error - pid->previous_err1) * pid->Kd +
             pid->integral_err * pid->Ki;

    output = PID_MIN(output, pid->max_output);
    output = PID_MAX(output, pid->min_output);
    pid->previous_err1 = error;
    return output;
}

/* 增量式 PID */
static float pid_calc_incremental(struct pid_ctrl_block_t *pid, float error)
{
    float output;
    output = (error - pid->previous_err1) * pid->Kp +
             (error - 2.0f * pid->previous_err1 + pid->previous_err2) * pid->Kd +
             error * pid->Ki +
             pid->last_output;

    output = PID_MIN(output, pid->max_output);
    output = PID_MAX(output, pid->min_output);

    pid->previous_err2 = pid->previous_err1;
    pid->previous_err1 = error;
    pid->last_output = output;
    return output;
}

/* 创建控制块 */
uint8 pid_new_control_block(const pid_ctrl_parameter_t *init_param,
                            pid_ctrl_block_handle_t *ret_pid)
{
    uint8 i;
    if (init_param == NULL || ret_pid == NULL) return PID_ERR_INV;

    for (i = 0; i < MAX_PID_BLOCKS; i++) {
        if (pid_used[i] == false) {
            pid_used[i] = true;
            pid_pool[i].Kp           = init_param->kp;
            pid_pool[i].Ki           = init_param->ki;
            pid_pool[i].Kd           = init_param->kd;
            pid_pool[i].max_output   = init_param->max_output;
            pid_pool[i].min_output   = init_param->min_output;
            pid_pool[i].max_integral = init_param->max_integral;
            pid_pool[i].min_integral = init_param->min_integral;

            if (init_param->cal_type == PID_CAL_TYPE_INCREMENTAL)
                pid_pool[i].calculate_func = pid_calc_incremental;
            else
                pid_pool[i].calculate_func = pid_calc_positional;

            pid_pool[i].integral_err = 0.0f;
            pid_pool[i].previous_err1 = 0.0f;
            pid_pool[i].previous_err2 = 0.0f;
            pid_pool[i].last_output = 0.0f;

            *ret_pid = &pid_pool[i];
            return PID_OK;
        }
    }
    return PID_ERR_MEM;
}

/* 删除控制块 */
uint8 pid_del_control_block(pid_ctrl_block_handle_t pid)
{
    uint8 i;
    unsigned long addr_pid, addr_pool;   /* 用于整数比较，避免指针空间警告 */

    if (pid == NULL) return PID_ERR_INV;

    addr_pid = (unsigned long)pid;
    for (i = 0; i < MAX_PID_BLOCKS; i++) {
        addr_pool = (unsigned long)&pid_pool[i];
        if (addr_pid == addr_pool) {
            if (pid_used[i]) {
                pid_used[i] = false;
                return PID_OK;
            } else {
                return PID_ERR_INV;     /* 重复释放 */
            }
        }
    }
    return PID_ERR_INV;                 /* 无效句柄 */
}

/* 更新参数 */
uint8 pid_update_parameters(pid_ctrl_block_handle_t pid, const pid_ctrl_parameter_t *params)
{
    if (pid == NULL || params == NULL) return PID_ERR_INV;

    pid->Kp = params->kp;
    pid->Ki = params->ki;
    pid->Kd = params->kd;
    pid->max_output = params->max_output;
    pid->min_output = params->min_output;
    pid->max_integral = params->max_integral;
    pid->min_integral = params->min_integral;

    if (params->cal_type == PID_CAL_TYPE_INCREMENTAL)
        pid->calculate_func = pid_calc_incremental;
    else
        pid->calculate_func = pid_calc_positional;

    return PID_OK;
}

/* 计算输出 */
uint8 pid_compute(pid_ctrl_block_handle_t pid, float input_error, float *ret_result)
{
    if (pid == NULL || ret_result == NULL) return PID_ERR_INV;
    *ret_result = pid->calculate_func(pid, input_error);
    return PID_OK;
}

/* 重置历史状态 */
uint8 pid_reset_ctrl_block(pid_ctrl_block_handle_t pid)
{
    if (pid == NULL) return PID_ERR_INV;
    pid->integral_err = 0.0f;
    pid->previous_err1 = 0.0f;
    pid->previous_err2 = 0.0f;
    pid->last_output = 0.0f;
    return PID_OK;
}