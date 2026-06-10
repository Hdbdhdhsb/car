#ifndef _CIRCLE2_H_
#define _CIRCLE2_H_

#include "zf_common_headfile.h"

typedef enum
{
    CIR2_IDLE = 0,
    CIR2_APPROACH,
    CIR2_ENTRY,
    CIR2_INSIDE,
    CIR2_EXIT
} Circle2State;

typedef enum
{
    CIR2_NONE = 0,
    CIR2_LEFT,
    CIR2_RIGHT
} Circle2Dir;

/*======================== 特征信息结构体 ========================*/
typedef struct
{
    // --- 边线断口特征 ---
    uint8  l_break_row;         // 左边线断口行号（0 = 未找到）
    uint8  r_break_row;         // 右边线断口行号
    int16  l_break_delta;       // 左断口处左边界跳变量
    int16  r_break_delta;       // 右断口处右边界跳变量

    // --- 拐点特征 ---
    uint8  l_inflect_row;       // 左边线上拐点行号
    uint8  r_inflect_row;       // 右边线上拐点行号

    // --- 宽度模型 ---
    uint8  width_max;           // 最大赛道宽度
    uint8  width_min;           // 最小赛道宽度
    uint8  width_bottom;        // 图像底部赛道宽度
    int16  width_diff;          // 宽度变化率（上 - 下）

    // --- 边线丢失统计 ---
    uint8  l_lost_cnt;          // 左边线丢失行数
    uint8  r_lost_cnt;          // 右边线丢失行数

    // --- 角点像素检测 ---
    uint8  corner_bl;           // 左下角像素 (bin_image[H-2][2])
    uint8  corner_br;           // 右下角像素 (bin_image[H-2][W-3])
    uint8  corner_tl;           // 左上角近底像素
    uint8  corner_tr;           // 右上角近底像素
} Circle2Feature;

/*======================== 全局变量声明 ========================*/
extern Circle2State  cir2_state;        // 当前圆环状态
extern Circle2Dir    cir2_dir;          // 当前圆环方向
extern uint8         cir2_timeout;      // 超时计数器（帧数）
extern uint8         cir2_entry_cnt;    // 入口计数：第1次入口观察 / 第2次确认入环

/*======================== 配置参数（可在 .c 中修改） ========================*/
#define CIR2_BREAK_THRESH       8       // 边线跳变阈值
#define CIR2_INFLECT_RANGE      5       // 拐点搜索邻域
#define CIR2_LOST_THRESH        15      // 丢线阈值（认定为大面积丢线）
#define CIR2_WIDTH_NARROW       20      // 赛道过窄阈值
#define CIR2_WIDTH_WIDE         120     // 赛道过宽阈值
#define CIR2_TIMEOUT_MAX        80      // 超时帧数上限（防卡死）
#define CIR2_APPROACH_TIMEOUT   25      // 第一入口到第二入口的最大等待帧数
#define CIR2_SCAN_TOP           5       // 扫描起始行（从上往下）
#define CIR2_SCAN_BOT           (MT9V03X_H - 5)    // 扫描结束行
#define CIR2_BREAK_MIN_ROW        30
#define CIR2_FIT_LEN              10
#define CIR2_APPROACH_REPAIR_LEN  15

void circle2_init(void);
uint8 circle2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);
void circle2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                             Circle2Feature *feat);
Circle2State circle2_get_state(void);
Circle2Dir circle2_get_dir(void);


#endif // _CIRCLE2_H_
