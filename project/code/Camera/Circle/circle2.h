/*******************************************************************************
 * circle2.h - 新版圆环处理模块
 *
 * 设计思路: 4状态 + 宽度模型 + 单侧补线 + 中线优先
 *   IDLE    → 未检测到圆环，每帧执行入口检测
 *   ENTRY   → 车辆正在进入圆环（断口出现，单侧边线异常）
 *   INSIDE  → 车辆在圆环内部（双侧均可能丢线或宽度突变）
 *   EXIT    → 车辆正在驶出圆环（断口重新出现，恢复正常赛道）
 *
 * 接口与 camera.c 完全兼容：
 *   - 使用 l_border / r_border / bin_image / points_l / points_r 等现有全局变量
 *   - 通过 circle2_process() 被 circle_track_process() 或 detect_track_type() 调用
 ******************************************************************************/
#ifndef _CIRCLE2_H_
#define _CIRCLE2_H_

#include "zf_common_headfile.h"

/*======================== 圆环状态枚举 ========================*/
typedef enum
{
    CIR2_IDLE   = 0,    // 空闲：未识别到圆环
    CIR2_ENTRY,         // 进入：看到圆环入口断口
    CIR2_INSIDE,        // 内部：行驶在圆环内
    CIR2_EXIT           // 驶出：看到圆环出口断口，恢复中
} Circle2State;

/*======================== 圆环方向枚举 ========================*/
typedef enum
{
    CIR2_NONE  = 0,     // 无圆环
    CIR2_LEFT,          // 左圆环（左侧出现断口，右侧正常）
    CIR2_RIGHT          // 右圆环（右侧出现断口，左侧正常）
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

/*======================== 配置参数（可在 .c 中修改） ========================*/
#define CIR2_BREAK_THRESH       8       // 边线跳变阈值
#define CIR2_INFLECT_RANGE      5       // 拐点搜索邻域
#define CIR2_LOST_THRESH        15      // 丢线阈值（认定为大面积丢线）
#define CIR2_WIDTH_NARROW       20      // 赛道过窄阈值
#define CIR2_WIDTH_WIDE         120     // 赛道过宽阈值
#define CIR2_TIMEOUT_MAX        80      // 超时帧数上限（防卡死）
#define CIR2_SCAN_TOP           5       // 扫描起始行（从上往下）
#define CIR2_SCAN_BOT           (MT9V03X_H - 5)    // 扫描结束行

/*======================== 函数声明 ========================*/

// 初始化/复位
void circle2_init(void);

// 主处理入口：每帧调用一次
// 返回值: 1 = 当前帧处于圆环状态（含 ENTRY/INSIDE/EXIT），0 = IDLE
uint8 circle2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);

// 特征提取（内部调用，也可外部调试用）
void circle2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                             Circle2Feature *feat);

// 获取当前状态（调试用）
Circle2State circle2_get_state(void);
Circle2Dir   circle2_get_dir(void);

#endif // _CIRCLE2_H_
