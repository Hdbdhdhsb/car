/*******************************************************************************
 * cross2.h - 新版十字处理模块
 *
 * 设计思路: 3状态 + 宽度模型 + 双侧断点补线 + 方向生长验证
 *   IDLE      → 未检测到十字，每帧在 detect_track_type 中被调用
 *   CROSSING  → 正在通过十字区域（双侧断口+宽度突变）
 *   RECOVER   → 离开十字，边线恢复正常中
 *
 * 十字 vs 圆环 的核心区别:
 *   - 十字: 双侧同时出现断口，宽度突然变大
 *   - 圆环: 单侧出现断口，对侧正常
 *   因此十字检测要求：左右断点同时存在 或 双侧同时大面积丢线
 *
 * 接口与 camera.c 完全兼容:
 *   - 使用 l_border / r_border / bin_image / dir_l / dir_r / points_l / points_r 等全局变量
 *   - 通过 cross2_process() 被 detect_track_type() 或 SPECIAL_MODE 调用
 ******************************************************************************/
#ifndef _CROSS2_H_
#define _CROSS2_H_

#include "zf_common_headfile.h"
#include "vision_config.h"

/*======================== 十字状态枚举 ========================*/
typedef enum
{
    CROSS2_IDLE     = 0,    // 空闲：未识别到十字
    CROSS2_CROSSING,        // 通过中：双侧断口，正在补线
    CROSS2_RECOVER          // 恢复中：断口消失，等待边线完全正常
} Cross2State;

/*======================== 特征信息结构体 ========================*/
typedef struct
{
    // --- 边线断口（边线值跳变） ---
    uint8  l_break_row;         // 左下边线断口行号（0 = 未找到）
    uint8  r_break_row;         // 右下边线断口行号
    uint8  l_break_row2;       // 左上断口行号
    uint8  r_break_row2;       // 右上断口行号
    int16  l_break_delta;       // 左断口跳变量
    int16  r_break_delta;       // 右断口跳变量

    // --- 方向链码断点（八邻域生长方向突变） ---
    uint8  l_dir_break_row;     // 左侧方向链码断点行号
    uint8  r_dir_break_row;     // 右侧方向链码断点行号

    // --- 宽度模型 ---
    uint16 width_sum;           // 扫描区域赛道宽度累加和
    uint8  width_max;           // 最大宽度
    uint8  width_min;           // 最小宽度
    uint8  width_bottom;        // 底部宽度
    uint8  width_mid;           // 中部宽度
    uint8  wide_cnt;            // 赛道过宽的行数

    // --- 边线丢失统计 ---
    uint8  l_lost_cnt;          // 左边线丢失行数（贴 BORDER_MIN）
    uint8  r_lost_cnt;          // 右边线丢失行数（贴 BORDER_MAX）
    uint8  both_lost_cnt;       // 双侧同时丢失行数

    // --- 角点像素 ---
    uint8  corner_bl;           // 左下角
    uint8  corner_br;           // 右下角

    // --- 斜率（用于补线） ---
    float  l_slope;             // 断点上方左边线斜率
    float  l_intercept;         // 断点上方左边线截距
    float  r_slope;             // 断点上方右边线斜率
    float  r_intercept;         // 断点上方右边线截距
} Cross2Feature;

/*======================== 全局变量声明 ========================*/
extern Cross2State  cross2_state;       // 当前十字状态
extern uint8        cross2_timeout;     // 超时计数器（帧数）

/*======================== 配置参数 ========================*/
#define CROSS2_DIR_UP           4       // 八邻域方向：向上
#define CROSS2_DIR_DOWN         0       // 八邻域方向：向下
#define CROSS2_DIR_BREAK_UP     2       // 链码断点模式中的"向上偏"方向
/* ===== 结构/派生常量(留在模块内)；其余 CROSS2_* 调参已集中到 config.h ===== */
#define CROSS2_SCAN_TOP         5       // 扫描起始行
#define CROSS2_SCAN_BOT         (MT9V03X_H - 5)    // 扫描结束行
//1：双侧断口同时存在
//2：双侧方向断点同时存在
//3：过宽行数足够多且底角为白
/*======================== 函数声明 ========================*/

// 初始化/复位
void cross2_init(void);

// 主处理入口：每帧调用一次
// 参数: l=l_border, r=r_border, img=bin_image
// 返回: 1 = 正在十字处理中 (CROSSING/RECOVER), 0 = IDLE
uint8 cross2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);

// 特征提取（内部使用，也可供调试查看）
uint8 cross2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                            Cross2Feature *feat);

// 调试查询
Cross2State cross2_get_state(void);

#endif // _CROSS2_H_
