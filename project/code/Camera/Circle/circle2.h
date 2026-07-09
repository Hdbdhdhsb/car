#ifndef _CIRCLE2_H_
#define _CIRCLE2_H_

#include "zf_common_headfile.h"

typedef enum
{
    CIR2_IDLE = 0,
    CIR2_APPROACH,
    CIR2_APPROACH_END,
    CIR2_ENTRY,
    CIR2_INSIDE,
    CIR2_EXIT,
    CIR2_EXITING,
    CIR2_EXIT_END
} Circle2State;

typedef enum
{
    CIR2_NONE = 0,
    CIR2_LEFT,
    CIR2_RIGHT
} Circle2Dir;

typedef struct
{
    /* 边线断口特征 */
    uint8  l_break_row;
    uint8  r_break_row;
    int16  l_break_delta;
    int16  r_break_delta;

    /* 边线拐点特征 */
    uint8  l_inflect_row;
    uint8  r_inflect_row;
    uint8  l_rebound_trend;
    uint8  r_rebound_trend;

    /* 宽度模型 */
    uint8  width_max;
    uint8  width_min;
    uint8  width_bottom;
    int16  width_diff;

    /* 丢线统计 */
    uint8  l_lost_cnt;
    uint8  r_lost_cnt;

    /* 角点像素 */
    uint8  corner_bl;
    uint8  corner_br;
    uint8  corner_tl;
    uint8  corner_tr;
} Circle2Feature;

extern Circle2State cir2_state;
extern Circle2Dir   cir2_dir;
extern uint8        cir2_timeout;
extern uint8        cir2_entry_cnt;           //入口数

#define CIR2_BREAK_THRESH           8         //断口判定阈值
#define CIR2_INFLECT_RANGE          5         //拐点范围
#define CIR2_LOST_THRESH            10        //丢线阈值
#define CIR2_WIDTH_NARROW           20        //最小赛道宽度
#define CIR2_WIDTH_WIDE             120       //最大赛道宽度
#define CIR2_TIMEOUT_MAX            80        //环岛处理超时
#define CIR2_APPROACH_TIMEOUT       25        //approach阶段超时
#define CIR2_SCAN_TOP               5         //扫描顶部
#define CIR2_SCAN_BOT               (MT9V03X_H - 5) //扫描底部
#define CIR2_BREAK_MIN_ROW          10        //断口行需 > 30
#define CIR2_FIT_LEN                10        //拟合长度
#define CIR2_APPROACH_REPAIR_LEN    20        //approach阶段补线长度
#define CIR2_INFLECT_TREND_THRESH   6         //拐点趋势阈值
#define CIR2_REBOUND_TREND_THRESH   4         //突跳后回摆趋势阈值
#define CIR2_BREAK_DELTA_MARGIN     4         //双断口主次差阈值
#define CIR2_WIDTH_DIFF_MIN         6         //入口宽度变化阈值
#define CIR2_ENTRY_SCORE_MIN        5         //入口判定最小得分
#define CIR2_REPAIR_VALID           2         //补线点标记
#define CIR2_ENTRY_HOLD_FRAMES      3         //入口锚点保持帧数
#define CIR2_ENTRY_PULL_MIN         4         //入口补线最小内拉
#define CIR2_ENTRY_PULL_MAX         18        //入口补线最大内拉
#define CIR2_MODEL_WIDTH_MIN        24        //宽度模型最小宽度
#define CIR2_MODEL_WIDTH_MAX        70        //宽度模型最大宽度
#define CIR2_ENTRY_RATIO_NUM        2         //ENTRY 单侧中线宽度系数分子
#define CIR2_ENTRY_RATIO_DEN        5         //ENTRY 单侧中线宽度系数分母
#define CIR2_INSIDE_RATIO_NUM       1         //INSIDE 单侧中线宽度系数分子
#define CIR2_INSIDE_RATIO_DEN       2         //INSIDE 单侧中线宽度系数分母
#define CIR2_EXIT_RATIO_NUM         9         //EXIT 单侧中线宽度系数分子
#define CIR2_EXIT_RATIO_DEN         20        //EXIT 单侧中线宽度系数分母
#define CIR2_ENTRY_ERR_ROW_A        24        //入口前瞻采样行 A
#define CIR2_ENTRY_ERR_ROW_B        16        //入口前瞻采样行 B

#define CIR2_EXIT_CHECK_FRAMES      2
#define CIR2_EXIT_ANCHOR_X          0
#define CIR2_EXIT_ANCHOR_Y          CIR2_SCAN_TOP
#define CIR2_EXIT_MIN_FRAMES        1
#define CIR2_EXITING_MIN_FRAMES     1
#define CIR2_EXIT_END_LEFT_LOST     28
#define CIR2_EXIT_END_RIGHT_OK      12
#define CIR2_EXIT_RECOVER_LOST      5
#define CIR2_EXIT_WIDTH_TOP         18
#define CIR2_EXIT_WIDTH_BOTTOM      44

void circle2_init(void);
uint8 circle2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);
void circle2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                             Circle2Feature *feat);
Circle2State circle2_get_state(void);
Circle2Dir   circle2_get_dir(void);

#endif
