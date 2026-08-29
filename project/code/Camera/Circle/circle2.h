#ifndef _CIRCLE2_H_
#define _CIRCLE2_H_

#include "zf_common_headfile.h"
#include "vision_config.h"

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

/* ===== 结构/派生常量(留在模块内)；其余 CIR2_* 调参已集中到 config.h ===== */
#define CIR2_SCAN_TOP               5         //扫描顶部
#define CIR2_SCAN_BOT               (MT9V03X_H - 5) //扫描底部
#define CIR2_REPAIR_VALID           2         //补线点标记
#define CIR2_EXIT_ANCHOR_X          0         //出口锚点 X
#define CIR2_EXIT_ANCHOR_Y          CIR2_SCAN_TOP //出口锚点 Y

void circle2_init(void);
uint8 circle2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);
void circle2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                             Circle2Feature *feat);
Circle2State circle2_get_state(void);
Circle2Dir   circle2_get_dir(void);

#endif
