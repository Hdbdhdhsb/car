#include "circle2.h"
#include "camera.h"
/*******************************************************************************
 * circle2.c - 新版圆环处理模块
 *
 * 4状态机:  IDLE → ENTRY → INSIDE → EXIT → IDLE
 *
 * 核心思路:
 *   1. 特征提取: 每帧扫描一遍边线，提取断口、拐点、宽度模型、丢线统计
 *   2. 状态判别: 用提取到的特征在 4 个状态间切换
 *   3. 单侧补线: 根据圆环方向，只对断口侧边线做线性插值修复
 *   4. 中线优先: 修复后立即重算中线
 *
 * 与 camera.c 兼容:
 *   - 直接操作 l_border / r_border / midline 数组（uint16 类型）
 *   - 设置 current_track_type / current_state 来驱动外层状态机
 ******************************************************************************/
#define false 0
#define true 1

/*======================== 全局变量定义 ========================*/
Circle2State  cir2_state   = CIR2_IDLE;
Circle2Dir    cir2_dir     = CIR2_NONE;
uint8         cir2_timeout = 0;
uint8         cir2_entry_cnt = 0;
static uint8  cir2_cached_anchor_top = 0;//上描点
static uint8  cir2_cached_anchor_bot = 0;//下描点
static uint8  cir2_entry_hold_cnt = 0;//入口计数

/*======================== 内部辅助函数声明 ========================*/
static void   _extract_break(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _extract_inflect(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _extract_width(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _extract_lost(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W], Circle2Feature *f);
static void   _extract_corner(uint8 (*img)[MT9V03X_W], Circle2Feature *f);
static uint8  _check_left_inflect_trend(uint16 *l, uint8 row);
static uint8  _check_right_inflect_trend(uint16 *r, uint8 row);
static uint8  _check_left_rebound_trend(uint16 *l, Circle2Feature *f);
static uint8  _check_right_rebound_trend(uint16 *r, Circle2Feature *f);

static uint8  _check_entry(Circle2Feature *f);
static uint8 _check_entry2(Circle2Feature *f);
static uint8  _check_inside(Circle2Feature *f);
static uint8  _check_exit(Circle2Feature *f);

static void   _calc_slope(uint16 *border, uint8 break_row, float *slope, float *intercept);
static void   _set_repaired_point(uint16 *border, uint8 *valid, uint8 row, int16 val);
static uint8  _calc_entry_pull(uint8 anchor_top, uint8 anchor_bot, uint8 row);
static void   _repair_approach(uint16 *l, uint16 *r, Circle2Feature *f);
static uint8  _repair_approach_end(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _repair_left(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _repair_right(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _recalc_midline(uint16 *l, uint16 *r);
static uint8  _get_model_width(Circle2Feature *f);
static uint8  _get_model_ratio_num(void);
static uint8  _get_model_ratio_den(void);
static void   _recalc_circle_midline(uint16 *l, uint16 *r, Circle2Feature *f);

/*======================== 辅助宏 ========================*/
#define ABS_DIFF(a, b)  ((int16)(a) > (int16)(b) ? (int16)(a) - (int16)(b) : (int16)(b) - (int16)(a))
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/*******************************************************************************
 * circle2_init - 复位圆环状态
 ******************************************************************************/
void circle2_init(void)
{
    cir2_state   = CIR2_IDLE;
    cir2_dir     = CIR2_NONE;
    cir2_timeout = 0;
    cir2_entry_cnt = 0;
    cir2_cached_anchor_top = 0;
    cir2_cached_anchor_bot = 0;
    cir2_entry_hold_cnt = 0;
}

/*******************************************************************************
 * circle2_extract_feature - 提取本帧所有圆环相关特征
 ******************************************************************************/
void circle2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                             Circle2Feature *f)
{
    /* 清零 */
    f->l_break_row    = 0;
    f->r_break_row    = 0;
    f->l_break_delta  = 0;
    f->r_break_delta  = 0;
    f->l_inflect_row  = 0;
    f->r_inflect_row  = 0;
    f->l_rebound_trend = 0;
    f->r_rebound_trend = 0;
    f->width_max      = 0;
    f->width_min      = 255;
    f->width_bottom   = 0;
    f->width_diff     = 0;
    f->l_lost_cnt     = 0;
    f->r_lost_cnt     = 0;
    f->corner_bl      = 0;
    f->corner_br      = 0;
    f->corner_tl      = 0;
    f->corner_tr      = 0;

    _extract_break(l, r, f);
    f->l_rebound_trend = _check_left_rebound_trend(l, f);
    f->r_rebound_trend = _check_right_rebound_trend(r, f);
    _extract_inflect(l, r, f);
    _extract_width(l, r, f);
    _extract_lost(l, r, img, f);
    _extract_corner(img, f);
}

/*******************************************************************************
 * circle2_process - 每帧调用一次的主入口
 *
 * 参数:
 *   l   - l_border 数组指针
 *   r   - r_border 数组指针
 *   img - bin_image 二值图
 *
 * 返回:
 *   1 = 当前帧处于圆环处理中 (ENTRY/INSIDE/EXIT)
 *   0 = IDLE
 ******************************************************************************/
uint8 circle2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W])
{
    Circle2Feature feat;
    Circle2Dir cand_dir = CIR2_NONE;

    /* 1. 特征提取 */
    circle2_extract_feature(l, r, img, &feat);

    if (feat.l_break_row && !feat.r_break_row)
        cand_dir = CIR2_LEFT;
    else if (feat.r_break_row && !feat.l_break_row)
        cand_dir = CIR2_RIGHT;
    else if (feat.l_break_delta > feat.r_break_delta)
        cand_dir = CIR2_LEFT;
    else if (feat.r_break_delta > 0)
        cand_dir = CIR2_RIGHT;

    /* 2. 状态机 */
    switch (cir2_state)
    {
    /*---------- IDLE: 寻找圆环入口 ----------*/
    case CIR2_IDLE:
    {
        if (_check_entry(&feat))
        {
            cir2_state   = CIR2_APPROACH;
            cir2_timeout = 0;
            cir2_entry_cnt = 1;
            cir2_dir = cand_dir;
        }
        break;
    }

    /*---------- APPROACH: 第一入口直行补线 ----------*/
    case CIR2_APPROACH:
    {
        /* 第一个入口仍在画面中：只做直行保护补线，不进入圆环 */
        if (cir2_entry_cnt == 1)
        {
            if (_check_entry(&feat) && cand_dir == cir2_dir)
            {
                _repair_approach(l, r, &feat);/*直线补线*/
                _recalc_midline(l, r);
                cir2_timeout++;
                if (cir2_timeout > CIR2_APPROACH_TIMEOUT)
                {
                    circle2_init();
                    return 0;
                }
                return 1;
            }

            /* 第一个入口特征消失，补线等待第二入口 */
            if (((cir2_dir == CIR2_LEFT &&
                  (feat.l_lost_cnt >= CIR2_LOST_THRESH ||
                   feat.l_break_row > 0 ||
                   feat.width_diff >= CIR2_WIDTH_DIFF_MIN)) ||
                 (cir2_dir == CIR2_RIGHT &&
                  (feat.r_lost_cnt >= CIR2_LOST_THRESH ||
                   feat.r_break_row > 0 ||
                   feat.width_diff >= CIR2_WIDTH_DIFF_MIN))) &&
                _repair_approach_end(l, r, &feat))/*找点连线补线*/
            {
                _recalc_midline(l, r);
                cir2_timeout++;
                if (cir2_timeout > CIR2_APPROACH_TIMEOUT)
                {
                    circle2_init();
                    return 0;
                }
                return 1;
            }

            cir2_timeout = 0;
            return 1;
        }

        /* 第二次看到同方向入口，才真正按圆环处理 */
        if (_check_entry2(&feat) && cand_dir == cir2_dir)
        {
            cir2_state     = CIR2_ENTRY;
            cir2_timeout   = 0;

            current_track_type = CIRCLE_TRACK;
            current_state      = SPECIAL_MODE;

            _recalc_circle_midline(l, r, &feat);/*宽度模型算中线*/
            return 1;
        }

        cir2_timeout++;
        if (cir2_timeout > CIR2_APPROACH_TIMEOUT)
        {
            circle2_init();
            return 0;
        }
        return 1;
    }

    /*---------- ENTRY: 正在进入圆环 ----------*/
    case CIR2_ENTRY:
    {
        _recalc_circle_midline(l, r, &feat);/*宽度模型算中线*/

        /* 判断是否已完全进入圆环内部 */
        if (_check_inside(&feat))
        {
            cir2_state = CIR2_INSIDE;
        }

        cir2_timeout++;
        if (cir2_timeout > CIR2_TIMEOUT_MAX)
        {
            circle2_init();
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
        }
        break;
    }

    /*---------- INSIDE: 行驶在圆环内部 ----------*/
    case CIR2_INSIDE:
    {
        _recalc_circle_midline(l, r, &feat);

        /* 判断是否进入出口阶段 */
        if (_check_exit(&feat))
        {
            cir2_state = CIR2_EXIT;
        }
        cir2_timeout++;
        if (cir2_timeout > CIR2_TIMEOUT_MAX)
        {
            circle2_init();
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
        }
        break;
    }

    /*---------- EXIT: 正在驶出圆环 ----------*/
    case CIR2_EXIT:
    {
        uint8 recovered = 0;
        _recalc_circle_midline(l, r, &feat);

        /* 判断是否已完全恢复正常赛道 */
        if (feat.l_lost_cnt < 5 && feat.r_lost_cnt < 5 &&
            !feat.l_break_row && !feat.r_break_row)
        {
            recovered = 1;
        }

        if (recovered)
        {
            circle2_init();
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
            return 0;
        }
        cir2_timeout++;
        if (cir2_timeout > CIR2_TIMEOUT_MAX)
        {
            circle2_init();
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
        }
        break;
    }

    default:
        circle2_init();
        break;
    }

    return (cir2_state == CIR2_APPROACH ||
            cir2_state == CIR2_ENTRY ||
            cir2_state == CIR2_INSIDE ||
            cir2_state == CIR2_EXIT) ? 1 : 0;
}

/*******************************************************************************
 * circle2_get_state / circle2_get_dir - 调试查询
 ******************************************************************************/
Circle2State circle2_get_state(void) { return cir2_state; }
Circle2Dir   circle2_get_dir(void)   { return cir2_dir;   }

/*============================================================================
 *                         内部: 特征提取函数
 *============================================================================*/

/*--------------------------------------------------------------------------
 * _extract_break - 搜索左/右边线断口
 *
 * 断口定义: 相邻行边线值跳变超过 CIR2_BREAK_THRESH
 * 从底部向上扫描，找到第一个显著断口即记录
 *--------------------------------------------------------------------------*/
static void _extract_break(uint16 *l, uint16 *r, Circle2Feature *f)
{
    uint8 row;

    /* 左边线断口: 向上扫描，l[row] 突然变小（向左跳） */
    for (row = CIR2_SCAN_BOT; row > CIR2_SCAN_TOP + 1; row--)
    {
        int16 delta = (int16)l[row] - (int16)l[row - 1];
        if (delta > CIR2_BREAK_THRESH && row > CIR2_BREAK_MIN_ROW)
        {
            f->l_break_row   = row;
            f->l_break_delta = delta;
            break;
        }
    }

    /* 右边线断口: 向上扫描，r[row] 突然变大（向右跳） */
    for (row = CIR2_SCAN_BOT; row > CIR2_SCAN_TOP + 1; row--)
    {
        int16 delta = (int16)r[row - 1] - (int16)r[row];
        if (delta > CIR2_BREAK_THRESH && row > CIR2_BREAK_MIN_ROW)
        {
            f->r_break_row   = row;
            f->r_break_delta = delta;
            break;
        }
    }
}

static uint8 _check_left_inflect_trend(uint16 *l, uint8 row)
{
    uint8 i;
    int16 rise_sum = 0;
    int16 fall_sum = 0;

    for (i = 1; i <= CIR2_INFLECT_RANGE; i++)
    {
        if (l[row - i] <= BORDER_MIN + 1 || l[row + i] <= BORDER_MIN + 1)
        {
            return 0;
        }

        rise_sum += (int16)l[row] - (int16)l[row - i];
        fall_sum += (int16)l[row + i] - (int16)l[row];
    }

    return (rise_sum >= CIR2_INFLECT_TREND_THRESH &&
            fall_sum <= -CIR2_INFLECT_TREND_THRESH) ? 1 : 0;
}

static uint8 _check_right_inflect_trend(uint16 *r, uint8 row)
{
    uint8 i;
    int16 rise_sum = 0;
    int16 fall_sum = 0;

    for (i = 1; i <= CIR2_INFLECT_RANGE; i++)
    {
        if (r[row - i] >= BORDER_MAX - 1 || r[row + i] >= BORDER_MAX - 1)
        {
            return 0;
        }

        rise_sum += (int16)r[row] - (int16)r[row - i];
        fall_sum += (int16)r[row + i] - (int16)r[row];
    }

    return (rise_sum <= -CIR2_INFLECT_TREND_THRESH &&
            fall_sum >= CIR2_INFLECT_TREND_THRESH) ? 1 : 0;
}

static uint8 _check_left_rebound_trend(uint16 *l, Circle2Feature *f)
{
    uint8 row = f->l_break_row;
    int16 break_delta;
    int16 rebound_delta;

    if (row <= CIR2_BREAK_MIN_ROW || row + CIR2_INFLECT_RANGE >= CIR2_SCAN_BOT)
    {
        return 0;
    }

    if (l[row] <= BORDER_MIN + 1 || l[row - 1] <= BORDER_MIN + 1 ||
        l[row + CIR2_INFLECT_RANGE] <= BORDER_MIN + 1)
    {
        return 0;
    }

    break_delta = (int16)l[row] - (int16)l[row - 1];
    rebound_delta = (int16)l[row] - (int16)l[row + CIR2_INFLECT_RANGE];

    return (break_delta > CIR2_BREAK_THRESH &&
            rebound_delta >= CIR2_REBOUND_TREND_THRESH) ? 1 : 0;
}

static uint8 _check_right_rebound_trend(uint16 *r, Circle2Feature *f)
{
    uint8 row = f->r_break_row;
    int16 break_delta;
    int16 rebound_delta;

    if (row <= CIR2_BREAK_MIN_ROW || row + CIR2_INFLECT_RANGE >= CIR2_SCAN_BOT)
    {
        return 0;
    }

    if (r[row] >= BORDER_MAX - 1 || r[row - 1] >= BORDER_MAX - 1 ||
        r[row + CIR2_INFLECT_RANGE] >= BORDER_MAX - 1)
    {
        return 0;
    }

    break_delta = (int16)r[row - 1] - (int16)r[row];
    rebound_delta = (int16)r[row + CIR2_INFLECT_RANGE] - (int16)r[row];

    return (break_delta > CIR2_BREAK_THRESH &&
            rebound_delta >= CIR2_REBOUND_TREND_THRESH) ? 1 : 0;
}

/*--------------------------------------------------------------------------
 * _extract_inflect - 搜索单侧边线拐点
 *
 * 拐点定义: 边线趋势先向一侧发展，再向另一侧回摆
 * 在断口上方区域搜索，用于确认圆环弧形边线
 *--------------------------------------------------------------------------*/
static void _extract_inflect(uint16 *l, uint16 *r, Circle2Feature *f)
{
    uint8 row;
    uint8 search_end;

    /* 左边线拐点: 在断口上方寻找“先增后减”的趋势反转 */
    search_end = (f->l_break_row > CIR2_INFLECT_RANGE + CIR2_SCAN_TOP)
                     ? (f->l_break_row - 15)
                     : CIR2_SCAN_TOP;
    if (search_end < CIR2_SCAN_TOP + CIR2_INFLECT_RANGE)
        search_end = CIR2_SCAN_TOP + CIR2_INFLECT_RANGE;

    for (row = search_end; row > CIR2_SCAN_TOP + CIR2_INFLECT_RANGE; row--)
    {
        if (_check_left_inflect_trend(l, row))
        {
            f->l_inflect_row = row;
            break;
        }
    }

    /* 右边线拐点: 在断口上方寻找“先减后增”的趋势反转 */
    search_end = (f->r_break_row > CIR2_INFLECT_RANGE + CIR2_SCAN_TOP)
                     ? (f->r_break_row - 15)
                     : CIR2_SCAN_TOP;
    if (search_end < CIR2_SCAN_TOP + CIR2_INFLECT_RANGE)
        search_end = CIR2_SCAN_TOP + CIR2_INFLECT_RANGE;

    for (row = search_end; row > CIR2_SCAN_TOP + CIR2_INFLECT_RANGE; row--)
    {
        if (_check_right_inflect_trend(r, row))
        {
            f->r_inflect_row = row;
            break;
        }
    }
}

/*--------------------------------------------------------------------------
 * _extract_width - 赛道宽度模型
 *--------------------------------------------------------------------------*/
static void _extract_width(uint16 *l, uint16 *r, Circle2Feature *f)
{
    uint8  row;
    uint8  w;
    uint16 w_top = 0;

    f->width_bottom = (uint8)(r[CIR2_SCAN_BOT] - l[CIR2_SCAN_BOT]);

    for (row = CIR2_SCAN_TOP; row <= CIR2_SCAN_BOT; row++)
    {
        if (r[row] > l[row])
            w = (uint8)(r[row] - l[row]);
        else
            w = 0;

        if (w > f->width_max) f->width_max = w;
        if (w < f->width_min) f->width_min = w;

        if (row == CIR2_SCAN_TOP + 10)
            w_top = w;
    }

    f->width_diff = (int16)w_top - (int16)f->width_bottom;
}

/*--------------------------------------------------------------------------
 * _extract_lost - 边线丢失统计
 *--------------------------------------------------------------------------*/
static void _extract_lost(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                           Circle2Feature *f)
{
    uint8 row;

    for (row = CIR2_SCAN_TOP; row <= CIR2_SCAN_BOT; row++)
    {
        if (l[row] <= BORDER_MIN + 1) f->l_lost_cnt++;
        if (r[row] >= BORDER_MAX - 1) f->r_lost_cnt++;
    }
}

/*--------------------------------------------------------------------------
 * _extract_corner - 四角像素采样
 *--------------------------------------------------------------------------*/
static void _extract_corner(uint8 (*img)[MT9V03X_W], Circle2Feature *f)
{
    f->corner_bl = img[MT9V03X_H - 2][2];
    f->corner_br = img[MT9V03X_H - 2][MT9V03X_W - 3];
    f->corner_tl = img[MT9V03X_H / 2][2];
    f->corner_tr = img[MT9V03X_H / 2][MT9V03X_W - 3];
}

static void _calc_slope(uint16 *border, uint8 break_row, float *slope, float *intercept)
{
    int16 i;
    uint8 start_row = break_row + CIR2_FIT_LEN;
    uint8 end_row   = break_row + 2;
    float sum_x = 0;
    float sum_y = 0;
    float sum_xy = 0;
    float sum_x2 = 0;
    float n = 0;
    float denom;

    if (start_row > CIR2_SCAN_BOT)
        start_row = CIR2_SCAN_BOT;
    if (end_row > CIR2_SCAN_BOT || end_row >= start_row)
    {
        *slope = 0;
        *intercept = (float)border[break_row];
        return;
    }

    for (i = start_row; i >= end_row; i--)
    {
        float x = (float)i;
        float y = (float)border[i];
        sum_x  += x;
        sum_y  += y;
        sum_xy += x * y;
        sum_x2 += x * x;
        n      += 1.0f;
    }

    denom = n * sum_x2 - sum_x * sum_x;
    if (denom > 0.001f || denom < -0.001f)
    {
        *slope     = (n * sum_xy - sum_x * sum_y) / denom;
        *intercept = (sum_y - (*slope) * sum_x) / n;
    }
    else
    {
        *slope = 0;
        *intercept = (float)border[break_row];
    }
}

/*
 * Mark repaired rows as valid at the same time we overwrite the border.
 * Without this, downstream midline code may ignore the repaired segment.
 */
static void _set_repaired_point(uint16 *border, uint8 *valid, uint8 row, int16 val)
{
    border[row] = (uint16)CLAMP(val, BORDER_MIN, BORDER_MAX);
    valid[row] = CIR2_REPAIR_VALID;
}

/*
 * During ENTRY, pull the repaired edge farther inward near the upper anchor.
 * This creates a stronger steering hint than a plain straight bridge.
 */
static uint8 _calc_entry_pull(uint8 anchor_top, uint8 anchor_bot, uint8 row)
{
    uint8 span = anchor_bot - anchor_top;
    uint8 progress = anchor_bot - row;
    uint16 offset_range = (uint16)(CIR2_ENTRY_PULL_MAX - CIR2_ENTRY_PULL_MIN);

    if(anchor_bot <= anchor_top)
    {
        return CIR2_ENTRY_PULL_MIN;
    }

    return (uint8)(CIR2_ENTRY_PULL_MIN +
        (uint8)((offset_range * progress + (span >> 1)) / span));
}

static void _repair_approach(uint16 *l, uint16 *r, Circle2Feature *f)
{
    int16 row;
    uint8 break_row = 0;
    uint16 *border = 0;
    uint8 *valid = 0;
    float k = 0;
    float b = 0;
    int16 stop_row;

    if (cir2_dir == CIR2_LEFT)
    {
        break_row = f->l_break_row;
        border = l;
        valid = l_border_valid;
    }
    else if (cir2_dir == CIR2_RIGHT)
    {
        break_row = f->r_break_row;
        border = r;
        valid = r_border_valid;
    }
    else
    {
        return;
    }

    if (break_row <= CIR2_SCAN_TOP + 1)
        return;

    _calc_slope(border, break_row, &k, &b);

    stop_row = (int16)break_row - CIR2_APPROACH_REPAIR_LEN;
    if (stop_row < CIR2_SCAN_TOP)
        stop_row = CIR2_SCAN_TOP;

    for (row = (int16)break_row; row >= stop_row; row--)
    {
        int16 val = (int16)(k * row + b);
        _set_repaired_point(border, valid, (uint8)row, val);
    }
}

/*============================================================================
 *                         内部: 状态判断函数
 *============================================================================*/

/*------------------------------------------------------------------
 * _check_entry - IDLE→ENTRY 条件
 *-----------------------------------------------------------------*/
static uint8 _check_entry(Circle2Feature *f)
{
    uint8 left_score = 0;
    uint8 right_score = 0;

    uint8 left_asym = 0;//主左侧断口强度
    uint8 right_asym = 0;//主右侧断口强度
    uint8 width_open = 0;//宽度过大
    uint8 left_break = 0;
    uint8 right_break = 0;

    if (f->width_diff >= CIR2_WIDTH_DIFF_MIN)
    {
        width_open = 1;
    }

    if (f->r_break_row == 0 ||
        f->l_break_delta >= f->r_break_delta + CIR2_BREAK_DELTA_MARGIN)
    {
        left_asym = 1;
    }

    if (f->l_break_row == 0 ||
        f->r_break_delta >= f->l_break_delta + CIR2_BREAK_DELTA_MARGIN)
    {
        right_asym = 1;
    }

    left_break = (f->l_break_row > CIR2_BREAK_MIN_ROW) ? 1 : 0;
    right_break = (f->r_break_row > CIR2_BREAK_MIN_ROW) ? 1 : 0;
    //判定圆环打分
    if (left_break)                          left_score += 2;
    if (f->l_inflect_row > 0)                left_score += 2;
    if (left_asym)                           left_score += 1;
    if (width_open)                          left_score += 1;
    if (left_break && f->l_lost_cnt >= CIR2_LOST_THRESH) left_score += 1;
    if (f->l_rebound_trend)                              left_score += 1;

    if (right_break)                         right_score += 2;
    if (f->r_inflect_row > 0)                right_score += 2;
    if (right_asym)                          right_score += 1;
    if (width_open)                          right_score += 1;
    if (right_break && f->r_lost_cnt >= CIR2_LOST_THRESH) right_score += 1;
    if (f->r_rebound_trend)                              right_score += 1;

    if (left_score >= CIR2_ENTRY_SCORE_MIN &&
        right_score >= CIR2_ENTRY_SCORE_MIN &&
        ABS_DIFF(f->l_break_delta, f->r_break_delta) < CIR2_BREAK_DELTA_MARGIN)
    {
        return 0;
    }

    return (left_score >= CIR2_ENTRY_SCORE_MIN ||
            right_score >= CIR2_ENTRY_SCORE_MIN) ? 1 : 0;
}
/*第二入口检测，下方角点恢复黑色且断口消失*/
static uint8 _check_entry2(Circle2Feature *f)
{
    if(f->corner_bl == BLACK_PIXEL && f->corner_br == BLACK_PIXEL&&f->l_lost_cnt <= CIR2_LOST_THRESH && f->r_lost_cnt <= CIR2_LOST_THRESH)
    {
        return 1;
    }
    return 0;
}

/*--------------------------------------------------------------------------
 * _check_inside - ENTRY→INSIDE 条件
 *
 * 进入内部的标志:
 *   - 左圆环: 底部左下角变黑（车辆已经偏离原赛道进入圆环内侧）
 *             且断口侧大面积丢线
 *   - 右圆环: 底部右下角变黑
 *--------------------------------------------------------------------------*/
static uint8 _check_inside(Circle2Feature *f)
{
    if (cir2_dir == CIR2_LEFT)
    {
        /* 左圆环: 左下角变黑 + 左侧大面积丢线 */
        if (f->corner_bl == BLACK_PIXEL && f->l_lost_cnt >= CIR2_LOST_THRESH)
            return 1;
        /* 或者：两侧都丢线严重（已完全在圆环内） */
        if (f->l_lost_cnt >= CIR2_LOST_THRESH && f->r_lost_cnt >= CIR2_LOST_THRESH)
            return 1;
    }
    else if (cir2_dir == CIR2_RIGHT)
    {
        if (f->corner_br == BLACK_PIXEL && f->r_lost_cnt >= CIR2_LOST_THRESH)
            return 1;
        if (f->l_lost_cnt >= CIR2_LOST_THRESH && f->r_lost_cnt >= CIR2_LOST_THRESH)
            return 1;
    }
    return 0;
}

/*--------------------------------------------------------------------------
 * _check_exit - INSIDE→EXIT 条件
 *
 * 出口特征:
 *   - 对侧角落重新变白（恢复正常赛道边界）
 *   - 断口侧丢线减少
 *   - 或者出现新的断口但方向反转
 *--------------------------------------------------------------------------*/
static uint8 _check_exit(Circle2Feature *f)
{
    if (cir2_dir == CIR2_LEFT)
    {
        /* 左圆环出口: 右下角白+左下角白，且左侧丢线开始恢复 */
        if (f->corner_bl == WHITE_PIXEL &&
            f->corner_br == WHITE_PIXEL &&
            f->l_lost_cnt < CIR2_LOST_THRESH)
        {
            return 1;
        }
        /* 或：右侧出现新断口（出口第二断口特征） */
        if (f->r_break_row > 0 && f->l_lost_cnt < CIR2_LOST_THRESH)
        {
            return 1;
        }
    }
    else if (cir2_dir == CIR2_RIGHT)
    {
        if (f->corner_bl == WHITE_PIXEL &&
            f->corner_br == WHITE_PIXEL &&
            f->r_lost_cnt < CIR2_LOST_THRESH)
        {
            return 1;
        }
        if (f->l_break_row > 0 && f->r_lost_cnt < CIR2_LOST_THRESH)
        {
            return 1;
        }
    }
    return 0;
}

/*============================================================================
 *                         内部: 边线修复函数
 *============================================================================*/

/*--------------------------------------------------------------------------
 * _repair_left - 左圆环补线
 *
 * 策略: 左侧断口区域用线性插值修复左边线
 *   - 如果找到拐点: 从拐点到断口做线性插值
 *   - 如果没有拐点: 用对侧(右)边线偏移来推测左边线
 *   - 大面积丢线(INSIDE): 左边线设为 BORDER_MIN，用右边线 - 固定宽度
 *--------------------------------------------------------------------------*/
static void _repair_left(uint16 *l, uint16 *r, Circle2Feature *f)
{
    {
        uint8 row;
        uint8 anchor_top = 0;
        uint8 anchor_bot = 0;
        float k;
        float b;

        /*
         * Reworked repair path:
         * - repaired rows become valid immediately
         * - ENTRY can hold the previous anchors for a few frames
         * - ENTRY adds an inward pull so the midline commits to the circle
         */
        if (cir2_state == CIR2_INSIDE)
        {
            uint16 est_width = f->width_bottom;
            if (est_width < CIR2_WIDTH_NARROW) est_width = CIR2_WIDTH_NARROW;

            for (row = CIR2_SCAN_TOP; row <= CIR2_SCAN_BOT; row++)
            {
                if (l[row] <= BORDER_MIN + 1)
                {
                    int16 val = (int16)r[row] - (int16)est_width;
                    _set_repaired_point(l, l_border_valid, row, val);
                }
            }
            return;
        }

        if (f->l_break_row == 0)
        {
            if (cir2_state == CIR2_ENTRY &&
                cir2_entry_hold_cnt < CIR2_ENTRY_HOLD_FRAMES &&
                cir2_cached_anchor_top > 0 &&
                cir2_cached_anchor_bot > cir2_cached_anchor_top)
            {
                anchor_top = cir2_cached_anchor_top;
                anchor_bot = cir2_cached_anchor_bot;
                cir2_entry_hold_cnt++;
            }
            else
            {
                return;
            }
        }
        else
        {
            if (f->l_inflect_row > 0 && f->l_inflect_row < f->l_break_row)
            {
                anchor_top = f->l_inflect_row;
            }
            else
            {
                for (row = f->l_break_row; row > CIR2_SCAN_TOP; row--)
                {
                    if (l[row] > BORDER_MIN + 1 && l[row] < BORDER_MAX - 1)
                    {
                        anchor_top = row;
                        break;
                    }
                }
            }

            for (row = f->l_break_row; row <= CIR2_SCAN_BOT; row++)
            {
                if (l[row] > BORDER_MIN + 1 && l[row] < BORDER_MAX - 1)
                {
                    anchor_bot = row;
                    break;
                }
            }

            if (!anchor_bot) anchor_bot = CIR2_SCAN_BOT;
            if (!anchor_top) anchor_top = CIR2_SCAN_TOP;
            if (anchor_top >= anchor_bot) return;

            if (cir2_state == CIR2_ENTRY)
            {
                cir2_cached_anchor_top = anchor_top;
                cir2_cached_anchor_bot = anchor_bot;
                cir2_entry_hold_cnt = 0;
            }
        }

        if (anchor_top >= anchor_bot) return;

        k = (float)((int16)l[anchor_bot] - (int16)l[anchor_top])
            / (float)(anchor_bot - anchor_top);
        b = (float)l[anchor_top] - k * anchor_top;

        for (row = anchor_top; row <= anchor_bot; row++)
        {
            int16 val = (int16)(k * row + b);

            if (cir2_state == CIR2_ENTRY)
            {
                val -= (int16)_calc_entry_pull(anchor_top, anchor_bot, row);
            }

            _set_repaired_point(l, l_border_valid, row, val);
        }
        return;
    }

#if 0
    uint8 row;
    uint8 anchor_top = 0;   /* 断口上方的正常点 */
    uint8 anchor_bot = 0;   /* 断口下方的正常点 */
    float k;
    float b;

    if (cir2_state == CIR2_INSIDE)
    {
        /* 内部阶段: 左侧全丢线，用右边线 - 赛道宽度估计 */
        uint16 est_width = f->width_bottom;
        if (est_width < CIR2_WIDTH_NARROW) est_width = CIR2_WIDTH_NARROW;

        for (row = CIR2_SCAN_TOP; row <= CIR2_SCAN_BOT; row++)
        {
            if (l[row] <= BORDER_MIN + 1)
            {
                int16 val = (int16)r[row] - (int16)est_width;
                l[row] = (uint16)CLAMP(val, BORDER_MIN, BORDER_MAX);
            }
        }
        return;
    }

    /* ENTRY / EXIT 阶段: 断口区域线性插值 */
    if (f->l_break_row == 0) return;

    /* 上锚点: 拐点 或 断口往上第一个非丢线行 */
    if (f->l_inflect_row > 0 && f->l_inflect_row < f->l_break_row)
    {
        anchor_top = f->l_inflect_row;
    }
    else
    {
        for (row = f->l_break_row; row > CIR2_SCAN_TOP; row--)
        {
            if (l[row] > BORDER_MIN + 1 && l[row] < BORDER_MAX - 1)
            {
                anchor_top = row;
                break;
            }
        }
    }

    /* 下锚点: 断口下方（靠底部方向）的正常点 */
    for (row = f->l_break_row; row <= CIR2_SCAN_BOT; row++)
    {
        if (l[row] > BORDER_MIN + 1 && l[row] < BORDER_MAX - 1)
        {
            anchor_bot = row;
            break;
        }
    }

    /* 如果只找到一个锚点，用底部边界值 */
    if (!anchor_bot) anchor_bot = CIR2_SCAN_BOT;
    if (!anchor_top) anchor_top = CIR2_SCAN_TOP;
    if (anchor_top >= anchor_bot) return;

    /* 线性插值 */
    k = (float)((int16)l[anchor_bot] - (int16)l[anchor_top])
        / (float)(anchor_bot - anchor_top);
    b = (float)l[anchor_top] - k * anchor_top;

    for (row = anchor_top; row <= anchor_bot; row++)
    {
        int16 val = (int16)(k * row + b);
        l[row] = (uint16)CLAMP(val, BORDER_MIN, BORDER_MAX);
    }
#endif
}

/*--------------------------------------------------------------------------
 * _repair_right - 右圆环补线（左右镜像逻辑）
 *--------------------------------------------------------------------------*/
static void _repair_right(uint16 *l, uint16 *r, Circle2Feature *f)
{
    {
        uint8 row;
        uint8 anchor_top = 0;
        uint8 anchor_bot = 0;
        float k;
        float b;

        if (cir2_state == CIR2_INSIDE)
        {
            uint16 est_width = f->width_bottom;
            if (est_width < CIR2_WIDTH_NARROW) est_width = CIR2_WIDTH_NARROW;

            for (row = CIR2_SCAN_TOP; row <= CIR2_SCAN_BOT; row++)
            {
                if (r[row] >= BORDER_MAX - 1)
                {
                    int16 val = (int16)l[row] + (int16)est_width;
                    _set_repaired_point(r, r_border_valid, row, val);
                }
            }
            return;
        }

        if (f->r_break_row == 0)
        {
            if (cir2_state == CIR2_ENTRY &&
                cir2_entry_hold_cnt < CIR2_ENTRY_HOLD_FRAMES &&
                cir2_cached_anchor_top > 0 &&
                cir2_cached_anchor_bot > cir2_cached_anchor_top)
            {
                anchor_top = cir2_cached_anchor_top;
                anchor_bot = cir2_cached_anchor_bot;
                cir2_entry_hold_cnt++;
            }
            else
            {
                return;
            }
        }
        else
        {
            if (f->r_inflect_row > 0 && f->r_inflect_row < f->r_break_row)
            {
                anchor_top = f->r_inflect_row;
            }
            else
            {
                for (row = f->r_break_row; row > CIR2_SCAN_TOP; row--)
                {
                    if (r[row] > BORDER_MIN + 1 && r[row] < BORDER_MAX - 1)
                    {
                        anchor_top = row;
                        break;
                    }
                }
            }

            for (row = f->r_break_row; row <= CIR2_SCAN_BOT; row++)
            {
                if (r[row] > BORDER_MIN + 1 && r[row] < BORDER_MAX - 1)
                {
                    anchor_bot = row;
                    break;
                }
            }

            if (!anchor_bot) anchor_bot = CIR2_SCAN_BOT;
            if (!anchor_top) anchor_top = CIR2_SCAN_TOP;
            if (anchor_top >= anchor_bot) return;

            if (cir2_state == CIR2_ENTRY)
            {
                cir2_cached_anchor_top = anchor_top;
                cir2_cached_anchor_bot = anchor_bot;
                cir2_entry_hold_cnt = 0;
            }
        }

        if (anchor_top >= anchor_bot) return;

        k = (float)((int16)r[anchor_bot] - (int16)r[anchor_top])
            / (float)(anchor_bot - anchor_top);
        b = (float)r[anchor_top] - k * anchor_top;

        for (row = anchor_top; row <= anchor_bot; row++)
        {
            int16 val = (int16)(k * row + b);

            if (cir2_state == CIR2_ENTRY)
            {
                val += (int16)_calc_entry_pull(anchor_top, anchor_bot, row);
            }

            _set_repaired_point(r, r_border_valid, row, val);
        }
        return;
    }

#if 0
    uint8 row;
    uint8 anchor_top = 0;
    uint8 anchor_bot = 0;
    float k;
    float b;

    if (cir2_state == CIR2_INSIDE)
    {
        uint16 est_width = f->width_bottom;
        if (est_width < CIR2_WIDTH_NARROW) est_width = CIR2_WIDTH_NARROW;

        for (row = CIR2_SCAN_TOP; row <= CIR2_SCAN_BOT; row++)
        {
            if (r[row] >= BORDER_MAX - 1)
            {
                int16 val = (int16)l[row] + (int16)est_width;
                r[row] = (uint16)CLAMP(val, BORDER_MIN, BORDER_MAX);
            }
        }
        return;
    }

    if (f->r_break_row == 0) return;

    if (f->r_inflect_row > 0 && f->r_inflect_row < f->r_break_row)
    {
        anchor_top = f->r_inflect_row;
    }
    else
    {
        for (row = f->r_break_row; row > CIR2_SCAN_TOP; row--)
        {
            if (r[row] > BORDER_MIN + 1 && r[row] < BORDER_MAX - 1)
            {
                anchor_top = row;
                break;
            }
        }
    }

    for (row = f->r_break_row; row <= CIR2_SCAN_BOT; row++)
    {
        if (r[row] > BORDER_MIN + 1 && r[row] < BORDER_MAX - 1)
        {
            anchor_bot = row;
            break;
        }
    }

    if (!anchor_bot) anchor_bot = CIR2_SCAN_BOT;
    if (!anchor_top) anchor_top = CIR2_SCAN_TOP;
    if (anchor_top >= anchor_bot) return;

    k = (float)((int16)r[anchor_bot] - (int16)r[anchor_top])
        / (float)(anchor_bot - anchor_top);
    b = (float)r[anchor_top] - k * anchor_top;

    for (row = anchor_top; row <= anchor_bot; row++)
    {
        int16 val = (int16)(k * row + b);
        r[row] = (uint16)CLAMP(val, BORDER_MIN, BORDER_MAX);
    }
#endif
}
/*
 * 第一入口消失阶段补线
 * Left circle: find the right-most local peak of the left arc in ROI row 10~40,
 * then draw a straight repaired left border from the image lower-left to it.
 * Right circle mirrors this by finding the left-most local valley of the right
 * arc and drawing a straight repaired right border from the image lower-right.
 */
static uint8 _repair_approach_end(uint16 *l, uint16 *r, Circle2Feature *f)
{
    uint8 row;
    uint8 bulge_row = 0;
    uint16 bulge_x;
    uint16 *border;
    uint8 *valid;
    uint8 roi_top = 10;
    uint8 roi_bot = 40;
    uint8 bottom_row = CIR2_SCAN_BOT;
    int16 bottom_x;
    int16 dy;

    (void)f;

    if (cir2_dir == CIR2_LEFT)
    {
        bulge_x = BORDER_MIN;
        border = l;
        valid = l_border_valid;
        bottom_x = BORDER_MIN;
    }
    else if (cir2_dir == CIR2_RIGHT)
    {
        bulge_x = BORDER_MAX;
        border = r;
        valid = r_border_valid;
        bottom_x = BORDER_MAX;
    }
    else
    {
        return 0;
    }

    if (roi_top < CIR2_SCAN_TOP)
        roi_top = CIR2_SCAN_TOP;
    if (roi_bot > CIR2_SCAN_BOT)
        roi_bot = CIR2_SCAN_BOT;
    if (roi_bot <= roi_top + 2)
        return 0;

    for (row = roi_top + 1; row < roi_bot; row++)
    {
        uint16 x = border[row];

        if (!valid[row])
            continue;
        if (x <= BORDER_MIN + 1 || x >= BORDER_MAX - 1)
            continue;

        if (cir2_dir == CIR2_LEFT)
        {
            if (x >= (MT9V03X_W >> 1))
                continue;

            if (x >= border[row - 1] && x >= border[row + 1] && x > bulge_x)
            {
                bulge_x = x;
                bulge_row = row;
            }
        }
        else
        {
            if (x <= (MT9V03X_W >> 1))
                continue;

            if (x <= border[row - 1] && x <= border[row + 1] && x < bulge_x)
            {
                bulge_x = x;
                bulge_row = row;
            }
        }
    }

    if (bulge_row == 0 || bulge_row >= bottom_row)
        return 0;

    dy = (int16)bottom_row - (int16)bulge_row;
    if (dy <= 0)
        return 0;

    for (row = bulge_row; row <= bottom_row; row++)
    {
        int16 rel = (int16)bottom_row - (int16)row;
        int16 val = bottom_x +
                    (int16)(((int32)((int16)bulge_x - bottom_x) * rel + (dy >> 1)) / dy);

        _set_repaired_point(border, valid, row, val);
    }

    return 1;
}

static uint8 _get_model_width(Circle2Feature *f)
{
    uint16 width = f->width_bottom;

    if (width < CIR2_MODEL_WIDTH_MIN) width = CIR2_MODEL_WIDTH_MIN;
    if (width > CIR2_MODEL_WIDTH_MAX) width = CIR2_MODEL_WIDTH_MAX;

    return (uint8)width;
}

static uint8 _get_model_ratio_num(void)
{
    if (cir2_state == CIR2_ENTRY) return CIR2_ENTRY_RATIO_NUM;
    if (cir2_state == CIR2_EXIT)  return CIR2_EXIT_RATIO_NUM;
    return CIR2_INSIDE_RATIO_NUM;
}

static uint8 _get_model_ratio_den(void)
{
    if (cir2_state == CIR2_ENTRY) return CIR2_ENTRY_RATIO_DEN;
    if (cir2_state == CIR2_EXIT)  return CIR2_EXIT_RATIO_DEN;
    return CIR2_INSIDE_RATIO_DEN;
}

static void _recalc_circle_midline(uint16 *l, uint16 *r, Circle2Feature *f)
{
    uint8 row;
    uint8 model_width = _get_model_width(f);
    uint8 ratio_num = _get_model_ratio_num();
    uint8 ratio_den = _get_model_ratio_den();
    int16 offset = (int16)((model_width * ratio_num + (ratio_den >> 1)) / ratio_den);
    int16 last_good = MT9V03X_W / 2;

    for (row = 0; row < MT9V03X_H; row++)
    {
        int16 mid_val;

        leftline[row] = (int)l[row];
        rightline[row] = (int)r[row];

        if (cir2_dir == CIR2_LEFT)
        {
            if (l_border_valid[row])
            {
                mid_val = (int16)l[row] + offset;
                midline[row] = CLAMP(mid_val, BORDER_MIN, BORDER_MAX);
                last_good = midline[row];
            }
            else
            {
                midline[row] = last_good;
            }
        }
        else if (cir2_dir == CIR2_RIGHT)
        {
            if (r_border_valid[row])
            {
                mid_val = (int16)r[row] - offset;
                midline[row] = CLAMP(mid_val, BORDER_MIN, BORDER_MAX);
                last_good = midline[row];
            }
            else
            {
                midline[row] = last_good;
            }
        }
        else if (l_border_valid[row] && r_border_valid[row])
        {
            midline[row] = (int)(l[row] + r[row]) / 2;
            last_good = midline[row];
        }
        else
        {
            midline[row] = last_good;
        }
    }
}

/*--------------------------------------------------------------------------
 * _recalc_midline - 修复后重算中线（感知 valid 标记）
 *--------------------------------------------------------------------------*/
static void _recalc_midline(uint16 *l, uint16 *r)
{
    uint8 row;
    int last_good = MT9V03X_W / 2;
    for (row = 0; row < MT9V03X_H; row++)
    {
        if (l_border_valid[row] && r_border_valid[row]) {
            midline[row]   = (int)(l[row] + r[row]) / 2;
            last_good      = midline[row];
        } else {
            midline[row]   = last_good;
        }
        leftline[row]  = (int)l[row];
        rightline[row] = (int)r[row];
    }
}
