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

/*======================== 内部辅助函数声明 ========================*/
static void   _extract_break(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _extract_inflect(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _extract_width(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _extract_lost(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W], Circle2Feature *f);
static void   _extract_corner(uint8 (*img)[MT9V03X_W], Circle2Feature *f);

static uint8  _check_entry(Circle2Feature *f);
static uint8  _check_inside(Circle2Feature *f);
static uint8  _check_exit(Circle2Feature *f);

static void   _repair_left(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _repair_right(uint16 *l, uint16 *r, Circle2Feature *f);
static void   _recalc_midline(uint16 *l, uint16 *r);

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
}

/*******************************************************************************
 * circle2_extract_feature - 一次性提取本帧所有圆环相关特征
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

    /* 1. 特征提取 */
    circle2_extract_feature(l, r, img, &feat);

    /* 2. 状态机 */
    switch (cir2_state)
    {
    /*---------- IDLE: 寻找圆环入口 ----------*/
    case CIR2_IDLE:
    {
        if (_check_entry(&feat))
        {
            cir2_state   = CIR2_ENTRY;
            cir2_timeout = 0;

            /* 判断左/右圆环 */
            if (feat.l_break_row && !feat.r_break_row)
                cir2_dir = CIR2_LEFT;
            else if (feat.r_break_row && !feat.l_break_row)
                cir2_dir = CIR2_RIGHT;
            else if (feat.l_break_delta > feat.r_break_delta)
                cir2_dir = CIR2_LEFT;
            else
                cir2_dir = CIR2_RIGHT;

            /* 通知外层状态机 */
            current_track_type = CIRCLE_TRACK;
            current_state      = SPECIAL_MODE;
        }
        break;
    }

    /*---------- ENTRY: 正在进入圆环 ----------*/
    case CIR2_ENTRY:
    {
        /* 补线 */
        if (cir2_dir == CIR2_LEFT)
            _repair_left(l, r, &feat);
        else
            _repair_right(l, r, &feat);
        _recalc_midline(l, r);

        /* 判断是否已完全进入圆环内部 */
        if (_check_inside(&feat))
        {
            cir2_state = CIR2_INSIDE;
        }
        /* 超时保护 */
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
        /* 内部也可能局部丢线，持续补线 */
        if (cir2_dir == CIR2_LEFT)
            _repair_left(l, r, &feat);
        else
            _repair_right(l, r, &feat);
        _recalc_midline(l, r);

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
        /* 出口阶段继续补线 */
        if (cir2_dir == CIR2_LEFT)
            _repair_left(l, r, &feat);
        else
            _repair_right(l, r, &feat);
        _recalc_midline(l, r);

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

    return (cir2_state != CIR2_IDLE) ? 1 : 0;
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
        if (delta > CIR2_BREAK_THRESH)
        {
            /* l[row] > l[row-1]：上面一行的左边界突然靠左（值变小）
             * 说明左侧出现断口 */
            f->l_break_row   = row;
            f->l_break_delta = delta;
            break;
        }
    }

    /* 右边线断口: 向上扫描，r[row] 突然变大（向右跳） */
    for (row = CIR2_SCAN_BOT; row > CIR2_SCAN_TOP + 1; row--)
    {
        int16 delta = (int16)r[row - 1] - (int16)r[row];
        if (delta > CIR2_BREAK_THRESH)
        {
            f->r_break_row   = row;
            f->r_break_delta = delta;
            break;
        }
    }
}

/*--------------------------------------------------------------------------
 * _extract_inflect - 搜索单侧边线拐点
 *
 * 拐点定义: 边线先增后减（或先减后增）的极值点
 * 在断口上方区域搜索，用于确认圆环弧形边线
 *--------------------------------------------------------------------------*/
static void _extract_inflect(uint16 *l, uint16 *r, Circle2Feature *f)
{
    uint8 row;
    uint8 search_end;

    /* 左边线拐点: 在断口上方寻找局部极大值 */
    search_end = (f->l_break_row > CIR2_INFLECT_RANGE + CIR2_SCAN_TOP)
                     ? (f->l_break_row - 15)
                     : CIR2_SCAN_TOP;
    if (search_end < CIR2_SCAN_TOP + CIR2_INFLECT_RANGE)
        search_end = CIR2_SCAN_TOP + CIR2_INFLECT_RANGE;

    for (row = search_end; row > CIR2_SCAN_TOP + CIR2_INFLECT_RANGE; row--)
    {
        if (l[row] > l[row + CIR2_INFLECT_RANGE] &&
            l[row] > l[row - CIR2_INFLECT_RANGE] &&
            l[row - CIR2_INFLECT_RANGE] != BORDER_MIN)
        {
            f->l_inflect_row = row;
            break;
        }
    }

    /* 右边线拐点: 局部极小值 */
    search_end = (f->r_break_row > CIR2_INFLECT_RANGE + CIR2_SCAN_TOP)
                     ? (f->r_break_row - 15)
                     : CIR2_SCAN_TOP;
    if (search_end < CIR2_SCAN_TOP + CIR2_INFLECT_RANGE)
        search_end = CIR2_SCAN_TOP + CIR2_INFLECT_RANGE;

    for (row = search_end; row > CIR2_SCAN_TOP + CIR2_INFLECT_RANGE; row--)
    {
        if (r[row] < r[row + CIR2_INFLECT_RANGE] &&
            r[row] < r[row - CIR2_INFLECT_RANGE] &&
            r[row - CIR2_INFLECT_RANGE] != BORDER_MAX)
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
 *
 * 判断丢线: 左边界贴 BORDER_MIN 或右边界贴 BORDER_MAX
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

/*============================================================================
 *                         内部: 状态判断函数
 *============================================================================*/

/*--------------------------------------------------------------------------
 * _check_entry - IDLE→ENTRY 条件
 *
 * 左圆环入口特征:
 *   1. 左侧出现断口 (l_break_row > 0)，且断口跳变量较大
 *   2. 右侧无断口（或断口很小）
 *   3. 断口上方能找到拐点（说明是连续弧线而非噪声）
 *   4. 右侧丢线少（排除十字等干扰）
 *
 * 右圆环同理取反
 *--------------------------------------------------------------------------*/
static uint8 _check_entry(Circle2Feature *f)
{
    uint8 left_entry  = 0;
    uint8 right_entry = 0;

    /* 左圆环入口 */
    if (f->l_break_row > 0 &&
        f->l_break_delta >= CIR2_BREAK_THRESH &&
        f->r_break_row == 0 &&
        f->l_inflect_row > 0 &&
        f->r_lost_cnt < CIR2_LOST_THRESH)
    {
        left_entry = 1;
    }

    /* 右圆环入口 */
    if (f->r_break_row > 0 &&
        f->r_break_delta >= CIR2_BREAK_THRESH &&
        f->l_break_row == 0 &&
        f->r_inflect_row > 0 &&
        f->l_lost_cnt < CIR2_LOST_THRESH)
    {
        right_entry = 1;
    }

    return (left_entry || right_entry);
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
}

/*--------------------------------------------------------------------------
 * _repair_right - 右圆环补线（左右镜像逻辑）
 *--------------------------------------------------------------------------*/
static void _repair_right(uint16 *l, uint16 *r, Circle2Feature *f)
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
