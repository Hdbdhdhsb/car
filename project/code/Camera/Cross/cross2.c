/*******************************************************************************
 * cross2.c - 新版十字处理模块
 *
 * 3状态机:  IDLE → CROSSING → RECOVER → IDLE
 *
 * 核心思路:
 *   1. 特征提取: 每帧扫描边线，提取断口、方向链码断点、宽度模型、丢线统计
 *   2. 状态判别: 双侧断口/宽度同时异常 → 十字; 单侧异常 → 不是十字(可能是圆环)
 *   3. 双侧补线: 断点上方取 N 行做最小二乘拟合斜率，向下延伸补线
 *   4. 中线重算: 补线后立即重算中线
 *
 ******************************************************************************/
#include "cross2.h"
#include "camera.h"

#define false 0
#define true 1
/*======================== 全局变量定义 ========================*/
Cross2State  cross2_state   = CROSS2_IDLE;
uint8        cross2_timeout = 0;

/*======================== 内部辅助函数声明 ========================*/
static void  _cr_extract_break(uint16 *l, uint16 *r, Cross2Feature *f);
static void   _cr_extract_dir_break(Cross2Feature *f);
static void   _cr_extract_width(uint16 *l, uint16 *r, Cross2Feature *f);
static void   _cr_extract_lost(uint16 *l, uint16 *r, Cross2Feature *f);
static void   _cr_extract_corner(uint8 (*img)[MT9V03X_W], Cross2Feature *f);
static void   _cr_calc_slope(uint8 status, uint16 *border, uint8 break_row, uint8 break_row2, float *slope, float *intercept);

static uint8  _cr_check_entry(Cross2Feature *f);
static uint8  _cr_check_recover(Cross2Feature *f);

static void   _cr_repair_both(uint8 status, uint16 *l, uint16 *r, Cross2Feature *f);
static void   _cr_recalc_midline(uint16 *l, uint16 *r);

/*======================== 辅助宏 ========================*/
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define CROSS2_REPAIR_VALID 2

static void _cr_set_repaired_point(uint16 *border, uint8 *valid, uint8 row, int16 val)
{
    border[row] = (uint16)CLAMP(val, BORDER_MIN, BORDER_MAX);
    valid[row] = CROSS2_REPAIR_VALID;
}

/*******************************************************************************
 * cross2_init - 复位十字状态
 ******************************************************************************/
void cross2_init(void)
{
    cross2_state   = CROSS2_IDLE;
    cross2_timeout = 0;
}

/*******************************************************************************
 * cross2_extract_feature - 一次性提取本帧所有十字相关特征
 ******************************************************************************/
uint8 cross2_extract_feature(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W],
                            Cross2Feature *feat)
{
    uint8 status = 1;
    feat->l_break_row     = 0;
    feat->r_break_row     = 0;
    feat->l_break_row2    = 0;
    feat->r_break_row2    = 0;
    feat->l_break_delta   = 0;
    feat->r_break_delta   = 0;
    feat->l_dir_break_row = 0;
    feat->r_dir_break_row = 0;
    feat->width_sum       = 0;
    feat->width_max       = 0;
    feat->width_min       = 255;
    feat->width_bottom    = 0;
    feat->width_mid       = 0;
    feat->wide_cnt        = 0;
    feat->l_lost_cnt      = 0;
    feat->r_lost_cnt      = 0;
    feat->both_lost_cnt   = 0;
    feat->corner_bl       = 0;
    feat->corner_br       = 0;
    feat->l_slope         = 0;
    feat->l_intercept     = 0;
    feat->r_slope         = 0;
    feat->r_intercept     = 0;

    _cr_extract_break(l, r, feat);
    _cr_extract_dir_break(feat);
    _cr_extract_width(l, r, feat);
    _cr_extract_lost(l, r, feat);
    _cr_extract_corner(img, feat);

    /* 在断点上方拟合斜率，后续补线用 */
    if (feat->l_break_row > CROSS2_FIT_LEN + CROSS2_SCAN_TOP&&feat->r_break_row > CROSS2_FIT_LEN + CROSS2_SCAN_TOP){
        status = 1;
    }else{
        status = 0;
    }
    _cr_calc_slope(status,l, feat->l_break_row, feat->l_break_row2, &feat->l_slope, &feat->l_intercept);
    _cr_calc_slope(status,r, feat->r_break_row, feat->r_break_row2, &feat->r_slope, &feat->r_intercept);
    return status;
}

/*******************************************************************************
 * cross2_process - 每帧调用一次的主入口
 *
 * 返回: 1 = 十字处理中 (CROSSING / RECOVER), 0 = IDLE
 ******************************************************************************/
uint8 cross2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W])
{
    uint8 normal = 0;
    Cross2Feature feat;
    uint8 status = 0;

    /* 1. 特征提取 */
    status = cross2_extract_feature(l, r, img, &feat);

    /* 2. 十字状态机 */
    switch (cross2_state)
    {
    /*---------- IDLE: 寻找十字入口 ----------*/
    case CROSS2_IDLE:
    {
        if (_cr_check_entry(&feat))
        {
            cross2_state   = CROSS2_CROSSING;
            cross2_timeout = 0;

            //通知外部总状态机
            current_track_type = CROSS_TRACK;
            current_state      = SPECIAL_MODE;

            /* 立即补一次线 */
            _cr_repair_both(status, l, r, &feat);
            _cr_recalc_midline(l, r);
        }
        break;
    }

    /*---------- CROSSING: 正在通过十字 ----------*/
    case CROSS2_CROSSING:
    {
        /* 每帧都补线 */
        _cr_repair_both(status, l, r, &feat);
        _cr_recalc_midline(l, r);

        /* 判断是否开始恢复 */
        if (_cr_check_recover(&feat))
        {
            cross2_state = CROSS2_RECOVER;

        }

        cross2_timeout++;
        if (cross2_timeout > CROSS2_TIMEOUT_MAX)
        {
            cross2_init();
            
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
        }
        break;
    }

    /*---------- RECOVER: 恢复中 ----------*/
    case CROSS2_RECOVER:
    {
        /* 恢复阶段仍然做轻量补线（断口可能还有残余） */
        _cr_repair_both(status, l, r, &feat);
        _cr_recalc_midline(l, r);

        /* 判断是否完全恢复正常 */
        if (feat.l_lost_cnt < 3 && feat.r_lost_cnt < 3 &&
            feat.l_break_row == 0 && feat.r_break_row == 0 &&
            feat.wide_cnt < 3)
        {
            normal = 1;
        }

        if (normal)
        {
            cross2_init();
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
            return 0;
        }

        cross2_timeout++;
        if (cross2_timeout > CROSS2_TIMEOUT_MAX)
        {
            cross2_init();
            current_track_type = NORMAL_TRACK;
            current_state      = SEARCH_MODE;
        }
        break;
    }

    default:
        cross2_init();
        break;
    }

    return (cross2_state != CROSS2_IDLE) ? 1 : 0;
}

/*******************************************************************************
 * cross2_get_state - 调试查询
 ******************************************************************************/
Cross2State cross2_get_state(void) { return cross2_state; }





/*============================================================================
 *                         内部: 特征提取函数
 *============================================================================*/

/*--------------------------------------------------------------------------
 * _cr_extract_break - 搜索左/右边线断口
 *
 * 从底部向上扫描，找到第一个跳变超过阈值的行
 * 十字特征: 左右两侧都出现断口
 *--------------------------------------------------------------------------*/
static void _cr_extract_break(uint16 *l, uint16 *r, Cross2Feature *f)
{
    uint8 row;

    /* 左边线断口: l[row] 突然变小（向画面左侧跳） */
    for (row = CROSS2_SCAN_BOT; row > CROSS2_SCAN_TOP + 1; row--)
    {
        int16 delta = (int16)l[row] - (int16)l[row - 2];
        int16 delta2_1 = (int16)l[row - 1] - (int16)l[row+1];
        int16 delta2_2 = (int16)l[row - 2] - (int16)l[row - 1];
        if (delta > CROSS2_BREAK_THRESH)
        {
            f->l_break_row   = row;
            f->l_break_delta = delta;
        }

        if (delta2_1 > CROSS2_BREAK_THRESH && delta2_2 < CROSS2_BREAK_THRESH)
        {
            f->l_break_row2   = row - 1;
            break;
        }
    }

    /* 右边线断口: r[row] 突然变大（向画面右侧跳） */
    for (row = CROSS2_SCAN_BOT; row > CROSS2_SCAN_TOP + 1; row--)
    {
        int16 delta = (int16)r[row - 2] - (int16)r[row];
        int16 delta2_1 = (int16)r[row+1] - (int16)r[row - 1];
        int16 delta2_2 = (int16)r[row - 1] - (int16)r[row - 2];
        if (delta > CROSS2_BREAK_THRESH)
        {
            f->r_break_row   = row;
            f->r_break_delta = delta;
        }

        if (delta2_1 > CROSS2_BREAK_THRESH && delta2_2 < CROSS2_BREAK_THRESH)
        {
            f->r_break_row2   = row - 1;
            break;
        }
    }
}

/*--------------------------------------------------------------------------
 * _cr_extract_dir_break - 利用八邻域方向链码寻找断点
 *
 * 原版旧代码用 dir_l/dir_r 判 {4,4} → {6,6,6} 模式来找十字断点。
 * 这里保留兼容: 如果链码中出现"连续向上→突然向上偏/向下"说明遇到了断口。
 * 结果记录为对应的行号（从 points_l/points_r 中取）。
 *--------------------------------------------------------------------------*/
static void _cr_extract_dir_break(Cross2Feature *f)
{
    uint16 i;
    extern uint16 data_stastics_l;
    extern uint16 data_stastics_r;

    /* 左侧链码断点 */
    if (data_stastics_l > 10)
    {
        for (i = 1; i + 7 < data_stastics_l; i++)
        {
            if (dir_l[i - 1] == CROSS2_DIR_UP && dir_l[i] == CROSS2_DIR_UP &&
                dir_l[i + 3] == CROSS2_DIR_BREAK_UP &&
                dir_l[i + 5] == CROSS2_DIR_BREAK_UP &&
                dir_l[i + 7] == CROSS2_DIR_BREAK_UP)
            {
                f->l_dir_break_row = (uint8)points_l[i][1];
                break;
            }
        }
    }

    /* 右侧链码断点 */
    if (data_stastics_r > 10)
    {
        for (i = 1; i + 7 < data_stastics_r; i++)
        {
            if (dir_r[i - 1] == CROSS2_DIR_UP && dir_r[i] == CROSS2_DIR_UP &&
                dir_r[i + 3] == CROSS2_DIR_BREAK_UP &&
                dir_r[i + 5] == CROSS2_DIR_BREAK_UP &&
                dir_r[i + 7] == CROSS2_DIR_BREAK_UP)
            {
                f->r_dir_break_row = (uint8)points_r[i][1];
                break;
            }
        }
    }
}

/*--------------------------------------------------------------------------
 * _cr_extract_width - 赛道宽度模型
 *--------------------------------------------------------------------------*/
static void _cr_extract_width(uint16 *l, uint16 *r, Cross2Feature *f)
{
    uint8 row;
    uint8 w;

    f->width_bottom = (r[CROSS2_SCAN_BOT] > l[CROSS2_SCAN_BOT])
                          ? (uint8)(r[CROSS2_SCAN_BOT] - l[CROSS2_SCAN_BOT])
                          : 0;

    for (row = CROSS2_SCAN_TOP; row <= CROSS2_SCAN_BOT; row++)
    {
        if (r[row] > l[row])
            w = (uint8)(r[row] - l[row]);
        else
            w = 0;

        f->width_sum += w;
        if (w > f->width_max) f->width_max = w;
        if (w < f->width_min) f->width_min = w;

        if (w >= CROSS2_WIDE_THRESH) f->wide_cnt++;

        if (row == MT9V03X_H / 2)
            f->width_mid = w;
    }
}

/*--------------------------------------------------------------------------
 * _cr_extract_lost - 边线丢失统计
 *--------------------------------------------------------------------------*/
static void _cr_extract_lost(uint16 *l, uint16 *r, Cross2Feature *f)
{
    uint8 row;

    for (row = CROSS2_SCAN_TOP; row <= CROSS2_SCAN_BOT; row++)
    {
        uint8 l_lost = (l[row] <= BORDER_MIN + 1) ? 1 : 0;
        uint8 r_lost = (r[row] >= BORDER_MAX - 1) ? 1 : 0;

        if (l_lost) f->l_lost_cnt++;
        if (r_lost) f->r_lost_cnt++;
        if (l_lost && r_lost) f->both_lost_cnt++;
    }
}

/*--------------------------------------------------------------------------
 * _cr_extract_corner - 底部两角像素采样
 *--------------------------------------------------------------------------*/
static void _cr_extract_corner(uint8 (*img)[MT9V03X_W], Cross2Feature *f)
{
    f->corner_bl = img[MT9V03X_H - 2][3];
    f->corner_br = img[MT9V03X_H - 2][MT9V03X_W - 4];
}

/*--------------------------------------------------------------------------
 * _cr_calc_slope - 在断点上方做最小二乘线性拟合
 *
 * 取 [break_row - FIT_LEN, break_row - 2] 这段边线数据拟合 y = k*x + b
 * 用于向下延伸补线
 *--------------------------------------------------------------------------*/
static void _cr_calc_slope(uint8 status, uint16 *border, uint8 break_row,uint8 break_row2, float *slope, float *intercept)
{
    int16  i;
    int16  ref_row   = (int16)break_row;
    int16  start_row = (int16)break_row + CROSS2_FIT_LEN;
    int16  end_row   = (int16)break_row + 2;
    float  sum_x  = 0;
    float  sum_y  = 0;
    float  sum_xy = 0;
    float  sum_x2 = 0;
    float  n = 0;
    float  x;
    float  y;
    float  denom;
    (void)status;

    *slope     = 0;
    *intercept = (float)border[break_row];
    //下断点触底时，改用上断点拟合
    if (cross2_state == CROSS2_RECOVER)
    {
        ref_row = (int16)break_row2;
        if (ref_row <= CROSS2_SCAN_TOP + 1)
        {
            return;
        }
        start_row = ref_row - 2;
        end_row   = ref_row - CROSS2_FIT_LEN;
    }
    else if (ref_row <= 0)
    {
        return;
    }

    if (start_row > CROSS2_SCAN_BOT)
        start_row = CROSS2_SCAN_BOT;
    if (end_row > CROSS2_SCAN_BOT)
        end_row = CROSS2_SCAN_BOT;
    if (start_row < CROSS2_SCAN_TOP)
        start_row = CROSS2_SCAN_TOP;
    if (end_row < CROSS2_SCAN_TOP)
        end_row = CROSS2_SCAN_TOP;
    if (start_row < end_row)
    {
        return;
    }

    for (i = start_row; i >= end_row; i--)
    {
        x = (float)i;
        y = (float)border[i];
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
        *slope     = 0;
        *intercept = (float)border[ref_row];
    }
}

/*============================================================================
 *                         内部: 状态判断函数
 *============================================================================*/

/*--------------------------------------------------------------------------
 * _cr_check_entry - IDLE→CROSSING 条件
 *
 * 十字入口特征（要求双侧同时异常，区别于圆环单侧异常）:
 *   方案A: 双侧边线断口同时存在
 *   方案B: 双侧链码断点同时存在
 *   方案C: 过宽行数足够多 + 底部角落为白色（排除全黑干扰）
 *
 * 额外排除: 如果只有一侧断口，不认为是十字（留给圆环处理）
 *--------------------------------------------------------------------------*/
static uint8 _cr_check_entry(Cross2Feature *f)
{
    /* 方案A: 边线跳变双侧都有 */
    uint8 both_break = (f->l_break_row > 0 && f->r_break_row > 0) ? 1 : 0;

    /* 方案B: 链码双侧都有断点 */
    uint8 both_dir   = (f->l_dir_break_row > 0 && f->r_dir_break_row > 0) ? 1 : 0;

    /* 方案C: 宽度模型判据 */
    uint8 width_wide = (f->wide_cnt >= CROSS2_WIDE_CNT_MIN) ? 1 : 0;

    /* 排除单侧断口（那更可能是圆环） */
    uint8 single_side = ((f->l_break_row > 0) ^ (f->r_break_row > 0)) ? 1 : 0;
    uint8 score = 0;
    if (single_side) return 0;

    /* 三选二: 至少满足两个方案才认为是十字 */
    if (both_break) score++;
    if (both_dir)   score++;
    if (width_wide) score++;

    return (score >= 2) ? 1 : 0;
}

/*--------------------------------------------------------------------------
 * _cr_check_recover - CROSSING→RECOVER 条件
 *
 * 断口开始消失 + 宽度回归正常
 *--------------------------------------------------------------------------*/
static uint8 _cr_check_recover(Cross2Feature *f)
{
    uint8 breaks_gone = 0;
    uint8 in_break = 0;
    uint8 lost_ok = 0;
    // /* 断口消失或变弱 */
    // breaks_gone = (f->l_break_row == 0 && f->r_break_row == 0) ? 1 : 0;

    // /* 过宽行数减少 */
    // width_ok = (f->wide_cnt < CROSS2_WIDE_CNT_MIN / 2) ? 1 : 0;

    // /* 丢线减少 */
    // lost_ok = (f->l_lost_cnt < CROSS2_LOST_THRESH &&
    //                  f->r_lost_cnt < CROSS2_LOST_THRESH) ? 1 : 0;

    // if (breaks_gone && width_ok)  return 1;
    // if (breaks_gone && lost_ok)   return 1;
    breaks_gone = (f->l_break_row <=  10|| f->r_break_row <=  10) ? 1 : 0;
    in_break = (f->l_break_row2 > 0 && f->r_break_row2 > 0) ? 1 : 0;
    if (breaks_gone && in_break)  return 1;

    return 0;
}

/*============================================================================
 *                         内部: 补线函数
 *============================================================================*/

/*--------------------------------------------------------------------------
 * _cr_repair_both - 双侧补线
 *
 * 策略:
 *   对每一侧，如果存在断口/断点:
 *     1. 取断点上方 FIT_LEN 行做最小二乘拟合
 *     2. 从断点处向下延伸到图像底部
 *   如果无断口（已经在 RECOVER 阶段）:
 *     不修改边线
 *   额外兜底: 如果丢线过多，强制补线（用对侧边线 - 估计宽度）
 *--------------------------------------------------------------------------*/
static void _cr_repair_both(uint8 status, uint16 *l, uint16 *r, Cross2Feature *f)
{
    int16 row;
    uint8 l_bp; 
    uint8 r_bp;
    uint8 l_bp2;
    uint8 r_bp2;
    uint16 est_w;
    int16 repair_start = -1;
    int16 repair_stop = -1;
    float k;
    float b;
    (void)status;
    /* 确定左侧断点行: 优先用边线断口，备选链码断点 */
    l_bp = f->l_break_row;
    l_bp2 = f->l_break_row2;
    if (!l_bp) l_bp = f->l_dir_break_row;

    /* 确定右侧断点行 */
    r_bp = f->r_break_row;
    r_bp2 = f->r_break_row2;
    if (!r_bp) r_bp = f->r_dir_break_row;

    /* ---------- 左侧补线 ---------- */
    if (l_bp > CROSS2_FIT_LEN + CROSS2_SCAN_TOP ||l_bp2 >0)
    {
        k = f->l_slope;
        b = f->l_intercept;

        if(cross2_state == CROSS2_RECOVER){
            //断点触底时，改用上断点补线补到底部
            repair_start = (l_bp2 > 0) ? (int16)l_bp2 : (int16)l_bp;
            if (repair_start < CROSS2_SCAN_TOP)
            {
                repair_start = CROSS2_SCAN_TOP;
            }
            for (row = repair_start; row <=CROSS2_SCAN_BOT ; row++)
            {
                int16 val = (int16)(k * row + b);
                _cr_set_repaired_point(l, l_border_valid, (uint8)row, val);
            }
        }else{
            /* 从下断点往上补线 */
            repair_start = (l_bp > 0) ? (int16)l_bp : (int16)l_bp2;
            repair_stop = (repair_start > CROSS2_FIX_LEN) ? (repair_start - CROSS2_FIX_LEN) : 0;
            for (row = repair_start; row >= repair_stop; row--)
            {
                int16 val = (int16)(k * row + b);
                _cr_set_repaired_point(l, l_border_valid, (uint8)row, val);
            }
        }


    }
    else if (f->l_lost_cnt > CROSS2_LOST_THRESH)
    {
        /* 大面积丢线兜底: 用右边线 - 底部宽度估计 */
        est_w = f->width_bottom;
        if (est_w < 20) est_w = 20;
        for (row = CROSS2_SCAN_TOP; row <= CROSS2_SCAN_BOT; row++)
        {
            if (l[row] <= BORDER_MIN + 1)
            {
                int16 val = (int16)r[row] - (int16)est_w;
                _cr_set_repaired_point(l, l_border_valid, (uint8)row, val);
            }
        }
    }

    /* ---------- 右侧补线 ---------- */
    if (r_bp > CROSS2_FIT_LEN + CROSS2_SCAN_TOP ||r_bp2 >0)
    {
        k = f->r_slope;
        b = f->r_intercept;

        if(cross2_state == CROSS2_RECOVER){
            //断点触底时，改用上断点补线补到底部
            repair_start = (r_bp2 > 0) ? (int16)r_bp2 : (int16)r_bp;
            if (repair_start < CROSS2_SCAN_TOP)
            {
                repair_start = CROSS2_SCAN_TOP;
            }
            for (row = repair_start; row <=CROSS2_SCAN_BOT ; row++)
            {
                int16 val = (int16)(k * row + b);
                _cr_set_repaired_point(r, r_border_valid, (uint8)row, val);
            }
        }else{
            /* 从下断点往上补线 */
            repair_start = (r_bp > 0) ? (int16)r_bp : (int16)r_bp2;
            repair_stop = (repair_start > CROSS2_FIX_LEN) ? (repair_start - CROSS2_FIX_LEN) : 0;
            for (row = repair_start; row >= repair_stop; row--)
            {
                int16 val = (int16)(k * row + b);
                _cr_set_repaired_point(r, r_border_valid, (uint8)row, val);
            }
        }
    }
    else if (f->r_lost_cnt > CROSS2_LOST_THRESH)
    {
        est_w = f->width_bottom;
        if (est_w < 20) est_w = 20;
        for (row = CROSS2_SCAN_TOP; row <= CROSS2_SCAN_BOT; row++)
        {
            if (r[row] >= BORDER_MAX - 1)
            {
                int16 val = (int16)l[row] + (int16)est_w;
                _cr_set_repaired_point(r, r_border_valid, (uint8)row, val);
            }
        }
    }
}

/*--------------------------------------------------------------------------
 * _cr_recalc_midline - 补线后重算中线（感知 valid 标记）
 *--------------------------------------------------------------------------*/
static void _cr_recalc_midline(uint16 *l, uint16 *r)
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
