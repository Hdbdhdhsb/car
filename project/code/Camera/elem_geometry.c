#include "elem_geometry.h"

/* BORDER_MIN/MAX 由 camera.h 派生(1 / MT9V03X_W-2)；此处本地派生，避免依赖 camera.h 的耦合 */
#define GEOM_BORDER_MAX         (MT9V03X_W - 2)
#define GEOM_BORDER_MIN         1
#define GEOM_CLAMP(x, lo, hi)   ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* camera.c 定义的全局数组（提边后结果） */
extern int    midline[MT9V03X_H];
extern int    leftline[MT9V03X_H];
extern int    rightline[MT9V03X_H];
extern uint8  l_border_valid[MT9V03X_H];
extern uint8  r_border_valid[MT9V03X_H];

/*--------------------------------------------------------------------------
 * elem_recalc_midline - 补线后重算中线（感知 valid 标记）
 *--------------------------------------------------------------------------*/
void elem_recalc_midline(uint16 *l, uint16 *r)
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

/*--------------------------------------------------------------------------
 * elem_set_repaired_point - 钳位写回一个补线点并标记补线
 *--------------------------------------------------------------------------*/
void elem_set_repaired_point(uint16 *border, uint8 *valid, uint8 row, int16 val)
{
    border[row] = (uint16)GEOM_CLAMP(val, GEOM_BORDER_MIN, GEOM_BORDER_MAX);
    valid[row]  = GEOM_REPAIR_VALID;
}

/*--------------------------------------------------------------------------
 * elem_extract_lost - 边线丢失统计（both_lost 可为 NULL）
 *--------------------------------------------------------------------------*/
void elem_extract_lost(const uint16 *l, const uint16 *r, uint8 top, uint8 bot,
                       uint8 *l_lost, uint8 *r_lost, uint8 *both_lost)
{
    uint8 row;
    *l_lost = 0;
    *r_lost = 0;
    if (both_lost) *both_lost = 0;
    for (row = top; row <= bot; row++)
    {
        uint8 l_lost_row = (l[row] <= GEOM_BORDER_MIN + 1) ? 1 : 0;
        uint8 r_lost_row = (r[row] >= GEOM_BORDER_MAX - 1) ? 1 : 0;
        if (l_lost_row) (*l_lost)++;
        if (r_lost_row) (*r_lost)++;
        if (both_lost && l_lost_row && r_lost_row) (*both_lost)++;
    }
}

/*--------------------------------------------------------------------------
 * elem_extract_corner - 四角像素采样（tl/tr 传 NULL 则跳过）
 *--------------------------------------------------------------------------*/
void elem_extract_corner(uint8 (*img)[MT9V03X_W],
                         uint8 *bl, uint8 *br, uint8 *tl, uint8 *tr,
                         uint8 bl_x, uint8 br_x, uint8 tl_x, uint8 tr_x)
{
    *bl = img[MT9V03X_H - 2][bl_x];
    *br = img[MT9V03X_H - 2][br_x];
    if (tl) *tl = img[MT9V03X_H / 2][tl_x];
    if (tr) *tr = img[MT9V03X_H / 2][tr_x];
}
