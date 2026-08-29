#ifndef _ELEM_GEOMETRY_H_
#define _ELEM_GEOMETRY_H_

#include "zf_common_headfile.h"

/* =============================================================================
 * elem_geometry.h — 元素检测共享几何原语
 *
 * 圈环(circle2)/十字(cross2)等元素状态机各自独立实现了大量相同的边线几何
 * 逻辑（中线重算、钳位补点、丢线统计、角点采样）。本模块抽公共部分，消除
 * 跨模块复制粘贴。纯静态/栈，无动态分配，适配 C251。
 *
 * 只含「两模块字节一致或仅参数不同」的原语；断口/宽度/斜率等语义有分歧的
 * 逻辑保留在各模块内，不强行合并（避免数值漂移）。
 * ============================================================================= */

/* 补线点标记值（circle2 与 cross2 均为 2） */
#define GEOM_REPAIR_VALID 2

/* 重算中线：感知 l/r_border_valid，非有效行沿用上一次有效中值 */
void elem_recalc_midline(uint16 *l, uint16 *r);

/* 写回一个补线点：钳位到 [BORDER_MIN, BORDER_MAX] 并标记为补线点 */
void elem_set_repaired_point(uint16 *border, uint8 *valid, uint8 row, int16 val);

/* 边线丢失统计：统计 l<=BORDER_MIN+1 / r>=BORDER_MAX-1 的行数；both_lost 可为 NULL */
void elem_extract_lost(const uint16 *l, const uint16 *r, uint8 top, uint8 bot,
                       uint8 *l_lost, uint8 *r_lost, uint8 *both_lost);

/* 角点像素采样：底部两角(行 H-2)；tl/tr(行 H/2) 传 NULL 则跳过 */
void elem_extract_corner(uint8 (*img)[MT9V03X_W],
                         uint8 *bl, uint8 *br, uint8 *tl, uint8 *tr,
                         uint8 bl_x, uint8 br_x, uint8 tl_x, uint8 tr_x);

#endif
