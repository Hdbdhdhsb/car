#ifndef _VISION_CONFIG_H_
#define _VISION_CONFIG_H_
/* ============================================================================= * config.h — 视觉参数集中配置 * * 只收【需要调】的视觉参数；派生结构常量(BORDER_MAX / USE_NUM / CIR2_SCAN_TOP / * CROSS2_SCAN_TOP / CIR2_EXIT_ANCHOR_Y 等)与协议 ABI 常量留在各自模块， * 避免 include 顺序冲突与对端同步问题。 * * 本文件零依赖：不接受 MT9V03X_W/H 等，纯数字常量，可被任意头文件安全包含。 * ============================================================================= */
/* -------------------- 二值化 / 灰度 -------------------- */
#define VISION_BIN_THRESHOLD     170   
/* 二值化阈值：像素灰度>此值判为白 */
/* -------------------- 赛道边线补线 -------------------- */
#define BORDER_GAP_MAX           3     
/* 短缺口允许跳过的最大行数(<=此值才线性插值补线) */
/* -------------------- 控制采样行 / 误差加权 -------------------- */
#define TRACK_SAMPLE_OFFSET_NEAR 14    
/* 主采样行：自屏底向上偏移14行(即 MT9V03X_H-14) */
#define TRACK_SAMPLE_OFFSET_FAR  8     
/* 副采样行：自屏底向上偏移8行 (即 MT9V03X_H-8)  */
#define TRACK_ERROR_W_NEAR       0.6f  
/* 主采样行误差权重(error1) */
#define TRACK_ERROR_W_FAR        0.4f  
/* 副采样行误差权重(error2) */
#define TRACK_ERR_MAX_STEP       15.0f 
/* 单帧 track_error 最大变化量(像素): 变化率限幅, 压制坏帧尖峰 -> 舵机不抽 */
#define TRACK_ERR_LPF_ALPHA      0.5f  
/* track_error 一阶低通系数(新值权重, 1=不滤波) */
/* half width estimate: bottom N rows, real scan points only */
#define CALC_HALF_WIDTH_ROWS     25
#define CALC_HALF_WIDTH_MIN      8
#define CALC_HALF_WIDTH_MAX      58
/* -------------------- 提边搜索 -------------------- */
#define EDGE_START_ROW_OFF       2     
/* 提边起始行：自屏底向上偏移2行 */
#define EDGE_MIN_ROW_DIV         3     
/* 提边最小行：取 MT9V03X_H/3 之上才开始扫描 */
#define LOST_TO_SEARCH_THRESH    5     
/* 连续丢线>此值 -> 退回 SEARCH_MODE */
/* -------------------- 斑马线 -------------------- */
#define ZEBRA_MIN_INTERVAL       3000  
/* 两次斑马线检测最小间隔(ms) */
#define ZEBRA_CHANGE_PT_THRESH   15    
/* 黑白变化点>此值 -> 判为斑马线 */
#define ZEBRA_DETECT_ROW_OFF1    3     
/* 斑马线检测行1：MT9V03X_H-3 */
#define ZEBRA_DETECT_ROW_OFF2    5     
/* 斑马线检测行2：MT9V03X_H-5 */
#define ZEBRA_DETECT_ROW_OFF3    7     
/* 斑马线检测行3：MT9V03X_H-7 */
#define ZEBRA_STOP_COUNT         2     
/* 第几次检测到斑马线后停车 */
#define ZEBRA_CLEAR_CNT          5     
/* 连续多少帧无条纹 -> 驶出斑马线复位 */
/* -------------------- 圆环 circle2 -------------------- */
#define CIR2_BREAK_THRESH         5    //断口判定阈值
#define CIR2_INFLECT_RANGE        5    //拐点范围
#define CIR2_LOST_THRESH          10   //丢线阈值
#define CIR2_WIDTH_NARROW         20   //最小赛道宽度
#define CIR2_TIMEOUT_MAX          80   //环岛处理超时(帧)
#define CIR2_BREAK_MIN_ROW        10   //断口行下限(需 > 10)
#define CIR2_FIT_LEN              10   //斜率拟合长度(行)
#define CIR2_APPROACH_REPAIR_LEN  20   //approach阶段补线长度(行)
#define CIR2_INFLECT_TREND_THRESH 6    //拐点趋势阈值
#define CIR2_REBOUND_TREND_THRESH 4    //突跳后回摆趋势阈值
#define CIR2_BREAK_DELTA_MARGIN   4    //双断口主次差阈值
#define CIR2_WIDTH_DIFF_MIN       4    //入口宽度变化阈值
#define CIR2_ENTRY_SCORE_MIN      3    //入口判定最小得分
/* 环岛入口的对面必须是连续实线(丢线行数 <= 此值)。 * 拐弯后未回正的十字：对面看到的是纯白赛道、根本没有线 -> 丢线多 -> 不是环岛。 */
#define CIR2_ENTRY_OPP_MAX_LOST   18
/* APPROACH_END 卡死保护：补线会把"缺口/丢线"特征自己抹平，导致 _check_entry() * 永远不成立 -> 一直补线 -> 直接开过环岛。持续补满这么多帧就强制进岛。 * 早进=切得早(可能撞内圈/切弯心)，晚进=冲过头。先给 12，实车再调。 */
#define CIR2_ENTRY_FORCE_FRAMES   12
/* APPROACH 驻留超过这么多帧还没等到第一入口消失 -> 直接推进到 APPROACH_END。 * 防止 _check_approch_exit() 永远不成立、一直卡在 APPROACH 光补线不进岛。 */
#define CIR2_APPROACH_FORCE_FRAMES 30
/* EXIT_END 至少保持这么多帧才允许交回普通巡线， * 否则刚出环就把转向收了、车直直冲出去。 */
#define CIR2_EXIT_END_MIN_FRAMES  8
#define CIR2_ENTRY_HOLD_FRAMES    3    //入口锚点保持帧数
#define CIR2_ENTRY_PULL_MIN       4    //入口补线最小内拉
#define CIR2_ENTRY_PULL_MAX       18   //入口补线最大内拉
#define CIR2_MODEL_WIDTH_MIN      24   //宽度模型最小宽度
#define CIR2_MODEL_WIDTH_MAX      70   //宽度模型最大宽度
#define CIR2_ENTRY_RATIO_NUM      2    //ENTRY 单侧中线宽度系数分子
#define CIR2_ENTRY_RATIO_DEN      5    //ENTRY 单侧中线宽度系数分母
#define CIR2_INSIDE_RATIO_NUM     1    //INSIDE 单侧中线宽度系数分子
#define CIR2_INSIDE_RATIO_DEN     2    //INSIDE 单侧中线宽度系数分母
#define CIR2_EXIT_RATIO_NUM       9    //EXIT 单侧中线宽度系数分子
#define CIR2_EXIT_RATIO_DEN       20   //EXIT 单侧中线宽度系数分母
#define CIR2_ENTRY_ERR_ROW_A      24   //入口前瞻采样行 A(由 camera.c 控制采样用)
#define CIR2_ENTRY_ERR_ROW_B      16   //入口前瞻采样行 B
#define CIR2_EXITING_MIN_FRAMES   1    //EXITING 阶段最小帧数
#define CIR2_EXIT_RECOVER_LOST    5    //出口恢复判定: 单侧丢线上限
#define CIR2_EXIT_WIDTH_TOP       18   //出口宽度模型-顶部半宽
#define CIR2_EXIT_WIDTH_BOTTOM    44   //出口宽度模型-底部半宽
/* -------------------- 十字 cross2 -------------------- */
#define CROSS2_BREAK_THRESH      8     //边线跳变阈值
#define CROSS2_WIDE_THRESH       100   //赛道"过宽"阈值(像素)
#define CROSS2_WIDE_CNT_MIN      5     //过宽行数阈值(认定为十字)
#define CROSS2_LOST_THRESH       10    //丢线行数阈值
#define CROSS2_TIMEOUT_MAX       40    //超时帧数上限
#define CROSS2_FIT_LEN           10    //拟合用行数(断点上方取多少行做最小二乘)
#define CROSS2_FIX_LEN           25    //补线固定长度(行)
#define CROSS2_RECOVER_LOST      3     //恢复判定: 单侧丢线需 < 此值
#define CROSS2_RECOVER_WIDE      3     //恢复判定: 过宽行需 < 此值
#define CROSS2_RECOVER_BREAK_ROW 10    //恢复判定: 断口行 <= 此值即认为断口已消失
#define CROSS2_FALLBACK_MIN_WIDTH 20   //丢线兜底补线时, 估计轨道最小宽度
/* -------------------- PID 转向闭环 -------------------- */
/* 经验初值，需上赛道实调。track_error(像素) 经位置式 PID 得到转向角(度)。 */
#define PID_DEFAULT_KP         10.0f   
/* 比例：像素误差 -> 转向角度 */
#define PID_DEFAULT_KI         0.0f   
/* 积分：先关，上赛道后按需 */
#define PID_DEFAULT_KD         0.1f  
/* 微分：经验初值 */
#define PID_INTEGRAL_MAX       10.0f  
/* 积分限幅(Ki=0 时仅作上限保护) */
#define PID_OUTPUT_MAX         150.0f   
/* 转向限幅：对应下位机 ±45° */
#define PID_STEER_SIGN         1.0f   
/* 转向符号：待断电/架车实测 track_error 正方向；±1 反转 */
#define PID_TARGET_SPEED       5.0f   
/* 目标车速：先沿用 LOWER_START_SPEED0(开环常量) */
/* -------------------- 圆检测 circle_detection -------------------- */
/* 注: circle_detection_process 当前未接入状态机(状态机用的是 circle2_process)， * 这两个常量暂为休眠参数，保留以便复用。 */
#define CIRCLE_MIN_LINE_WIDTH     2U  //圆形检测: 单段黑色最小宽(像素)
#define CIRCLE_MIN_BLACK_SEGMENTS 2U  //圆形检测: 最少黑色段数
#endif 
/* _VISION_CONFIG_H_ */
