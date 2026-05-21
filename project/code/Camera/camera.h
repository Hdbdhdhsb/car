#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "zf_common_headfile.h"
#include "circle2.h"
#include "cross2.h"
#include "circle_detection.h"

/* 颜色定义 */
#define USER_RED     0xF800    /* 红色 */
#define USER_GREEN   0x07E0    /* 绿色 */
#define USER_BLUE    0x001F    /* 蓝色 */

/* 图像处理参数 */
#define WHITE_PIXEL 255
#define BLACK_PIXEL 0
#define BORDER_MAX  MT9V03X_W-2 
#define BORDER_MIN  1 
#define USE_NUM     MT9V03X_H*3 

/* 赛道元素类型 */
typedef enum {
    NORMAL_TRACK = 0,   /* 普通赛道 */
    CROSS_TRACK,        /* 十字路口 */
    ZEBRA_TRACK,        /* 斑马线 */
    CIRCLE_TRACK        /* 圆环 */
} TrackType;

/* 状态机状态 */
typedef enum {
    SEARCH_MODE = 0,    /* 搜索模式 */
    TRACKING_MODE,      /* 跟踪模式 */
    SPECIAL_MODE        /* 特殊元素处理模式 */
} StateType;

/* 全局变量声明 */
extern int midline[MT9V03X_H];/* 图像中线 */
extern int leftline[MT9V03X_H];
extern int rightline[MT9V03X_H];
extern float track_error;         /* 赛道中线与屏幕中线的偏差（用于PID反馈） */

extern uint8 binaryArray[MT9V03X_H][MT9V03X_W];

extern uint8 bin_image[MT9V03X_H][MT9V03X_W];

extern TrackType current_track_type;/* 当前赛道元素 */
extern StateType current_state;/* 当前状态机状态 */
extern uint8 emergency_stop;  /* 紧急停止标志 */




/* 八邻域相关变量 */
extern uint16 points_l[USE_NUM][2]; /* 左线点 */
extern uint16 points_r[USE_NUM][2]; /* 右线点 */
extern uint16 dir_r[USE_NUM];       /* 右边生长方向 */
extern uint16 dir_l[USE_NUM];       /* 左边生长方向 */
extern uint16 l_border[MT9V03X_H]; /* 左线数组 */
extern uint16 r_border[MT9V03X_H]; /* 右线数组 */
extern uint8  l_border_valid[MT9V03X_H]; /* 左边界有效标记 (1=实测, 2=插值, 0=无效) */
extern uint8  r_border_valid[MT9V03X_H]; /* 右边界有效标记 */




/* 斑马线相关变量 */
extern uint8 zebra_detected;     /* 斑马线检测标志 */
extern uint16 change_point;      /* 变化点计数器 */
extern uint32 last_zebra_time_ms;
extern uint8 zebra_count;             /* 斑马线检测计数 */
#define ZEBRA_MIN_INTERVAL 3000       /* 两次斑马线检测的最小间隔时间(ms) */



/* 函数声明 */
void Bin_Image_Filter(uint8 Bin_Image[MT9V03X_H][MT9V03X_W]);

int16 limit_a_b(int16 x, int a, int b);
int16 limit1(int16 x, int16 y);


void turn_to_bin(void);

uint8 get_start_point(uint8 start_row);
void search_l_r(uint16 break_flag, uint8(*image)[MT9V03X_W], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest);
void get_left(uint16 total_L);
void get_right(uint16 total_R);
uint8 extract_borders(void);
void calc_midline(void);
void image_filter(uint8(*bin_image)[MT9V03X_W]);
void image_draw_rectan(uint8(*image)[MT9V03X_W]);





/* 状态机 */
void state_machine_init(void);
void detect_track_type(void);
void normal_track_process(void);
uint8 cross2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);
uint8 circle2_process(uint16 *l, uint16 *r, uint8 (*img)[MT9V03X_W]);
void image_process();


#endif 