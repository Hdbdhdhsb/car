#include "camera.h"
#include "main.h"

#define threshold 100 /* 二值化阈值 */

//方案2：detect_track_type 改成隔帧或条件触发


#define false 0
#define true 1
/* 全局变量定义 */
int midline[MT9V03X_H] = {0};
int leftline[MT9V03X_H] = {0};
int rightline[MT9V03X_H] = {0};
float track_error = 0.0f;

uint8 bin_image[MT9V03X_H][MT9V03X_W] = {0};/*二值化后的图像*/


/* 八邻域相关变量 */
uint16 points_l[USE_NUM][2] = {0}; /* 左线点*/
uint16 points_r[USE_NUM][2] = {0}; /* 右线点*/
uint16 dir_r[USE_NUM] = {0};       /* 右赛道生长方向集合数组*/
uint16 dir_l[USE_NUM] = {0};       /* 左赛道生长方向集合数组*/
uint16 data_stastics_l = 0;        /* 左边找到点的个数*/
uint16 data_stastics_r = 0;        /* 右边找到点的个数*/

uint16 l_border[MT9V03X_H];       /* 左线数组*/
uint16 r_border[MT9V03X_H];       /* 右线数组*/
uint8  l_border_valid[MT9V03X_H]; /* 左边界有效标记 (1=八邻域实测, 2=插值补线, 0=无效)*/
uint8  r_border_valid[MT9V03X_H]; /* 右边界有效标记*/
uint16 l_index[MT9V03X_H];        /* 左边界索引数组*/
uint16 r_index[MT9V03X_H];        /* 右边界索引数组*/


/* 状态机相关变量 */
TrackType current_track_type = NORMAL_TRACK;
StateType current_state = SEARCH_MODE;
uint8 lost_counter = 0;            /* 丢线计数器*/
uint8 cross_detected = 0;          /* 十字检测标志*/
int16 open_speed = 0;            /* 开环速度*/

/* 斑马线相关变量 */
uint8 zebra_detected = 0;          /* 斑马线检测标志*/
uint16 change_point = 0;           /* 变化点计数器*/
uint32 last_zebra_time_ms = 0; /* 上次斑马线检测时间(ms) */
uint8 zebra_count = 0;       /* 斑马线检测计数*/


/* 十字相关变量 */
uint8 flag_cross = 0;              /* 十字标志位*/
int cross_jud = 0;                 /* 十字状态判断*/
int cross_jud_time = 0;            /* 十字状态计时*/



/* 使用图像下方采样行作为控制误差，减去屏幕中心得到横向偏差 */
static void update_track_error(void) {
    uint8 sample_row = (MT9V03X_H > 5) ? (MT9V03X_H - 5) : (MT9V03X_H - 1);
    int16 screen_center = MT9V03X_W / 2;
    track_error = (float)((int16)midline[sample_row] - screen_center);
}


/* 图像二值化 */
void turn_to_bin(void) {
    uint8 i, j;
    
    for(i = 0; i < MT9V03X_H; i++) {
        for(j = 0; j < MT9V03X_W; j++) {
            bin_image[i][j] = (mt9v03x_image[i][j] > threshold) ? WHITE_PIXEL : BLACK_PIXEL;
        }
    }
}

uint8 start_point_l[2] = {0}; /* 左线起点的x,y坐标 */
uint8 start_point_r[2] = {0}; /* 右线起点的x,y坐标 */
uint8 get_start_point(uint8 start_row) {
    uint8 i = 0, l_found = 0, r_found = 0;
    uint8 *p_row;
    
    start_point_l[0] = 0; 
    start_point_l[1] = 0; 
    start_point_r[0] = 0; 
    start_point_r[1] = 0; 

    p_row = bin_image[start_row];
    
    /* 从中间往左边找起点(往左扫到白黑黑视为边线) */
    for(i = MT9V03X_W / 2; i > BORDER_MIN; i--) {
        start_point_l[0] = i; 
        start_point_l[1] = start_row; 
        if(*(p_row + i) == WHITE_PIXEL && *(p_row + i - 1) == BLACK_PIXEL && *(p_row + i - 2) == BLACK_PIXEL) {
            l_found = 1;
            break;
        }
    }

    /* 从中间往右边找起点（往右扫到白黑黑视为边线）*/
    for(i = MT9V03X_W / 2; i < BORDER_MAX; i++) {
        start_point_r[0] = i;
        start_point_r[1] = start_row; 
        if(*(p_row + i) == WHITE_PIXEL && *(p_row + i + 1) == BLACK_PIXEL && *(p_row + i + 2) == BLACK_PIXEL) {
            r_found = 1;
            break;
        }
    }

    return (l_found && r_found) ? 1 : 0;
}

/* 八邻域搜索主函数 */
void search_l_r(uint16 break_flag, uint8(*image)[MT9V03X_W], uint16 *l_stastic, uint16 *r_stastic, 
               uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest) {
    uint8 i;
    uint8 next_i;
    uint8 best_x_l, best_y_l, best_x_r, best_y_r;
    int16 x_cur, y_cur, x_next, y_next;
    uint8 *p_row_cur;
    uint8 *p_row_next;
    
    /* 左边变量 */
    int16 search_filds_l[8][2] = {0};
    uint8 index_l = 0;
    uint8 center_point_l[2] = {0};
    uint16 l_data_statics = *l_stastic;
    
    /* 左边搜索方向 */
    static int8 seeds_l[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, 
        {0, -1}, {1, -1}, {1, 0}, {1, 1}
    };
    
    /* 右边变量 */
    int16 search_filds_r[8][2] = {0};
    uint8 center_point_r[2] = {0};
    uint8 index_r = 0;
    uint16 r_data_statics = *r_stastic;
    
    /* 右边搜索方向 */
    static int8 seeds_r[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, 
        {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };
    
    /* 初始化中心点 */
    center_point_l[0] = l_start_x;
    center_point_l[1] = l_start_y;
    center_point_r[0] = r_start_x;
    center_point_r[1] = r_start_y;
    
    /* 开始八邻域搜索 */
    while(break_flag--) {
        /* 左边处理 */
        for(i = 0; i < 8; i++) {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
        }
        
        points_l[l_data_statics][0] = center_point_l[0];
        points_l[l_data_statics][1] = center_point_l[1];
        l_data_statics++;
        
        /* 右边处理 */
        for(i = 0; i < 8; i++) {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
        }
        
        points_r[r_data_statics][0] = center_point_r[0];
        points_r[r_data_statics][1] = center_point_r[1];
        
        index_l = 0;
        
        /* 左边边界判断（指向偏移访问图像） */
        for(i = 0; i < 8; i++) {
            next_i = (uint8)((i + 1) & 7);
            x_cur  = search_filds_l[i][0];
            y_cur  = search_filds_l[i][1];
            x_next = search_filds_l[next_i][0];
            y_next = search_filds_l[next_i][1];

            if(x_cur < 0 || x_cur >= MT9V03X_W || y_cur < 0 || y_cur >= MT9V03X_H ||
               x_next < 0 || x_next >= MT9V03X_W || y_next < 0 || y_next >= MT9V03X_H) {
                continue;
            }

            p_row_cur  = image[y_cur];
            p_row_next = image[y_next];
            if(p_row_cur[x_cur] == BLACK_PIXEL && p_row_next[x_next] == WHITE_PIXEL) {
                if(index_l == 0 || best_y_l > (uint8)y_cur) {
                    best_x_l = (uint8)x_cur;
                    best_y_l = (uint8)y_cur;
                }
                index_l++;
                dir_l[l_data_statics-1] = i;
            }
        }
        
        if(index_l) {
            center_point_l[0] = best_x_l;
            center_point_l[1] = best_y_l;
        }
        
        /* 终止*/
        if((r_data_statics >= 2 && l_data_statics >= 3) &&
           ((points_r[r_data_statics][0] == points_r[r_data_statics-1][0] && 
             points_r[r_data_statics][0] == points_r[r_data_statics-2][0] &&
             points_r[r_data_statics][1] == points_r[r_data_statics-1][1] && 
             points_r[r_data_statics][1] == points_r[r_data_statics-2][1]) ||
            (points_l[l_data_statics-1][0] == points_l[l_data_statics-2][0] && 
             points_l[l_data_statics-1][0] == points_l[l_data_statics-3][0] &&
             points_l[l_data_statics-1][1] == points_l[l_data_statics-2][1] && 
             points_l[l_data_statics-1][1] == points_l[l_data_statics-3][1]))) {
            break;
        }
        
        if(my_abs(points_r[r_data_statics][0] - points_l[l_data_statics-1][0]) < 2 &&
           my_abs(points_r[r_data_statics][1] - points_l[l_data_statics-1][1]) < 2) {
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics-1][1]) >> 1;
            break;
        }
        
        if(points_r[r_data_statics][1] < points_l[l_data_statics-1][1]) {
            continue; /* 如果左边比右边高，左边等待右边 */
        }
        
        if(dir_l[l_data_statics-1] == 7 && 
           (points_r[r_data_statics][1] > points_l[l_data_statics-1][1])) {
            /* 左边开始向下走了，等待右边 */
            center_point_l[0] = points_l[l_data_statics-1][0];
            center_point_l[1] = points_l[l_data_statics-1][1];
            l_data_statics--;
        }
        
        r_data_statics++;
        
        index_r = 0;
        
        /* 右边边界判断（指向偏移访问图像） */
        for(i = 0; i < 8; i++) {
            next_i = (uint8)((i + 1) & 7);
            x_cur  = search_filds_r[i][0];
            y_cur  = search_filds_r[i][1];
            x_next = search_filds_r[next_i][0];
            y_next = search_filds_r[next_i][1];

            if(x_cur < 0 || x_cur >= MT9V03X_W || y_cur < 0 || y_cur >= MT9V03X_H ||
               x_next < 0 || x_next >= MT9V03X_W || y_next < 0 || y_next >= MT9V03X_H) {
                continue;
            }

            p_row_cur  = image[y_cur];
            p_row_next = image[y_next];
            if(p_row_cur[x_cur] == BLACK_PIXEL && p_row_next[x_next] == WHITE_PIXEL) {
                if(index_r == 0 || best_y_r > (uint8)y_cur) {
                    best_x_r = (uint8)x_cur;
                    best_y_r = (uint8)y_cur;
                }
                index_r++;
                dir_r[r_data_statics-1] = i;
            }
        }
        
        if(index_r) {
            center_point_r[0] = best_x_r;
            center_point_r[1] = best_y_r;
        }
    }
    
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

/* 获取左边界线(赛道线) —— 允许跳行 + 有效标记 + 短缺口插值 */
#define BORDER_GAP_MAX  3   /* 允许跳过的最大多行数 */
void get_left(uint16 total_L) {
    uint8 i = 0;
    uint16 j = 0;
    uint8 row = 0;
    uint8 last_valid = 0;
    uint8 gap_start = 0;
    uint8 next_valid = 0;
    uint8 found = 0;
    uint8 k = 0;
    uint8 gap_len = 0;
    uint8 span = 0;
    uint8 step = 0;
    int16 delta = 0;
    int16 base = 0;
    int16 val = 0;
    //赛道线默认值
    for(i = 0; i < MT9V03X_H; i++) {
        l_border[i] = BORDER_MIN;
        l_index [i] = 0;
        l_border_valid[i] = 0;
    }
    
    /* 遍历所有八邻域点，按行号直接写入 */
    for(j = 0; j < total_L; j++) {
        row = (uint8)points_l[j][1];
        if(row >= MT9V03X_H) continue;
        /* 每行只取第一个命中的点（八邻域由底部向上扫，先到的更靠外侧） */
        if(l_border_valid[row] == 0) {
            l_border[row] = points_l[j][0] + 1;
            l_index [row] = j;
            l_border_valid[row] = 1;  /* 1 = 实测点 */
        }
    }
    
    /* 短缺口线性插值补线
       从底部往上扫，找到连续的无效段，如果段长 <= BORDER_GAP_MAX 且两端有效就插值 */
    /* 先找到底部第一个有效行 */
    for(i = MT9V03X_H - 1; i > 0; i--) {
        if(l_border_valid[i]) { last_valid = i; break; }
    }
    
    for(i = last_valid; i > 0; ) {
        /* 找下一个无效行 */
        if(l_border_valid[i]) { last_valid = i; i--; continue; }
        /* i 是无效行，往上找下一个有效行 */
        gap_start = i;  /* 无效段的底行 */
        next_valid = 0;
        found = 0;
        for(k = i; k > 0; k--) {
            if(l_border_valid[k]) { next_valid = k; found = 1; break; }
        }
        if(!found) break;  
        
        gap_len = gap_start - next_valid;  /* 缺口行数（不含两端有效行） */
        if(gap_len <= BORDER_GAP_MAX && last_valid > gap_start) {
            /* 两端锚点: last_valid (下方有效), next_valid (上方有效) */
            span  = last_valid - next_valid;
            delta = (int16)l_border[last_valid] - (int16)l_border[next_valid];
            base  = (int16)l_border[next_valid];
            for(k = next_valid + 1; k < last_valid; k++) {
                if(!l_border_valid[k]) {
                    step = k - next_valid;
                    if(delta >= 0) {
                        val = (int16)(base + (int16)((delta * step + (span >> 1)) / span));
                    } else {
                        val = (int16)(base + (int16)((delta * step - (span >> 1)) / span));
                    }
                    if(val < BORDER_MIN) val = BORDER_MIN;
                    if(val > BORDER_MAX) val = BORDER_MAX;
                    l_border[k] = (uint16)val;
                    l_border_valid[k] = 2;  /* 2 = 插值点 */
                }
            }
        }
        i = next_valid;
    }
}

/* 获取右边界线 —— 允许跳行 + 标记是否有效 + 短缺口插值 */
void get_right(uint16 total_R) {
    uint8 i = 0;
    uint16 j = 0;
    uint8 row = 0;
    uint8 last_valid = 0;
    uint8 gap_start = 0;
    uint8 next_valid = 0;
    uint8 found = 0;
    uint8 k = 0;
    uint8 gap_len = 0;
    uint8 span = 0;
    uint8 step = 0;
    int16 delta = 0;
    int16 base = 0;
    int16 val = 0;
    
    for(i = 0; i < MT9V03X_H; i++) {
        r_border[i] = BORDER_MAX;
        r_index[i] = 0;
        r_border_valid[i] = 0;
    }
    
    /* 2. 遍历所有八邻域点，按行号直接写入 */
    for(j = 0; j < total_R; j++) {
        row = (uint8)points_r[j][1];
        if(row >= MT9V03X_H) continue;
        if(r_border_valid[row] == 0) {
            r_border[row] = points_r[j][0] - 1;
            r_index[row] = j;
            r_border_valid[row] = 1;
        }
    }
    
    /* 3. 短缺口线性插值 */
    for(i = MT9V03X_H - 1; i > 0; i--) {
        if(r_border_valid[i]) { last_valid = i; break; }
    }
    
    for(i = last_valid; i > 0; ) {
        if(r_border_valid[i]) { last_valid = i; i--; continue; }
        gap_start = i;
        next_valid = 0;
        found = 0;
        for(k = i; k > 0; k--) {
            if(r_border_valid[k]) { next_valid = k; found = 1; break; }
        }
        if(!found) break;
        
        gap_len = gap_start - next_valid;
        if(gap_len <= BORDER_GAP_MAX && last_valid > gap_start) {
            span  = last_valid - next_valid;
            delta = (int16)r_border[last_valid] - (int16)r_border[next_valid];
            base  = (int16)r_border[next_valid];
            for(k = next_valid + 1; k < last_valid; k++) {
                if(!r_border_valid[k]) {
                    step = k - next_valid;
                    if(delta >= 0) {
                        val = (int16)(base + (int16)((delta * step + (span >> 1)) / span));
                    } else {
                        val = (int16)(base + (int16)((delta * step - (span >> 1)) / span));
                    }
                    if(val < BORDER_MIN) val = BORDER_MIN;
                    if(val > BORDER_MAX) val = BORDER_MAX;
                    r_border[k] = (uint16)val;
                    r_border_valid[k] = 2;
                }
            }
        }
        i = next_valid;
    }
}
/* 形态学滤波*/
// #define THRESHOLD_MAX 255*5
// #define THRESHOLD_MIN 255*2
// #define FILTER_ROI_TOP  (MT9V03X_H / 3)  /* 滤波起始行 */
// void image_filter(uint8(*bin_image)[MT9V03X_W]) {
//     uint16 i, j;
//     uint32 num = 0;
//     uint16 top = FILTER_ROI_TOP > 1 ? FILTER_ROI_TOP : 1;
//     uint8 *p_prev;
//     uint8 *p_curr;
//     uint8 *p_next;
    
//     for(i = top; i < MT9V03X_H - 1; i++) {
//         p_prev = bin_image[i-1]; 
//         p_curr = bin_image[i];   
//         p_next = bin_image[i+1];  
//         for(j = 1; j < (MT9V03X_W - 1); j++) {
//             // 统计八个方向的像素值（指针偏移）
//             num = *(p_prev+j-1) + *(p_prev+j) + *(p_prev+j+1) +
//                   *(p_curr+j-1) + *(p_curr+j+1) +
//                   *(p_next+j-1) + *(p_next+j) + *(p_next+j+1);
            
//             if(num >= THRESHOLD_MAX && *(p_curr+j) == BLACK_PIXEL) {
//                 *(p_curr+j) = WHITE_PIXEL;
//             }
//             if(num <= THRESHOLD_MIN && *(p_curr+j) == WHITE_PIXEL) {
//                 *(p_curr+j) = BLACK_PIXEL;
//             }
//         }
//     }
// }

/* 绘制图像边框 */
void image_draw_rectan(uint8(*image)[MT9V03X_W]) {
    uint8 i = 0;
    
    for(i = 0; i < MT9V03X_H; i++) {
        image[i][0] = BLACK_PIXEL;
        image[i][1] = BLACK_PIXEL;
        image[i][MT9V03X_W-1] = BLACK_PIXEL;
        image[i][MT9V03X_W-2] = BLACK_PIXEL;
    }
    
    for(i = 0; i < MT9V03X_W; i++) {
        image[0][i] = BLACK_PIXEL;
        image[1][i] = BLACK_PIXEL;
    }
}



/**
 * @brief 斑马线检测函数
 * @param row 要检测的行号
 */
void detect_zebra_line(uint8 row) {
    uint8 *p_row0;
    uint8 *p_row1;
    uint16 i;
    uint32 current_time;
    uint32 last_time;

    change_point = 0;
    
    if(row < 2 || row >= MT9V03X_H) return;
    
    /* 检查左右边界是否有效 */
    if(l_border[row] >= r_border[row] || 
       l_border[row] < BORDER_MIN || 
       r_border[row] > BORDER_MAX) {
        return;
    }
    
    /* 扫描指定行及其上一行（指针偏移） */
    p_row0 = bin_image[row];
    p_row1 = bin_image[row - 1];
    for(i = l_border[row]; i < r_border[row] - 3; i++) {
        /* 检测黑白交替变化点 */
        if((*(p_row0+i) == BLACK_PIXEL && *(p_row0+i+1) == WHITE_PIXEL) ||   /* 黑变白 */
           (*(p_row0+i) == WHITE_PIXEL && *(p_row0+i+1) == BLACK_PIXEL) ||   /* 白变黑 */
           (*(p_row1+i) == BLACK_PIXEL && *(p_row1+i+1) == WHITE_PIXEL) ||   /* 上一行黑变白 */
           (*(p_row1+i) == WHITE_PIXEL && *(p_row1+i+1) == BLACK_PIXEL)) {   /* 上一行白变黑 */
            change_point++;
        }
    }
    
    /* 如果变化点超过阈值，判定为斑马线 */
    if(change_point > 15) {
        current_time = g_sysTick_ms;
        last_time = last_zebra_time_ms;
        
        /* 如果是第一次检测到斑马线，或者距离上次检测时间超过最小间隔 */
        if(zebra_count == 0 || (current_time - last_time) > ZEBRA_MIN_INTERVAL) {
            zebra_count++;
            last_zebra_time_ms = current_time; /* 更新最后检测时间 */
            
            /* 第二次检测到斑马线时才停车 */
            if(zebra_count == 2) {
                zebra_detected = 1;
                current_track_type = ZEBRA_TRACK;
            }
        }
    } else {
        zebra_detected = 0;
    }
}

/* 状态机初始化 */
void state_machine_init(void) {
    open_speed=1800;   /*电机开环 */
    current_track_type = NORMAL_TRACK;
    current_state = SEARCH_MODE;
    lost_counter = 0;
    cross_detected = 0;
    zebra_detected = 0;
    zebra_count = 0;        
    last_zebra_time_ms = 0; /* 清零时间 */
    circle_detection_init();

}

/* 检测赛道类型 */
void detect_track_type(void) {

    /*检测是否为环岛元素 */
    if(circle2_process(l_border, r_border, bin_image)) return;

    /*检测是否为十字元素 */
    if (cross2_process(l_border, r_border, bin_image)) return;
    

    /* 3. 检查是否斑马线 */
    if(!zebra_detected) {
        /* 检测图像中下部三行 */
        detect_zebra_line(MT9V03X_H - 3);
        if(!zebra_detected) detect_zebra_line(MT9V03X_H - 5);
        if(!zebra_detected) detect_zebra_line(MT9V03X_H - 7);
        
        if(zebra_detected) {
            current_track_type = ZEBRA_TRACK;
            return;
        }
    }

}


/* 提取本帧边界（供所有后续模块使用）*/
uint8 extract_borders(void) {
    uint8 hightest = 0;
    uint8 start_row = MT9V03X_H - 2;
    uint8 min_row = MT9V03X_H / 3;
    uint8 found = 0;

    for(start_row = MT9V03X_H - 2; start_row >= min_row; start_row--) {
        if(get_start_point(start_row)) {
            found = 1;
            break;
        }
        if(start_row == 0) {
            break;
        }
    }

    if(!found) {
        return 0;
    }
    
    search_l_r((uint16)USE_NUM, bin_image, &data_stastics_l, &data_stastics_r, 
              start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
    
    get_left(data_stastics_l);
    get_right(data_stastics_r);
    
    return 1;
}

/* 根据 l_border / r_border + valid 标记计算中线 */
void calc_midline(void) {
    int last_good_mid = MT9V03X_W / 2;
    uint8 i;
    
    for(i = 0; i < MT9V03X_H; i++) {
        if(l_border_valid[i] && r_border_valid[i]) {
            midline[i] = (l_border[i] + r_border[i]) / 2;
            last_good_mid = midline[i];
        } else {
            midline[i] = last_good_mid;
        }
        leftline[i] = l_border[i];
        rightline[i] = r_border[i];
    }
    
    /* 三点均值平滑 */
    for(i = 1; i < MT9V03X_H - 1; i++) {
        if(midline[i] > 0 && midline[i-1] > 0 && midline[i+1] > 0) {
            midline[i] = (midline[i-1] + midline[i] + midline[i+1]) / 3;
        }
    }
}

/* 普通赛道处理 */
void normal_track_process(void) {
    /* 提边提到主循环中完成，直接算中线 */
    calc_midline();
    lost_counter = 0;
    current_state = TRACKING_MODE;
}

/* 主图像处理函数 */
void image_process() {
    uint8 border_ok;
    
    turn_to_bin();                  
    //image_filter(bin_image);       
    image_draw_rectan(bin_image);   

    data_stastics_l = 0;
    data_stastics_r = 0;
    
    /* ===== 第一步：统一提边（所有状态共享本帧边界）===== */
    border_ok = extract_borders();
    if(!border_ok) {
        lost_counter++;
        if(lost_counter > 5) {
            current_state = SEARCH_MODE;
        }
    }
    
    /* ===== 第二步：状态机处理（基于本帧边界）===== */
    if(border_ok) {
    circle_detection_process(l_border, r_border, bin_image);
    switch(current_state) {
        case SEARCH_MODE:
        case TRACKING_MODE:
            detect_track_type();
            
            if (current_track_type != NORMAL_TRACK) {
                current_state = SPECIAL_MODE;
            } else {
                normal_track_process();
            }
            break;
            
        case SPECIAL_MODE:
            /* 特殊元素处理 */
            switch(current_track_type) {
                case CROSS_TRACK:
                        cross2_process(l_border, r_border, bin_image);
                    break;
                case CIRCLE_TRACK:
                    circle2_process(l_border, r_border, bin_image);
                    break;
        
                case ZEBRA_TRACK:
                        if(zebra_count == 2) 
                        {
                            normal_track_process();
                            open_speed = 0;
                        }
                    break;
                    
                default:
                    normal_track_process();
                    break;
            }
            break;
    }
    } 

    update_track_error();
}
/*======================================================*/
int my_abs(int value) {
    return value >= 0 ? value : -value;
}



int16 limit_a_b(int16 x, int a, int b) {
    if(x < a) x = a;
    if(x > b) x = b;
    return x;
}

int16 limit1(int16 x, int16 y) {
    if(x > y) return y;
    else if(x < -y) return -y;
    else return x;
}