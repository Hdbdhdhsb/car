#include "circle_detection.h"

CircleType circle_state = CIRC_NONE;
uint8 is_detection = 0;

/* 圆环检测行(属派生, 保留在实现里; CIRCLE_MIN_* 调参已集中到 config.h) */
#define detect_row      (MT9V03X_H / 2)

static uint8 circle_row_test(uint16 *l_border, uint16 *r_border, uint8 (*img)[MT9V03X_W])
{
    uint16 row = detect_row;
    uint16 scan_left = 1;
    uint16 scan_right = MT9V03X_W - 2;
    uint16 col = 0;
    uint8 black_segment_cnt = 0;
    uint8 in_black = 0;
    uint16 black_width = 0;

    
    if(l_border != NULL && r_border != NULL) {
        if(l_border[row] > 0 && l_border[row] < MT9V03X_W) {
            scan_left = l_border[row];
        }
        if(r_border[row] > 0 && r_border[row] < MT9V03X_W) {
            scan_right = r_border[row];
        }
    }

    if(scan_left < 1) {
        scan_left = 1;
    }
    if(scan_right > MT9V03X_W - 2) {
        scan_right = MT9V03X_W - 2;
    }
    if(scan_left >= scan_right) {
        scan_left = 1;
        scan_right = MT9V03X_W - 2;
    }

    for(col = scan_left; col <= scan_right; col++) {
        if(img[row][col] == 0) {
            if(!in_black) {
                in_black = 1;
                black_width = 1;
            } else {
                black_width++;
            }
        } else if(in_black) {
            if(black_width >= CIRCLE_MIN_LINE_WIDTH) {
                black_segment_cnt++;
                if(black_segment_cnt >= CIRCLE_MIN_BLACK_SEGMENTS) {
                    return 1;
                }
            }
            in_black = 0;
            black_width = 0;
        }
    }

    if(in_black && black_width >= CIRCLE_MIN_LINE_WIDTH) {
        black_segment_cnt++;
    }

    return (black_segment_cnt >= CIRCLE_MIN_BLACK_SEGMENTS) ? 1 : 0;
}

void circle_detection_init(void)
{
    circle_state = CIRC_NONE;
    is_detection = 0;
}

CircleType circle_detection_process(uint16 *l_border, uint16 *r_border, uint8 (*img)[MT9V03X_W])
{
    uint8 has_circle = circle_row_test(l_border, r_border, img);

    if(has_circle) {
        circle_state = CIRC_START;
        is_detection = 0;
    } else {
        if(circle_state == CIRC_START) {
            circle_state = CIRC_END;
            is_detection = 1;
        } else {
            circle_state = CIRC_NONE;
        }
    }

    return circle_state;
}