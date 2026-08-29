
#ifndef _CIRCLE_DETECTION_H_
#define _CIRCLE_DETECTION_H_

#include "zf_common_headfile.h"
#include "vision_config.h"



typedef enum {
    CIRC_NONE = 0,      /* 无圆环 */
    CIRC_START,         /* 检测到圆环 */
    CIRC_END            /* 圆环消失 */
} CircleType;

extern CircleType circle_state;
extern uint8 is_detection;

void circle_detection_init(void);
CircleType circle_detection_process(uint16 *l_border, uint16 *r_border, uint8 (*img)[MT9V03X_W]);

#endif