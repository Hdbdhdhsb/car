#include "zf_common_headfile.h"
#include "main.h"
#include "camera.h"
#include "PID.h"

#define false 0
#define true 1
extern float track_error;

#define INCLUDE_BOUNDARY_TYPE   3   /*摄像头返回数据类型*/
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

/* 图像备份数组，在发送前将图像备份再进行发送，避免图像出现撕裂的问题 */
uint8 image_copy[MT9V03X_H][MT9V03X_W];

#define LED1                    (P33 )
int32 j;
int32 i = MT9V03X_H - 1;
void main(void)
{
    static uint32 current_tick = 0;
    static uint32 last_10ms_tick = 0;
    static uint32 last_200ms_tick = 0;

    clock_init(SYSTEM_CLOCK_96M); 				/* 时钟配置及系统初始化<务必保留> */
    debug_init();    
    pit_ms_init(TIM0_PIT, 1, sys_tick_handler);                     		
    gpio_init(IO_P33,GPO,1,GPO_PUSH_PULL); 
    uart_init(UART_2, 115200, UART2_TX, UART2_RX);
    /* 初始化状态机 */
    state_machine_init();

    while(g_sysTick_ms <=1500);                         /*这里可以放入按键检测，然而目前开发板上按键引脚是悬空的，故延时处理*/

    enable_control(CTRL_PIN);
    send_pid_debug_data(PID_SPEED0, &pid_param, UART_2);
    send_pid_debug_data(PID_SPEED1, &pid_param, UART_2);                   		
    
    /* 设置逐飞助手使用DEBUG串口进行收发 */
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

#if(3 == INCLUDE_BOUNDARY_TYPE)
    j=0;
#endif

    /* 初始化摄像头 */
    while(1)
    {
        if(mt9v03x_init())
            gpio_toggle_level(LED1);                                            /* 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢 */
        else
            break;
        system_delay_ms(500);                                                  /* 闪灯表示异常 */
    }


#if(3 == INCLUDE_BOUNDARY_TYPE)
    j = 0;
	 
    for(i = MT9V03X_H - 1; i >= MT9V03X_H / 2; i--)
    {
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = (uint8)i;

        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = (uint8)i;
        j++;
    }

    for(i = MT9V03X_H / 2 - 1; i >= 0; i--)
    {
        xy_x1_boundary[j] = 34 + (MT9V03X_H / 2 - (uint8)i) * (MT9V03X_W / 2 - 34) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = 47 + (MT9V03X_H / 2 - (uint8)i) * (MT9V03X_W / 2 - 47) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + (uint8)i * 3 / 4;

        xy_x3_boundary[j] = 60 + (MT9V03X_H / 2 - (uint8)i) * (MT9V03X_W / 2 - 60) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + (uint8)i / 2;
        j++;
    }

    for(i = 0; i < MT9V03X_H / 2; i++)
    {
        xy_x1_boundary[j] = MT9V03X_W / 2 + (uint8)i * (138 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = MT9V03X_W / 2 + i * (133 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + (uint8)i * 3 / 4;

        xy_x3_boundary[j] = MT9V03X_W / 2 + i * (128 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + (uint8)i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

#endif
    while(1)
    {
         current_tick = g_sysTick_ms;

        if (current_tick - last_10ms_tick >= 10) {
            last_10ms_tick = current_tick;
            
        }

        if((current_tick - last_200ms_tick) >= 200){
            last_200ms_tick = current_tick; 
            gpio_toggle_level(IO_P33); 
        }

        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        
            seekfree_assistant_camera_send();
            image_process();
            send_error_data(track_error, 0.0f, UART_2);
        }

    }
}

void sys_tick_handler (void){
    g_sysTick_ms += 1;                                                                 /* 系统心跳 */      
}

