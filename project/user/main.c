#include "zf_common_headfile.h"
#include "main.h"
#include "camera.h"
#include "PID.h"

#define false 0
#define true 1
extern float track_error;
uint32 g_sysTick_ms = 0;

#define INCLUDE_BOUNDARY_TYPE   3   /*����ͷ������������*/
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

#define LED1                    (P33 )
#define WIFI_SSID_TEST          "SEEKFREE_2.4G"
#define WIFI_PASSWORD_TEST      "SEEKFREEV2" 
int32 j;
int32 i = MT9V03X_H - 1;
void main(void)
{
    static uint32 current_tick = 0;
    static uint32 last_10ms_tick = 0;
    static uint32 last_200ms_tick = 0;

    clock_init(SYSTEM_CLOCK_96M); 				/* ʱ�����ü�ϵͳ��ʼ��<��ر���?> */
    debug_init();    
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初�?�化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连�?(建立) WIFI 之后，是否自动连�? TCP 服务器、创�? UDP 连接、创�? TCP 服务器等操作
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开�?�?动连�? 就需要手动连接目�? IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目�? IP 的�??口建�? TCP 连接
            "TCP",                                                              // 指定使用TCP方式通�??
            WIFI_SPI_TARGET_IP,                                                 // 指定远�??的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,                                               // 指定远�??的�??口号，填写上位机的�??口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))                                               // 指定�?机的�?口号
        {
            // 如果一直建立失�? 考虑一下是不是没有接硬件�?�位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }

    // 推荐先初始化摄像头，后初始化逐�?�助�?
    while(mt9v03x_init())
    {
        system_delay_ms(100); 
		printf("\r\n mt9v03x_init error, try again.");
    }

    // 使用高�? WIFI SPI模块时无法使用屏幕（因为引脚有共�?�?
    // 逐�?�助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    pit_ms_init(TIM0_PIT, 1, sys_tick_handler);                     		
    gpio_init(IO_P33,GPO,1,GPO_PUSH_PULL); 
    uart_init(UART_2, 115200, UART2_TX, UART2_RX);
    /* ��ʼ��״̬�� */
    state_machine_init();

    while(g_sysTick_ms <=1500);                         /*������Է��밴����⣬Ȼ��Ŀǰ�������ϰ������������յģ�����ʱ����*/

    enable_control(CTRL_PIN);
    //send_pid_debug_data(PID_SPEED0, &pid_param, UART_2);
    //send_pid_debug_data(PID_SPEED1, &pid_param, UART_2);                   		
    

#if(3 == INCLUDE_BOUNDARY_TYPE)
    j=0;
#endif

    /* ��ʼ������ͷ */
    while(1)
    {
        if(mt9v03x_init())
            gpio_toggle_level(LED1);                                            /* ��ת LED ����������? ���� LED ���� ��ʼ����������ƻ����ĺ���? */
        else
            break;
        system_delay_ms(500);                                                  /* ���Ʊ�ʾ�쳣 */
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
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
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
            seekfree_assistant_camera_send();
            image_process();
            send_error_data(track_error, 0.0f, UART_2);
        }

    }
}

void sys_tick_handler (void){
    g_sysTick_ms += 1;                                                                 /* ϵͳ���� */      
}

