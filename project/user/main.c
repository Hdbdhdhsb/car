#include "zf_common_headfile.h"
#include "main.h"
#include "camera.h"
#include "PID.h"
#include "vision_config.h"
#include <string.h>
#include <stdio.h>

#define false 0
#define true 1

extern float track_error;
extern volatile uint8 track_lost_stop;   /* 1 = 冲出赛道视野 -> 停车 */
static uint8 track_stop_pid_reset = 0;   /* 停车时只清一次 PID，防积分残留 */
uint32 g_sysTick_ms = 0;
static pid_ctrl_block_handle_t steer_pid = NULL;   /* 转向 PID(位置式)，main() 建立 */

#define BOUNDARY_NUM            MT9V03X_H
#define ASSISTANT_SEND_DIV      3U
#define ASSISTANT_SEND_IMAGE    1U
#define SERVO_TEST 0   /* 1=servo test via control frames, 0=normal */

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

#define LED1                    (P33)
#define WIFI_SSID_TEST          "parafume"
#define WIFI_PASSWORD_TEST      "39lym520"

int32 j;
int32 i = MT9V03X_H - 1;

static char  wifi_cmd_line[WIFI_CMD_LINE_SIZE];
static uint8 wifi_cmd_line_len = 0;
static uint8 wifi_ctrl_running = false;
static uint32 last_wifi_ctrl_tick = 0;
static uint32 last_ctrl_send_tick = 0;
static float  last_turn = 0.0f;   /* 最近一次转向输出，供 50ms 保活帧复用(避免与 PID 帧打架) */

/* 现场调速：跑态把这两个值（千分比占空比 0..1000）发给下位机，SPD 命令可改 */
static float  g_run_speed0 = LOWER_START_SPEED0;
static float  g_run_speed1 = LOWER_START_SPEED1;

//更新图像边界
static void update_camera_boundary_overlay(void)
{
    uint8 row;

    for(row = 0; row < MT9V03X_H; row++)
    {
        xy_x1_boundary[row] = (uint8)limit_a_b((int16)leftline[row], 0, MT9V03X_W - 1);
        xy_x2_boundary[row] = (uint8)limit_a_b((int16)midline[row], 0, MT9V03X_W - 1);
        xy_x3_boundary[row] = (uint8)limit_a_b((int16)rightline[row], 0, MT9V03X_W - 1);
    }
}

/* Send a short text response so the desktop side can see command handling state. */
static void wifi_ctrl_send_ack(const char *message)
{
    wifi_spi_send_string(message);
    wifi_spi_send_string("\r\n");
}

static void send_lower_start_frame(void)
{
    send_ctrl_data(g_run_speed0, g_run_speed1, LOWER_START_TURN, LOWER_CTRL_UART);
}

static void send_lower_stop_frame(void)
{
    send_ctrl_data(LOWER_STOP_SPEED0, LOWER_STOP_SPEED1, LOWER_STOP_TURN, LOWER_CTRL_UART);
}

/* 极简 float 解析：支持 "400" / "400.5" / "-400"，前后可有空格，尾部有垃圾则判非法 */
static uint8 parse_float_simple(const char *p, float *out)
{
    float v = 0.0f;
    float div = 1.0f;
    uint8 neg = 0;
    uint8 digits = 0;

    while(' ' == *p) { p++; }
    if('-' == *p) { neg = 1; p++; }
    while(*p >= '0' && *p <= '9') { v = v * 10.0f + (float)(*p - '0'); p++; digits++; }
    if('.' == *p)
    {
        p++;
        while(*p >= '0' && *p <= '9') { div *= 10.0f; v += (float)(*p - '0') / div; p++; digits++; }
    }
    if(0 == digits) { return 0; }
    while(' ' == *p) { p++; }
    if('\0' != *p) { return 0; }
    *out = neg ? -v : v;
    return 1;
}

static void handle_wifi_ctrl_command(const char *command)
{
    if(0 == strcmp(command, "START"))
    {
        last_wifi_ctrl_tick = g_sysTick_ms;

        /* The GUI sends START periodically as a heartbeat. Only ack the state change once. */
        if(!wifi_ctrl_running)
        {
            wifi_ctrl_running = true;
            send_lower_start_frame();
            last_ctrl_send_tick = g_sysTick_ms;
            if(steer_pid) pid_reset_ctrl_block(steer_pid);   /* 起跑清零，防积分残留 */
            wifi_ctrl_send_ack("ACK START");
        }
    }
    else if(0 == strcmp(command, "STOP"))
    {
        wifi_ctrl_running = false;
        send_lower_stop_frame();
        if(steer_pid) pid_reset_ctrl_block(steer_pid);   /* 停车清零，防积分残留 */
        last_turn = 0.0f;
        wifi_ctrl_send_ack("ACK STOP");
    }
    else if(0 == strcmp(command, "PING"))
    {
        wifi_ctrl_send_ack("PONG");
    }
    else if(0 == strncmp(command, "SPD", 3))
    {
        /* SPD <千分比占空比 0..1000>  现场调速，不用重烧
         * SPD      -> 回读当前值
         * SPD 400  -> 两轮一起设成 40% */
        const char *arg = command + 3;
        float v = 0.0f;
        char  ack_buf[32];

        while(' ' == *arg) { arg++; }

        if('\0' == *arg)
        {
            sprintf(ack_buf, "ACK SPD %.0f", (double)g_run_speed0);
            wifi_ctrl_send_ack(ack_buf);
        }
        else if(parse_float_simple(arg, &v))
        {
            if(v < 0.0f)    { v = 0.0f;    }
            if(v > 1000.0f) { v = 1000.0f; }
            g_run_speed0 = v;
            g_run_speed1 = v;
            sprintf(ack_buf, "ACK SPD %.0f", (double)v);
            wifi_ctrl_send_ack(ack_buf);
        }
        else
        {
            wifi_ctrl_send_ack("ERR SPD ARG");
        }
    }
    else if('\0' != command[0])
    {
        wifi_ctrl_send_ack("ERR UNKNOWN CMD");
    }
}

/* Read line-based commands from the existing WiFi TCP link. */
static void poll_wifi_ctrl_command(void)
{
    uint8 rx_buffer[WIFI_CMD_RX_BUF_SIZE];
    uint32 rx_length = wifi_spi_read_buffer(rx_buffer, WIFI_CMD_RX_BUF_SIZE);
    uint32 index = 0;

    for(index = 0; index < rx_length; index++)
    {
        uint8 ch = rx_buffer[index];

        if('\r' == ch)
        {
            continue;
        }

        if('\n' == ch)
        {
            wifi_cmd_line[wifi_cmd_line_len] = '\0';
            handle_wifi_ctrl_command(wifi_cmd_line);
            wifi_cmd_line_len = 0;
            continue;
        }

        if(wifi_cmd_line_len < (WIFI_CMD_LINE_SIZE - 1U))
        {
            wifi_cmd_line[wifi_cmd_line_len++] = (char)ch;
        }
        else
        {
            wifi_cmd_line_len = 0;
            wifi_ctrl_send_ack("ERR CMD TOO LONG");
        }
    }
}

/*
 * Keep sending control frames while the GUI is in run mode.
 * If the GUI or WiFi link disappears, the heartbeat times out and the car is stopped.
 */
static void service_wifi_ctrl_loop(uint32 current_tick)
{
    /* ── 停车态：不再直接 return，而是每 50ms 持续发一帧“停”。
     *   下位机只认“最新一帧”，若停车只发一帧，会被主循环里
     *   持续发送的调试帧(send_error_data)覆盖 → 车停不住。
     *   让“停”和“跑”对称地持续发，下位机收到的最新帧才稳定是“停”。 */
    if(!wifi_ctrl_running)
    {
        if((current_tick - last_ctrl_send_tick) >= WIFI_CTRL_SEND_PERIOD_MS)
        {
            last_ctrl_send_tick = current_tick;
            send_lower_stop_frame();   /* 持续发停帧，刷新下位机的“最新指令” */
        }
        return;
    }

    if((current_tick - last_wifi_ctrl_tick) > WIFI_CTRL_COMMAND_TIMEOUT_MS)
    {
        wifi_ctrl_running = false;
        last_ctrl_send_tick = current_tick;   /* 超时后交给上面的停车态按周期续发停帧 */
        send_lower_stop_frame();
        if(steer_pid) pid_reset_ctrl_block(steer_pid);   /* 超时停车清零，防积分残留 */
        last_turn = 0.0f;
        wifi_ctrl_send_ack("ACK TIMEOUT STOP");
        return;
    }

    if((current_tick - last_ctrl_send_tick) >= WIFI_CTRL_SEND_PERIOD_MS)
    {
        last_ctrl_send_tick = current_tick;
        /* 用同一个状态变量分流：跑 → 控制帧；停 → 停帧 */
        if(wifi_ctrl_running && !track_lost_stop)
        {
            /* ★★实验：跑态停发控制帧，让下位机稳定在“误差帧/闭环”模式。
             *   控制帧的 target_speed=PID_TARGET_SPEED=5.0 会被下位机当占空比(5/1000=0.5%)
             *   直接写电机 → 与控制帧交叉到来时把电机瞬间拉停 → 一卡一卡/忽快忽停。
             *   验证：本行 + 摄像头块内那行同时注释掉。 */
            send_ctrl_data(g_run_speed0, g_run_speed1, PID_STEER_SIGN * last_turn, LOWER_CTRL_UART);
        }
        else
        {
            send_lower_stop_frame();
        }
    }
}

void main(void)
{
    static uint32 current_tick = 0;
    static uint32 last_10ms_tick = 0;
    static uint32 last_200ms_tick = 0;
    static uint8 assistant_send_divider = 0;
    static uint32 assistant_retry_after_tick = 0;

    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

    if(1 != WIFI_SPI_AUTO_CONNECT)
    {
        while(wifi_spi_socket_connect(
            "TCP",
            WIFI_SPI_TARGET_IP,
            WIFI_SPI_TARGET_PORT,
            WIFI_SPI_LOCAL_PORT))
        {
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);
        }
    }

    while(mt9v03x_init())
    {
        system_delay_ms(100);
        printf("\r\n mt9v03x_init error, try again.");
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    pit_ms_init(TIM0_PIT, 1, sys_tick_handler);
    uart_init(DEBUG_DATA_UART, 115200, UART4_TX, UART4_RX);
    state_machine_init();//状态机初始化

    /* Boot into a known-safe state so the lower controller does not keep stale motion commands. */
    send_lower_stop_frame();

    /* 建立转向位置式 PID(N 个控制块之一)。参数为经验初值，需上赛道实调(见 vision_config.h)。 */
    {
        pid_ctrl_parameter_t steer_cfg = {
            PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
            PID_OUTPUT_MAX, -PID_OUTPUT_MAX,
            PID_INTEGRAL_MAX, -PID_INTEGRAL_MAX,
            PID_CAL_TYPE_POSITIONAL
        };
        pid_new_control_block(&steer_cfg, &steer_pid);
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        ASSISTANT_SEND_IMAGE ? mt9v03x_image[0] : NULL,
        MT9V03X_W,
        MT9V03X_H);
    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY,
        BOUNDARY_NUM,
        xy_x1_boundary,
        xy_x2_boundary,
        xy_x3_boundary,
        NULL,
        NULL,
        NULL);
        enable_control(CTRL_PIN); /* 使能下位机 */
    while(1)
    {

        poll_wifi_ctrl_command();
        current_tick = g_sysTick_ms;
        service_wifi_ctrl_loop(current_tick);

        if((current_tick - last_10ms_tick) >= 10)
        {
            last_10ms_tick = current_tick;
        }

        if((current_tick - last_200ms_tick) >= 200)
        {
            last_200ms_tick = current_tick;
            //gpio_toggle_level(IO_P33);
        }

        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            image_process();
            assistant_send_divider++;
            if(assistant_send_divider >= ASSISTANT_SEND_DIV)
            {
                assistant_send_divider = 0;
                if(current_tick >= assistant_retry_after_tick)
                {
                    update_camera_boundary_overlay();
                    if(!seekfree_assistant_camera_send())
                    {
                        assistant_retry_after_tick = current_tick + 500;
                    }
                }
            }
            
            /* 转向闭环：track_error 经位置式 PID 得转向角；车速先开环常量。
             * 经验初值(见 vision_config.h)，转向符号/量级需在断电/架车状态实测后再上赛道。 */
            if(wifi_ctrl_running && steer_pid != NULL && !track_lost_stop)
            {
                float turn = 0.0f;
                track_stop_pid_reset = 0;
                if(pid_compute(steer_pid, track_error, &turn) == PID_OK)
                {
                    last_turn = turn;
                    /* ★★实验：与 service_wifi_ctrl_loop 同步关闭控制帧 */
                    send_ctrl_data(g_run_speed0, g_run_speed1, PID_STEER_SIGN * turn, LOWER_CTRL_UART);
                }
            }
            else if(wifi_ctrl_running && track_lost_stop)
            {
                /* 冲出赛道视野(下方全黑) -> 停车，不再发控制帧 */
                if(!track_stop_pid_reset)
                {
                    track_stop_pid_reset = 1;
                    if(steer_pid) pid_reset_ctrl_block(steer_pid);
                    last_turn = 0.0f;
                }
                send_lower_stop_frame();
            }

            /* 调试帧（误差遥测）只在“跑”态发；停车态若不门控，
             * 它会和控制帧/停帧共用 UART_4，且发送频率更高，
             * 会把“停”覆盖掉 → 下位机继续跑。 */
            if(wifi_ctrl_running && !track_lost_stop)
            {
                send_error_data(PID_STEER_SIGN * last_turn, g_run_speed0, DEBUG_DATA_UART);
            }
        }
    }
}

void sys_tick_handler(void)
{
    g_sysTick_ms += 1;
}

