#include "zf_common_headfile.h"
#include "main.h"
#include "camera.h"
#include "PID.h"
#include <string.h>

#define false 0
#define true 1

extern float track_error;
uint32 g_sysTick_ms = 0;

#define BOUNDARY_NUM            MT9V03X_H
#define ASSISTANT_SEND_DIV      3U
#define ASSISTANT_SEND_IMAGE    1U

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

#define LED1                    (P33)
#define WIFI_SSID_TEST          "y"
#define WIFI_PASSWORD_TEST      "ksh12345"

int32 j;
int32 i = MT9V03X_H - 1;

static char  wifi_cmd_line[WIFI_CMD_LINE_SIZE];
static uint8 wifi_cmd_line_len = 0;
static uint8 wifi_ctrl_running = false;
static uint32 last_wifi_ctrl_tick = 0;
static uint32 last_ctrl_send_tick = 0;

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
    send_ctrl_data(LOWER_START_SPEED0, LOWER_START_SPEED1, LOWER_START_TURN, LOWER_CTRL_UART);
}

static void send_lower_stop_frame(void)
{
    send_ctrl_data(LOWER_STOP_SPEED0, LOWER_STOP_SPEED1, LOWER_STOP_TURN, LOWER_CTRL_UART);
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
            wifi_ctrl_send_ack("ACK START");
        }
    }
    else if(0 == strcmp(command, "STOP"))
    {
        wifi_ctrl_running = false;
        send_lower_stop_frame();
        wifi_ctrl_send_ack("ACK STOP");
    }
    else if(0 == strcmp(command, "PING"))
    {
        wifi_ctrl_send_ack("PONG");
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
    if(!wifi_ctrl_running)
    {
        return;
    }

    if((current_tick - last_wifi_ctrl_tick) > WIFI_CTRL_COMMAND_TIMEOUT_MS)
    {
        wifi_ctrl_running = false;
        send_lower_stop_frame();
        wifi_ctrl_send_ack("ACK TIMEOUT STOP");
        return;
    }

    if((current_tick - last_ctrl_send_tick) >= WIFI_CTRL_SEND_PERIOD_MS)
    {
        last_ctrl_send_tick = current_tick;
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
    state_machine_init();

    /* Boot into a known-safe state so the lower controller does not keep stale motion commands. */
    send_lower_stop_frame();

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
            if(wifi_ctrl_running)
            {
                send_error_data(track_error, 5.0f, DEBUG_DATA_UART);
            }
           
        }
    }
}

void sys_tick_handler(void)
{
    g_sysTick_ms += 1;
}
