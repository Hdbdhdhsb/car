/*********************************************************************************************************************
* STC32G144K Opensourec Library ����STC32G144K ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2025 SEEKFREE ��ɿƼ�
*
* ���ļ���STC32G144K��Դ���һ����
*
* STC32G144K ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MDK FOR C251
* ����ƽ̨          STC32G144K
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����           ��ע
* 2025-11-20        ��W            first version
********************************************************************************************************************/

#ifndef _zf_device_mt9v03x_h
#define _zf_device_mt9v03x_h


#include "zf_common_debug.h"
#include "zf_common_clock.h"



// ��֧��IIC����
#define MT9V03X_COF_IIC_DELAY   ( 50 )                              // ����� IIC ��ʱ
#define MT9V03X_COF_IIC_SCL     ( IO_PB6 )                          // ����� IIC-SCL ���� ,IICƵ�ʽ������50Khz
#define MT9V03X_COF_IIC_SDA     ( IO_PB7 )                          // ����� IIC-SDA ���� ,IICƵ�ʽ������50Khz

// ������������ţ���ֹ�޸�
#define MT9V03X_D0_PIN          ( IO_P20 )                          // D0��ӦP20
#define MT9V03X_D1_PIN          ( IO_P21 )                          // D1��ӦP21
#define MT9V03X_D2_PIN          ( IO_P22 )                          // D2��ӦP22
#define MT9V03X_D3_PIN          ( IO_P23 )                          // D3��ӦP23
#define MT9V03X_D4_PIN          ( IO_P24 )                          // D4��ӦP24
#define MT9V03X_D5_PIN          ( IO_P25 )                          // D5��ӦP25
#define MT9V03X_D6_PIN          ( IO_P26 )                          // D6��ӦP26
#define MT9V03X_D7_PIN          ( IO_P27 )                          // D7��ӦP27

#define MT9V03X_INIT_TIMEOUT    ( 400 )                          	// Ĭ�ϵ�����ͷ��ʼ����ʱʱ�� ����Ϊ��λ

//--------------------------------------------------------------------------------------------------
// ����ͷĬ�ϲ������� �ڴ��޸�����ͷ����
//--------------------------------------------------------------------------------------------------
#define MT9V03X_W               ( 120 )                           	// ͼ�����   �������޸ģ�Ĭ��ֵ188
#define MT9V03X_H               ( 60 )                           	// ͼ��߶�   �������޸ģ�Ĭ��ֵ120
#define MT9V03X_IMAGE_SIZE      ( MT9V03X_W * MT9V03X_H )           // ����ͼ���С���ܳ��� 65535

#define MT9V03X_AUTO_EXP_DEF    ( 0  )                           	// �Զ��ع�����     Ĭ�ϲ������Զ��ع�����  ��Χ [0-63] 0Ϊ�ر�
																	//                  ����Զ��ع⿪��  EXP_TIME���������Զ��ع�ʱ�������
																	//                  һ������ǲ���Ҫ�����Զ��ع����� ����������߷ǳ������ȵ�������Գ��������Զ��ع⣬����ͼ���ȶ���
#define MT9V03X_EXP_TIME_DEF    ( 512)                           	// �ع�ʱ��         ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ
#define MT9V03X_FPS_DEF         ( 100 )                           	// ͼ��֡��         ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS
#define MT9V03X_LR_OFFSET_DEF   ( 0  )                           	// ͼ������ƫ����   ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ188 376 752ʱ�޷�����ƫ��
																	//                  ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
#define MT9V03X_UD_OFFSET_DEF   ( 0  )                           	// ͼ������ƫ����   ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ120 240 480ʱ�޷�����ƫ��
																	//                  ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
#define MT9V03X_GAIN_DEF        ( 32 )                           	// ͼ������         ��Χ [16-64]  ����������ع�ʱ��̶�������¸ı�ͼ�������̶�

                                                

// ����ͷ����ö��
typedef enum
{
    MT9V03X_INIT = 0,                              				    // ����ͷ��ʼ������
    MT9V03X_AUTO_EXP = 1,                              		        // �Զ��ع�����
    MT9V03X_EXP_TIME = 2,                              		        // �ع�ʱ������
    MT9V03X_FPS = 3,                                   		        // ����ͷ֡������
    MT9V03X_SET_COL = 4,                               		        // ͼ��������
    MT9V03X_SET_ROW = 5,                               		        // ͼ��������
    MT9V03X_LR_OFFSET = 6,                             		        // ͼ������ƫ������
    MT9V03X_UD_OFFSET = 7,                             		        // ͼ������ƫ������
    MT9V03X_GAIN = 8,                                  		        // ͼ��ƫ������
    MT9V03X_PCLK_MODE = 9,                             		        // ����ʱ��ģʽ����(�������MT9V034 V1.5�Լ����ϰ汾֧�ָ�����)
    MT9V03X_CONFIG_FINISH = 10,                         	        // ������λ����Ҫ����ռλ����
						
    MT9V03X_COLOR_GET_WHO_AM_I = 0xEF,						
    MT9V03X_SET_EXP_TIME = 0XF0,                   				    // ���������ع�ʱ������
    MT9V03X_GET_STATUS,                            				    // ��ȡ����ͷ��������
    MT9V03X_GET_VERSION,                           				    // �̼��汾������
                            
    MT9V03X_SET_ADDR = 0XFE,                       				    // �Ĵ�����ַ����
    MT9V03X_SET_DATA                               				    // �Ĵ�����������
}m9v03x_cmd_enum;


// ����ͷ�ӿ�����ö��
typedef enum
{
    MT9V03X_UART,
    MT9V03X_SCCB,
}mt9v03x_type_enum;

extern vuint8 mt9v03x_finish_flag;
extern uint8  mt9v03x_image[MT9V03X_H][MT9V03X_W];


void    mt9v03x_vsync_handler(void);
void    mt9v03x_dma_handler(void);

uint8   mt9v03x_set_exposure_time  (uint16 light);
uint8   mt9v03x_set_reg            (uint8 addr, uint16 dat);
uint8   mt9v03x_init               (void);

#endif
