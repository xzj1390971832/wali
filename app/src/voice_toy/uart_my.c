
#include "app_config.h"
#include "includes.h"
#include "asm/power_interface.h"
#include "asm/power/p33.h"
#include "cpu.h"
#include "uart_dev.h"
#include "toy_music.h"
//
#include "uart_my.h"
#include "user_config.h"
#include "user_motor.h"

#include "vm_api.h"
#define LOG_TAG_CONST       UTD
#define LOG_TAG             "[uart my]"
#include "log.h"



#if 1
#include "typedef.h"
#define DMA_BUF_LEN2			64


static u8 tmp_buf[DMA_BUF_LEN2] = {0};
static u8 usr_uart_buf[DMA_BUF_LEN2] __attribute__((aligned(4)));
static const uart_bus_t *ut0 = NULL;
static const uart_bus_t *ut1 = NULL;
extern const uart_bus_t *uart_dev_open(const struct uart_platform_data_t *arg);
void usr_uart_recieve(u8*data,u16 len);


//
const uart_bus_t *uart_dev_open(const struct uart_platform_data_t *arg);
u32 uart_dev_close(uart_bus_t *ut);
extern const  uart_bus_t *app_uart;
extern u8 power_on_flow;
//----------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------//
static u8 uart_rxbuf[64] __attribute__((aligned(4)));
static u8 uart_txbuf[16] __attribute__((aligned(4))) = {0xAA,0x55,0,0,0,0};
static u8 u1_rx_buff[64] __attribute__((aligned(4))),u1_rx_cnt=0;
//----------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------//
// const u8 u1_cmd_table[][3]=
// {
//     0xAA	,0x0	,0x0,	  //待机：
//     0xAA	,0x1	,0x0,	  //前进1：这里2个指令，都循环播同一个声
//     0xAA	,0x8	,0x0,	  //前进2：
//     0xAA	,0x2	,0x0,	  //后退1：这里2个指令，都循环播同一个声
//     0xAA	,0x4	,0x0,	  //后退2：
//     0xAA	,0x80	,0x20,	  //暂停/恢复：
//     0xAA	,0x80	,0x00,	  //暂停/恢复：

// };
typedef enum
{
    //要和u1_cmd_table对应上
    STA_stanby=0,
    STA_forward_1,
    STA_forward_2,
    STA_back_1,
    STA_back_2,
    STA_pause_resume_1,
    STA_pause_resume_2,

    STA_stop=0xfe,
    STA_on=0xff,

}T_STA;
//----------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------//
#define  S_one_Package_len  64  //
#define  S_wait_check_time  20  //time_base: 10ms  * 20 == 200ms
#define  S_stop_check_time  100  //time_base: 10ms  * 20 == 200ms
//
u8 cmd_buf[16][4] = {0,0,0,0};//数据串，截取，转信号包
u8 work_sta=1;
u8 standby_cb;
u8 stop_check_time;
u8 uart1_cp_ok_flag;

static uart_bus_t uart1;

u32 uart1_recv_len = 0;  // 保存接收数据的长度
static u8 last_buf[64] = {0};  // 存储上一次接收的数据
static u32 last_len = 0;        // 存储上一次数据长度
extern void uart_set_dma_dir(u32 rx_en);
// 判断当前数据与上一次是否不同
int data_changed = 0;


T_STA status_mode=STA_on,last_mode=STA_on;

//初始状态为停止
volatile RemoteControlStatus remote_control_status = receive_remove_control_idle;
//----------------------------------------------------------------------------------------------------//
//2.4G数据
//----------------------------------------------------------------------------------------------------//
//new
#define receive_remove_control_left_forward         0x01
#define receive_remove_control_forward              0x02
#define receive_remove_control_right_forward        0x03
#define receive_remove_control_head_left            0x04
#define receive_remove_control_back                 0x05
#define receive_remove_control_head_right           0x06
#define receive_remove_control_program              0x07
#define receive_remove_control_demo                 0x08
#define receive_remove_control_stop                 0x09


#define receive_remove_control_distance_switch      0x0C
#define receive_remove_control_angle_switch         0x0D

#define receive_remove_control_idle                 0x00
// #define receive_remove_control_left_back            0x03
// #define receive_remove_control_right_back           0x06
// #define receive_remove_control_forward_20           0x08
// #define receive_remove_control_back_20              0x0B
// #define receive_remove_control_head_reset           0x07

//----------------------------------------------------------------------------------------------------//
//AI数据
//----------------------------------------------------------------------------------------------------//
#define receive_ai_forward                        0x01  // 前进
#define receive_ai_back                           0x02  // 后退
#define receive_ai_left                           0x03  // 左转
#define receive_ai_right                          0x04  // 右转
//----------------------------------------------------------------------------------------------------//
//蓝牙遥控
//----------------------------------------------------------------------------------------------------//
#define receive_ble_left_up                        0x0B  // 左上
#define receive_ble_left_down                      0x0C  // 左下
#define receive_ble_left_left                      0x17  // 左左
#define receive_ble_left_right                     0x06  // 左右
#define receive_ble_left_mid                       0x0A  // 左中

#define receive_ble_stop                           0x00  // 停止

#define receive_ble_right_up                       0x1A  // 右上（注意：与左上指令值相同，需结合hand_type区分）
#define receive_ble_right_down                     0x02  // 右下
#define receive_ble_right_left                     0x09  // 右左
#define receive_ble_right_right                    0x19  // 右右
#define receive_ble_right_mid                      0x08  // 右中
//----------------------------------------------------------------------------------------------------//
static void usr_uart_isr_hook(void *arg, u32 status)
{
    const uart_bus_t *ubus = arg;

    if (status == UT_RX_OT) {
        u32 len = ubus->read(tmp_buf, 64, 0);
        if (len != 0) {
            usr_uart_recieve(tmp_buf,len); //打印接收到的数据
            uart1_cp_ok_flag =  1;
            //将接收长度保存到全局变量
            uart1_recv_len = len;
            // 将接收到的数据再次发送出去
            // UT1_write_buf(tmp_buf, len);                                                

        }
    }
}


static int uart_dev_init(const uart_bus_t *ut)
{
    memset((void *)usr_uart_buf, 0, sizeof(usr_uart_buf));
    struct uart_platform_data_t arg;
    arg.tx_pin = IO_PORTA_05;
    arg.rx_pin = IO_PORTA_07;       
    arg.rx_cbuf = usr_uart_buf;
    arg.rx_cbuf_size = 64;//
    arg.frame_length = 64;//
    arg.rx_timeout = 5;
    arg.isr_cbfun = usr_uart_isr_hook;
    arg.argv = JL_UT1;
    arg.is_9bit = 0;
    arg.baud = 2400;
    ut = uart_dev_open(&arg);
    if (NULL != ut) {
        log_info("uart1 success");
        return 0;
    } else {
        return -1;
    }
}



void usr_uart_init(void)
{
    if (0 != uart_dev_init(ut0)) {
        log_info("######uart1_usr init fail!\n");
        return;
    }
}

void usr_uart_recieve(u8*data,u16 len)///串口1接收回调函数
{
    log_info_hexdump(data, len);
}

void usr_uart_24data_process(u8*data)
{
    // 一、检查帧头是否符合要求
    if (data[0] != 0xC3 || data[1] != 0x03) {
        return; // 帧头错误
    }

    // 二、检查指令字节是否匹配
    if (data[2] != data[3]) {
        return; // 指令字节不匹配
    }

    // 三、检查帧尾
    if (data[4] != 0xAA) {
        return; // 帧尾错误
    }

     // 四、根据指令字节设置标志位
    switch (data[2]) {
    case receive_remove_control_left_forward: 
        //前左转
        remote_control_status = remote_control_left_forward;
        // log_info("left forward\n");
        break;

    case receive_remove_control_forward: 
        //前进10cm
        remote_control_status = remote_control_forward;
        // log_info("forward\n");
        break;

    case receive_remove_control_right_forward: 
        //前右转
        remote_control_status = remote_control_right_forward;
        // log_info("right forward\n");
        break;

    case receive_remove_control_head_left: 
        //头部左转
        remote_control_status = remote_control_head_left;
        // log_info("head left\n");
        break;

    case receive_remove_control_back: 
        //后退
        remote_control_status = remote_control_back;
        // log_info("back\n");
        break;

    case receive_remove_control_head_right: 
        //头部右转
        remote_control_status = remote_control_head_right;
        break;

    case receive_remove_control_program: 
        remote_control_status = remote_control_program;
        break;

    case receive_remove_control_demo: 
        remote_control_status = remote_control_demo;
        break;

    case receive_remove_control_stop: 
        //停止
        remote_control_status = remote_control_head_reset;
        
        break;

    case receive_remove_control_distance_switch: 
        //切换距离
        remote_control_status = remote_control_distance_switch;
        break;

    case receive_remove_control_angle_switch: 
        //切换角度
        remote_control_status = remote_control_angle_switch;
        break;

    // case receive_remove_control_head_reset: 
    //     //头部复位
    //     remote_control_status = remote_control_head_reset;
    //     // log_info("head reset\n");
    //     break;

    default:
        remote_control_status = remote_control_ble_stop;
        // log_info("stop\n");
        break;
    }

}


void usr_uart_ble_ai_data_process(u8*data)
{
    // 一、检查帧头是否符合要求 
    if (data[0] != 0xC2 || data[1] != 0x04 || data[5] != 0xAA) {
        return; // 帧头错误
    }

    // 二、检查指令字节是否匹配
    if (data[2] != data[4]) {
        return; // 指令字节不匹配
    }


     // 四、根据指令字节设置标志位
     switch (data[2]) {
        case receive_ai_forward: 
            //前进10cm
            remote_control_status = remote_control_forward;
            break;

        case receive_ai_back: 
            remote_control_status = remote_control_back;
            break;

        case receive_ai_left: 
            remote_control_status = remote_control_left_forward;
            break;

        case receive_ai_right: 
            remote_control_status = remote_control_right_forward;
            break;

        default:
            remote_control_status = remote_control_ble_stop;
            break;
     }
}


void usr_uart_ble_remote_data_process(u8*data)
{
    
    // 一、检查帧头是否符合要求 
    if (data[0] != 0xC1 || data[1] != 0x03 || data[4] != 0xAA) {
        return; // 帧头错误
    }

    // 二、检查指令字节是否匹配
    if (data[2] != data[3]) {
        return; // 指令字节不匹配
    }

     // 四、根据指令字节设置标志位
     switch (data[2]) {
        case receive_ble_left_up: 
            //前进10cm
            remote_control_status = remote_control_forward;
            break;

        case receive_ble_left_down: 
            remote_control_status = remote_control_back;
            break;

        case receive_ble_left_left: 
            remote_control_status = remote_control_left_forward;
            break;

        case receive_ble_left_right: 
            remote_control_status = remote_control_right_forward;
            break;

        case receive_ble_left_mid: 
            //速度切换
            remote_control_status = remote_control_distance_switch;
            break;

        // case receive_ble_right_up: 
        //     //for20
        //     remote_control_status = remote_control_forward_20;
        //     break;

        // case receive_ble_right_down: 
        //     //back20
        //     remote_control_status = remote_control_back_20;
        //     break;

        case receive_ble_right_left: 
            //头部左转
            remote_control_status = remote_control_head_left;
            break;

        case receive_ble_right_right: 
            //头部右转
            remote_control_status = remote_control_head_right;
            break;

        case receive_ble_right_mid: 
            remote_control_status = remote_control_angle_switch;
            break;
        
        default:
            remote_control_status = remote_control_ble_stop;
            break;

     }
}
//=========================================================================================//
//=========================================================================================//
//=========================================================================================//
//10ms时基
void uart1_check_isr(void)
{
    //------------------------------------------//
    if(standby_cb)
    {
        standby_cb--;
        if(standby_cb==0)
        {
           log_info("standby");
//           user_loop_play_f1a(standby_f1a);
           stop_check_time=0;
        }
    }
    //------------------------------------------//
    if(stop_check_time)
    {
        stop_check_time--;
    }
    //------------------------------------------//



}
//------------------------------------------------------------//
u32  spi_key;
void SPI_makekey(unsigned char d)
{

    u8 n=8;

    while(n)
    {

        if(d&0x80)
        {
            spi_key++;
        }
        d<<=1;
        n--;
    }

}


u8 TUB_data_checksum(void)
{
    spi_key = 0;

    SPI_makekey(tmp_buf[1]);
    SPI_makekey(tmp_buf[2]);
    SPI_makekey(tmp_buf[3]);

    if(tmp_buf[4] == spi_key)
    {
        return 1;
    }
    else
    {
        log_info("tub  checksum error");
        return 0;
    }


}
u8 key_flag4,key_flag5,key_flag6,key_flag7,key_flag8;
//-----------------------------------------------------------------------------------------------------//


u8 key_last;



u8 rf_key(){
    return key_last;
}

void uart1_data_handler(void)
{
    if(uart1_cp_ok_flag==1)
    {
        uart1_cp_ok_flag=0;

        usr_uart_24data_process(tmp_buf);
        usr_uart_ble_ai_data_process(tmp_buf);

        usr_uart_ble_remote_data_process(tmp_buf);
        //判断数据是否相同
        if(memcmp(tmp_buf, last_buf, uart1_recv_len) != 0) {
                data_changed = 1;
            }
        //==== 仅当数据变化时才发送 ==== //
        if(data_changed) {
        UT1_write_buf(tmp_buf, uart1_recv_len);
        data_changed = 0;
        // ==== 更新历史数据 ==== //
        memcpy(last_buf, tmp_buf, uart1_recv_len);
        last_len = uart1_recv_len;
        }
    }
}

#endif


void uart1_write_data(const u8 *buf, u32 len)
{
    u32 i;
    if (len == 0) {
        return;
    }
    if (1) {
        if (0) {
            uart_set_dma_dir(0);
        }
        UT_OSSemSet(&uart1.sem_tx, 0);
        JL_UT1->CON0 |= BIT(13);
        JL_UT1->CON0 |= BIT(2);
        JL_UT1->TXADR = (u32)buf;
        JL_UT1->TXCNT = len;
        UT_OSSemPend(&uart1.sem_tx, 0);
        JL_UT1->CON0 &= ~BIT(2);
    } else {
        for (i = 0; i < len; i ++) {
            UT1_putbyte(*(buf + i));
        }
    }
}

