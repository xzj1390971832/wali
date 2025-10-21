#ifndef TM_UART_DRIVER_H
#define TM_UART_DRIVER_H


#include "typedef.h"
#include "gpio.h"
#include "user_config.h"

typedef struct 
{
    u8 wr_flag;
    u8 ram_buff[6];

    //isr
    u8 send_flow;
    u8 send_byte;
    u8 send_len;
}T_tm_uart;

//
typedef enum
{
    JZQ_U1_L1,
    JZQ_U1_L2,
    JZQ_U1_L3,
    JZQ_U1_L4,
    JZQ_U1_L5,
    JZQ_U1_L6,
    JZQ_U1_L7,
    JZQ_U1_L8,
    JZQ_U1_L9,
    JZQ_U1_PK_L1,
    JZQ_U1_PK_L2,
    JZQ_U1_PK_L3,

    JZQ_U2_L1,
    JZQ_U2_L2,
    JZQ_U2_L3,
    JZQ_U2_L4,
    JZQ_U2_L5,
    JZQ_U2_L6,
    JZQ_U2_L7,
    JZQ_U2_L8,
    JZQ_U2_L9,
    JZQ_U2_PK_L1,
    JZQ_U2_PK_L2,
    JZQ_U2_PK_L3,


    JZQ_TEST_MAX,
};


//端口配置
#define UART_PORT PORTB
#define UART_BIT  b5

#define UART_IO_H   port_set(UART_PORT,UART_BIT);
#define UART_IO_L   port_reset(UART_PORT,UART_BIT);
//映射
#define  tm1652_g1_buff tm_uart.ram_buff[0]
#define  tm1652_g2_buff tm_uart.ram_buff[1]
#define  tm1652_g3_buff tm_uart.ram_buff[2]
#define  tm1652_g4_buff tm_uart.ram_buff[3]
#define  tm1652_g5_buff tm_uart.ram_buff[4]
#define  tm1652_g6_buff tm_uart.ram_buff[5]
//
#define SEG1_BIT b0
#define SEG2_BIT b1
#define SEG3_BIT b2
#define SEG4_BIT b3
#define SEG5_BIT b4
#define SEG6_BIT b5
#define SEG7_BIT b6
#define SEG8_BIT b7

//符号设定
//U1红灯
#define  TM_R1_ON  tm1652_g2_buff |= SEG1_BIT
#define  TM_R2_ON  tm1652_g2_buff |= SEG2_BIT
#define  TM_R3_ON  tm1652_g2_buff |= SEG3_BIT
#define  TM_R4_ON  tm1652_g2_buff |= SEG4_BIT
#define  TM_R5_ON  tm1652_g2_buff |= SEG5_BIT
#define  TM_R6_ON  tm1652_g2_buff |= SEG6_BIT
#define  TM_R7_ON  tm1652_g2_buff |= SEG7_BIT
#define  TM_R8_ON  tm1652_g2_buff |= SEG8_BIT
#define  TM_R9_ON  tm1652_g4_buff |= SEG8_BIT


#define  TM_STA_R1_ON  tm1652_g3_buff |= SEG2_BIT
#define  TM_STA_R2_ON  tm1652_g4_buff |= SEG2_BIT
#define  TM_STA_R3_ON  tm1652_g5_buff |= SEG3_BIT
//
#define  TM_R1_OFF  tm1652_g2_buff &= ~SEG1_BIT
#define  TM_R2_OFF  tm1652_g2_buff &= ~SEG2_BIT
#define  TM_R3_OFF  tm1652_g2_buff &= ~SEG3_BIT
#define  TM_R4_OFF  tm1652_g2_buff &= ~SEG4_BIT
#define  TM_R5_OFF  tm1652_g2_buff &= ~SEG5_BIT
#define  TM_R6_OFF  tm1652_g2_buff &= ~SEG6_BIT
#define  TM_R7_OFF  tm1652_g2_buff &= ~SEG7_BIT
#define  TM_R8_OFF  tm1652_g2_buff &= ~SEG8_BIT
#define  TM_R9_OFF  tm1652_g4_buff &= ~SEG8_BIT


#define  TM_STA_R1_OFF  tm1652_g3_buff &= ~SEG2_BIT
#define  TM_STA_R2_OFF  tm1652_g4_buff &= ~SEG2_BIT
#define  TM_STA_R3_OFF  tm1652_g5_buff &= ~SEG3_BIT
//U2蓝灯
#define  TM_B1_ON  tm1652_g1_buff |= SEG1_BIT
#define  TM_B2_ON  tm1652_g1_buff |= SEG2_BIT
#define  TM_B3_ON  tm1652_g1_buff |= SEG3_BIT
#define  TM_B4_ON  tm1652_g1_buff |= SEG4_BIT
#define  TM_B5_ON  tm1652_g1_buff |= SEG5_BIT
#define  TM_B6_ON  tm1652_g1_buff |= SEG6_BIT
#define  TM_B7_ON  tm1652_g1_buff |= SEG7_BIT
#define  TM_B8_ON  tm1652_g1_buff |= SEG8_BIT
#define  TM_B9_ON  tm1652_g3_buff |= SEG8_BIT


#define  TM_STA_B1_ON  tm1652_g3_buff |= SEG1_BIT
#define  TM_STA_B2_ON  tm1652_g4_buff |= SEG1_BIT
#define  TM_STA_B3_ON  tm1652_g5_buff |= SEG1_BIT
//
#define  TM_B1_OFF  tm1652_g1_buff &= ~SEG1_BIT
#define  TM_B2_OFF  tm1652_g1_buff &= ~SEG2_BIT
#define  TM_B3_OFF  tm1652_g1_buff &= ~SEG3_BIT
#define  TM_B4_OFF  tm1652_g1_buff &= ~SEG4_BIT
#define  TM_B5_OFF  tm1652_g1_buff &= ~SEG5_BIT
#define  TM_B6_OFF  tm1652_g1_buff &= ~SEG6_BIT
#define  TM_B7_OFF  tm1652_g1_buff &= ~SEG7_BIT
#define  TM_B8_OFF  tm1652_g1_buff &= ~SEG8_BIT
#define  TM_B9_OFF  tm1652_g3_buff &= ~SEG8_BIT


#define  TM_STA_B1_OFF  tm1652_g3_buff &= ~SEG1_BIT
#define  TM_STA_B2_OFF  tm1652_g4_buff &= ~SEG1_BIT
#define  TM_STA_B3_OFF  tm1652_g5_buff &= ~SEG1_BIT

void tm1652_init(void);
void tm_uart_test(void);

void tm1652_write_ram(void);


void jzq_led_ctrl(u8 num,u8 sta);


void tm1652_send_data_isr(void);

void robot_dh_change(void);


#endif
