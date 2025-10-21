
#include "user_config.h"
//#include "user_key_app.h"
//#include "sfr.h"
#include "tm_uart_driver.h"


#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"



T_tm_uart tm_uart;

//----------------------------------------------------------------------------------------------//
static  void delay_ms(u32 ms)
{
	int i,us;

    for (i = 0; i < ms; i++) {

        for (us = 0; us < 100; us++)
        {
       		wdt_clear();
        }

    }
}

static  void delay(u32 us)  //10us
{
	int i;

    for (i = 0; i < us; i++)//11
    {
        wdt_clear();
    }

}
//----------------------------------------------------------------------------------------------//
void tm1652_init(void)
{

    port_mode_out(UART_PORT,UART_BIT);

    memset(&tm_uart,0,sizeof(tm_uart));

        //上电显示
        tm_uart.ram_buff[0] = 0xaa;
        tm_uart.ram_buff[1] = 0x01;
        tm_uart.ram_buff[2] = 0x01;

        //
        tm_uart.wr_flag = 5;
        tm_uart.send_flow = 1;
        tm_uart.send_len = 0;
        tm_uart.send_byte = tm_uart.ram_buff[0];
}

//2400: 416us脉宽   19200:52us
#define uart_mk_len 92
void tm1652_send_data(unsigned char sdat)
{
#if 0
    //硬件UART1 发送
    UT1_putbyte(sdat);
    delay_ms(2);
#else
    //IO模拟发送，注意宽度47us~57us

 unsigned char i=0, sfalg=1;

   //起始位
    UART_IO_H;
   delay(1);     
    UART_IO_L;
   delay(5);    //

   //发送8位数据
   for(i=0; i<8; i++)
   {
      if(sdat & 0x01)
      {
         UART_IO_H;
         sfalg ^= 1;
      }else
      {
         UART_IO_L;
      }
      delay(uart_mk_len);
      sdat >>=1;
   }
#if 0
    //校验位,按照发送数据中1的个数来判断
    //偶校验： 奇数0，偶数1
    if(sfalg)
    {
      UART_IO_H;
    }
    else
    {
      UART_IO_L;
    }
    delay(12);
#endif
    //停止位
    UART_IO_H;
    delay(uart_mk_len);
#endif

//delay_ms(1);//
}


void tm1652_write_ram(void)
{


    if(tm_uart.wr_flag==0)
    {
        return;
    }
    else
    {
        tm_uart.wr_flag--;
    }


#if  0
        //符合1652的串口时序
    	tm1652_send_data(0x08);//设置数据，最大长度6
        tm1652_send_data(tm_uart.ram_buff[0]);
        tm1652_send_data(tm_uart.ram_buff[1]);
        tm1652_send_data(tm_uart.ram_buff[2]);
        tm1652_send_data(tm_uart.ram_buff[3]);
        tm1652_send_data(tm_uart.ram_buff[4]);
 //     tm1652_send_data(tm_uart.ram_buff[5]);

        //
        UART_IO_H;
        delay_ms(10);
        tm1652_send_data(0x18);     //写入数据
        tm1652_send_data(0xFF);     //设置占空比+段电流
        //
        UART_IO_H;
        delay_ms(10);
#endif
        //自模拟2400的程序


        //发送数据
        tm1652_send_data(0xaa);
       // tm1652_send_data(tm_uart.ram_buff[1]);
       // tm1652_send_data(tm_uart.ram_buff[2]);

        //恢复空闲
        delay_ms(10);
        UART_IO_H;


}
void jzq_led_ctrl(u8 num,u8 sta)
{
    //
    tm_uart.wr_flag = 10;


    switch (num)
    {
    //-----------------------------------------//
    case JZQ_U1_L1:
        if(sta) {TM_R1_ON; }
        else    {TM_R1_OFF;}
        break;
    case JZQ_U1_L2:
        if(sta) {TM_R2_ON; }
        else    {TM_R2_OFF;}
        break;
    case JZQ_U1_L3:
        if(sta) {TM_R3_ON; }
        else    {TM_R3_OFF;}
        break;
    case JZQ_U1_L4:
        if(sta) {TM_R4_ON; }
        else    {TM_R4_OFF;}
        break;
    case JZQ_U1_L5:
        if(sta) {TM_R5_ON; }
        else    {TM_R5_OFF;}
        break;
    case JZQ_U1_L6:
        if(sta) {TM_R6_ON; }
        else    {TM_R6_OFF;}
        break;
    case JZQ_U1_L7:
        if(sta) {TM_R7_ON; }
        else    {TM_R7_OFF;}
        break;
    case JZQ_U1_L8:
        if(sta) {TM_R8_ON; }
        else    {TM_R8_OFF;}
        break;
    case JZQ_U1_L9:
        if(sta) {TM_R9_ON; }
        else    {TM_R9_OFF;}
        break;


    case JZQ_U1_PK_L1:
        if(sta) {TM_STA_R1_ON; }
        else    {TM_STA_R1_OFF;}
        break;

    case JZQ_U1_PK_L2:
        if(sta) {TM_STA_R2_ON; }
        else    {TM_STA_R2_OFF;}
        break;

    case JZQ_U1_PK_L3:
        if(sta) {TM_STA_R3_ON; }
        else    {TM_STA_R3_OFF;}
        break;
    //-----------------------------------------//
    case JZQ_U2_L1:
        if(sta) {TM_B1_ON; }
        else    {TM_B1_OFF;}
        break;
    case JZQ_U2_L2:
        if(sta) {TM_B2_ON; }
        else    {TM_B2_OFF;}
        break;
    case JZQ_U2_L3:
        if(sta) {TM_B3_ON; }
        else    {TM_B3_OFF;}
        break;
    case JZQ_U2_L4:
        if(sta) {TM_B4_ON; }
        else    {TM_B4_OFF;}
        break;
    case JZQ_U2_L5:
        if(sta) {TM_B5_ON; }
        else    {TM_B5_OFF;}
        break;
    case JZQ_U2_L6:
        if(sta) {TM_B6_ON; }
        else    {TM_B6_OFF;}
        break;
    case JZQ_U2_L7:
        if(sta) {TM_B7_ON; }
        else    {TM_B7_OFF;}
        break;
    case JZQ_U2_L8:
        if(sta) {TM_B8_ON; }
        else    {TM_B8_OFF;}
        break;
    case JZQ_U2_L9:
        if(sta) {TM_B9_ON; }
        else    {TM_B9_OFF;}
        break;


    case JZQ_U2_PK_L1:
        if(sta) {TM_STA_B1_ON; }
        else    {TM_STA_B1_OFF;}
        break;

    case JZQ_U2_PK_L2:
        if(sta) {TM_STA_B2_ON; }
        else    {TM_STA_B2_OFF;}
        break;

    case JZQ_U2_PK_L3:
        if(sta) {TM_STA_B3_ON; }
        else    {TM_STA_B3_OFF;}
        break;
    //-----------------------------------------//




    default:
        break;
    }







}

#if 0
u8 test_cnt=0,test_sta=1,speed;
void jzq_test(void)
{
if(++speed==4)
{
    speed=0;
}



    jzq_led_ctrl(test_cnt,1);

   //
    if(++test_cnt==JZQ_TEST_MAX)
    {
        test_cnt = JZQ_U1_L1;
      //   test_sta ^= 1;
    }

}
#endif

//u8 

void tm1652_send_bit(u8 buff,u8 send_bit)
{

    if(buff & send_bit)
    {
      UART_IO_H;
    }
    else
    {
      UART_IO_L;        
    }
}
void tm1652_send_data_isr(void)
{

        switch (tm_uart.send_flow)
        {
            //无数据发送，结束本函数，退出查询。            
            case 0:return;

            //发送流程
            case 1:  UART_IO_H;break;//空闲
            case 2:  UART_IO_L;break;//起始
            case 3:  tm1652_send_bit(tm_uart.send_byte,b0);break;
            case 4:  tm1652_send_bit(tm_uart.send_byte,b1);break;          
            case 5:  tm1652_send_bit(tm_uart.send_byte,b2);break;
            case 6:  tm1652_send_bit(tm_uart.send_byte,b3);break;  
            case 7:  tm1652_send_bit(tm_uart.send_byte,b4);break;
            case 8:  tm1652_send_bit(tm_uart.send_byte,b5);break;          
            case 9:  tm1652_send_bit(tm_uart.send_byte,b6);break;
            case 10: tm1652_send_bit(tm_uart.send_byte,b7);break;             
            case 11: UART_IO_H;break;//停止   

            //给点延时帧间隔
            case 12:
            case 13:
            case 14:
            case 15:                                    
            UART_IO_H;
            break;
            
            default:
                break;
        }

        //1byte发送完毕
        if(tm_uart.send_flow==15)
        {
            //查询是否重发
            if(tm_uart.wr_flag==0)
            {
                tm_uart.send_flow=0;
            }
            else
            {

                if(tm_uart.send_len==2)
                {
                      //发送完一帧                    
                      tm_uart.wr_flag--;    
                      tm_uart.send_len=0;   

     
                 

                }
                else
                {
                    tm_uart.send_len++; 
                }
                tm_uart.send_byte = tm_uart.ram_buff[tm_uart.send_len];

                tm_uart.send_flow=1;
            }


        }
        else
        {   
  
            tm_uart.send_flow++;
        }


}
extern u32 spi_key;
extern u8 n;
extern void SPI_makekey(unsigned char d);



void dh_startup(u8 cnt)
{
    

        //上电显示
        tm_uart.ram_buff[0] = 0xaa;
        tm_uart.ram_buff[1] = cnt;
    	
        //
        u8 check_sum = 0;    
        u8 k=8;
        u8 data = tm_uart.ram_buff[1];
        while(k)
        {

            if(data&0x80)
            {
                check_sum++;
            }
            data<<=1;
            k--;
        }

    //
        tm_uart.ram_buff[2] = check_sum;

        //
        tm_uart.wr_flag = 5;
        tm_uart.send_flow = 1;
        tm_uart.send_len = 0;
        tm_uart.send_byte = tm_uart.ram_buff[0];

        log_info("dh_startup --> b0:%x,b1:%x,b2:%x,",tm_uart.ram_buff[0],tm_uart.ram_buff[1],tm_uart.ram_buff[2]);
}
u8  dh_cnt,speed=0;
void robot_dh_change(void)
{
    if(speed<4)
    {
        speed++;



    }
    else
    {
                speed=0;


                
		if(++dh_cnt==106)
		{
			dh_cnt = 0;
 

		}

                   dh_startup(dh_cnt);
    }

    

}
//----------------------------------------------------------------------------------------------//



