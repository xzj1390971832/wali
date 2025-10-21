#include "sfr.h"
#include "cpu.h"
#include "config.h"
#include "gpio.h"
#include "clock.h"
#include "mcpwm.h"
#include "timer_drv.h"

#include "adc_drv.h"
#include "user_motor.h"
#include "user_config.h"
#include "tm_uart_driver.h"


#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"
#include "uart_dev.h"
#include "uart_my.h"

extern u32 adc_value[4]; 
#define TIMER_SFR(ch) JL_TMR##ch

u8 timer1_num = 0;
volatile u8 control = 0;
u16 timer1_500us_count = 0;
u16 timer2_500us_count = 0;

/*
 *timer中断优先级，范围:0~7(低~高)
 * */
#define _timer_init(ch,us)  \
    HWI_Install(IRQ_TIME##ch##_IDX, (u32)timer##ch##_isr, 7); 	\
	TIMER_SFR(ch)->PRD = clk_get("lsb")/1000000 * us;			\
	TIMER_SFR(ch)->CON = BIT(0)|BIT(6);

#define HS_MISO     gpio_read(IO_PORTA_11)

u8 spi_buf;
u8 n;
u8 k;
u8 spi_index;


SET(interrupt(""))
static void timer0_isr(void)
{
    TIMER_SFR(0)->CON |= BIT(6);
}

SET(interrupt(""))
static void timer1_isr(void)
{
    TIMER_SFR(1)->CON |= BIT(6);

    //250us
    // timer1_500us_count++;
    // if (timer1_500us_count == 4)
    // {
        // timer1_500us_count = 0;

        // 处理当前电机的PWM和缓启动
        static u8 motor_count = 0;
        motor_count++;
        
        if (motor_count >= 20) {
            motor_count = 0;
            
            // 缓启动控制：增加占空比直到达到目标值
            if (motor1.num < motor1.val) motor1.num++;
            if (motor2.num < motor2.val) motor2.num++;
            if (motor3.num < motor3.val) motor3.num++;
            if (motor4.num < motor4.val) motor4.num++;
        }

         // 更新每个电机的PWM输出
        motor1.pwm = (motor_count < motor1.num);
        motor2.pwm = (motor_count < motor2.num);
        motor3.pwm = (motor_count < motor3.num);
        motor4.pwm = (motor_count < motor4.num);
         
        // 更新电机控制状态
        update_motor_pwm();
    
}

SET(interrupt(""))
static void timer2_isr(void)
{
    TIMER_SFR(2)->CON |= BIT(6);
    timer2_500us_count++;
    if (timer2_500us_count == 1000)
    {
        timer2_500us_count = 0;

        control = !control;
        timer1_num = 0;
    }
    
}

void timer_init(u8 timer_ch, u32 us)
{
    switch (timer_ch) {
    case 0:
        _timer_init(0, us);
        break;
    case 1:
        _timer_init(1, us);
        break;
    case 2:
        _timer_init(2, us);
        break;
    default:
        break;
    }   
}

/*
 *timer pwm
 * */
const u32 timer2_pwm_tab[] = {
    IO_PORTA_11,
    IO_PORTA_12,
};

#define _timer2_pwm_init(ch,fre,duty)  							\
	TIMER_SFR(2)->PRD = clk_get("lsb")/fre;						\
	TIMER_SFR(2)->PWM##ch = TIMER_SFR(2)->PRD*duty/100;			\
	TIMER_SFR(2)->CON |= BIT(8 + 4*ch);							\
    gpio_set_direction(timer2_pwm_tab[ch], 0);      			\
    gpio_set_die(timer2_pwm_tab[ch], 0);      			\
	JL_IOMC->IOMC1 |= BIT(15 + ch);							\
	TIMER_SFR(2)->CON |= BIT(0)|BIT(6);
    

void timer2_pwm_init(u8 ch, u32 fre, u8 duty)
{
    switch (ch) {
    case 0:
        _timer2_pwm_init(0, fre, duty);
        log_info("pwm start");
        break;
    case 1:
        _timer2_pwm_init(1, fre, duty);
        break;
    default:
        break;
    }
}


void  SPI_Readabyte(void)
{
	k=8;
	spi_buf=0;
	while(k)
	{
		spi_buf>>=1;
		//HS_LED ^= 1;
		if(HS_MISO){
			spi_buf|= 0x80;
		}else{
			spi_buf&= 0x7F;
		}
		k--;

		udelay(338);
	}

}
