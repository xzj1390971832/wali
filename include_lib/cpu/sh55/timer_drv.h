#ifndef _TIMER_DRV_H
#define _TIMER_DRV_H

extern volatile u8 timer1_PWM;
extern volatile u8 control;

void timer_init(u8 timer_ch, u32 us);
void timer2_pwm_init(u8 ch, u32 fre, u8 duty);
void SPI_Readabyte();
#endif
