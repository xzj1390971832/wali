#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#include "typedef.h"
#include "gpio.h"
#include "resources.h"

#define  S_POWER_CTRL_EN  0  //io_key也要开启
#define  S_POWER_CTRL_MOS_PORT	 PORTA
#define  S_POWER_CTRL_MOS_BIT    b12

#define  S_POWER_CTRL_MOS_OUT_CONFIG    port_mode_out(S_POWER_CTRL_MOS_PORT,S_POWER_CTRL_MOS_BIT)
#define  S_POWER_CTRL_MOS_ON            port_set(S_POWER_CTRL_MOS_PORT,S_POWER_CTRL_MOS_BIT)
#define  S_POWER_CTRL_MOS_OFF		    port_reset(S_POWER_CTRL_MOS_PORT,S_POWER_CTRL_MOS_BIT)



#define b0              0x0001
#define b1              0x0002
#define b2              0x0004
#define b3              0x0008
#define b4              0x0010
#define b5              0x0020
#define b6              0x0040
#define b7              0x0080
#define b8              0x0100
#define b9              0x0200
#define b10             0x0400
#define b11             0x0800
#define b12             0x1000
#define b13             0x2000
#define b14             0x4000
#define b15             0x8000
//=================================================================================//
//----------------------------------声音重映射--------------------------------------//
//=================================================================================//


enum
{
	S_on_f1a=1,
	S_off_f1a=1,

};

//=================================================================================//
//-------------------------------------GPIO----------------------------------------//
//=================================================================================//
typedef enum
{
	GPIO_IOC_OUTPUT=0,
	GPIO_IOC_INPUT=1,

}T_ioc_status;

typedef enum
{
	GPIO_FLOAT=0,
	GPIO_IN_UP,
	GPIO_IN_DOWN,

}T_input_mode;
//=================================================================================//
void user_poweron_init(void);
void user_sleep_init(void);
void user_wakeup_init(void);
//=================================================================================//
//输出输入，跨平台通用函数
#define PORTA       JL_PORTA
#define PORTB       JL_PORTB
#define PORTD       JL_PORTD


//输入
void port_mode_ipu(JL_PORT_TypeDef* port, u16 pin) ;
void port_mode_ipd(JL_PORT_TypeDef* port, u16 pin) ;
void port_mode_floating(JL_PORT_TypeDef* port, u16 pin) ;
void port_mode_ain(JL_PORT_TypeDef* port, u16 pin) ;
u16 port_read(JL_PORT_TypeDef* port, u16 pin) ;


//输出
void port_mode_out(JL_PORT_TypeDef* port, u16 pin);
void port_write(JL_PORT_TypeDef* port, u16 value);
void port_set(JL_PORT_TypeDef* port, u16 pin);
void port_reset(JL_PORT_TypeDef* port, u16 pin);
void port_toggle(JL_PORT_TypeDef* port, u16 pin);

//大电流输出
void port_hsink(JL_PORT_TypeDef* port, u16 pin);
void port_hsink_1(JL_PORT_TypeDef* port, u16 pin);
void port_hsink_2(JL_PORT_TypeDef* port, u16 pin);
void port_hsink_3(JL_PORT_TypeDef* port, u16 pin);
//=================================================================================//

//music play
void user_play_f1a(u32 f1a_index);
void user_loop_play_f1a(u32 f1a_index);

void  user_f1a_stop(void);

//-------------------------------------------------------//

// /**
//   **********************************************************************************
//   * SLEP
//   **********************************************************************************
//   */

// enum {
//     C_OFF,
//     C_ACTIVE,
//     C_STANDBY,
//     C_WAKEUP,
//     c_test_mode,
// };

// #define SlepTime(time)            ((time)/100)
// #define FeedSlep(time)            feed_slep(SlepTime(time))
// #define FeedSlepEx(time)          feed_slep_ex(SlepTime(time))

// #define C_SLEP_TIME               2000 /* 休眠时间=2000ms */
// #define C_STB_TIME                500  /* 待机时间=500ms */

// extern void dec_sleptime(void);
// extern void feed_slep(u32 time);
// extern void feed_slep_ex(u32 time);
// extern void check_slep(void);
// extern void system_on(__bool voice);
// extern void system_off(__bool voice);
// extern void system_onoff(void);
// extern u8 get_sys_sta(void);

// extern void io_reinit(void);
// extern void io_close(void);
// // extern void wakeup_setting(void);
// extern void wkup_param_setting(void);



// /**
//   **********************************************************************************
//   * VM
//   **********************************************************************************
//   */


// extern __iomem__ u8 g_vsram_buff[VM_BUFF_SIZE];

// extern void v_malloc_init(void);
// extern void* v_malloc(u16 nbyte);
// extern void vm_sram_load(void);
// extern void vm_sram_flush(void);

// extern u8 vm_read_byte(u16 offset);
// extern void vm_write_byte(u16 offset, u8 dat);
// extern u16 vm_read_word(u16 offset);
// extern void vm_write_word(u16 offset, u16 dat);


// /**
//   **********************************************************************************
//   * Others
//   **********************************************************************************
//   */

// extern __iomem__ u8 g_sys_main_vol;
// extern u8 g_lvd_flag;


// extern __iomem__ u16 g_var_speech;
// // extern __iomem__ u16 g_var_speech1;
// // extern __iomem__ u16 g_var_speech2;
// // extern __iomem__ u16 g_var_speech3;
// // extern __iomem__ u16 g_var_speech4;
// // extern __iomem__ u16 g_var_speech5;


// extern unsigned long systick(u8 ioctl);
// extern void input(u16 key);
// extern u16 read_input(void);
// extern void program_off(void);
// extern void user_init(void);
// extern void app_sample(int *pmsg);
// extern void umain(void* param);
// extern void app_kick(void);

// extern void dac_fade_select(u8 fade);/* 1打开, 0关闭 */
// extern void dac_gain(u16 phy);


// Sentence_Extern(stc_1varsph);


// extern u16 srand_get(void);
// #define __SRAND()             srand_get() /* 获取随机�? */
// #define __SRAND_SEEK(a,b)     srand_get()%((b)-(a)+1)+(a) /* 获取区间[a,b]的随机数 */




/**
  * USER EXTERN HERE
  */





#endif
