#ifndef USER_MOTOR_H
#define USER_MOTOR_H



#define B1_Z_PORT      PORTA
#define B1_Z_BIT       b8
#define B1_F_PORT      PORTA
#define B1_F_BIT       b3

#define B2_Z_PORT      PORTB
#define B2_Z_BIT       b2
#define B2_F_PORT      PORTB
#define B2_F_BIT       b1      


#define B3_Z_PORT      PORTB
#define B3_Z_BIT       b5
#define B3_F_PORT      PORTB
#define B3_F_BIT       b4    


#define B4_Z_PORT      PORTA
#define B4_Z_BIT       b11
#define B4_F_PORT      PORTA
#define B4_F_BIT       b12





#define L_FOOT_Z_PORT      PORTA
#define L_FOOT_Z_BIT       b14
#define L_FOOT_F_PORT      PORTA
#define L_FOOT_F_BIT       b13

#define R_FOOT_Z_PORT      PORTB
#define R_FOOT_Z_BIT       b3
#define R_FOOT_F_PORT      PORTB
#define R_FOOT_F_BIT       b2

#define CZ_Z_PORT      PORTA
#define CZ_Z_BIT       b12
#define CZ_F_PORT      PORTA
#define CZ_F_BIT       b11


//
#define B1_Z_CONFIG     port_mode_out(B1_Z_PORT,B1_Z_BIT)
#define B1_Z_ON         port_set     (B1_Z_PORT,B1_Z_BIT);
#define B1_Z_OFF        port_reset   (B1_Z_PORT,B1_Z_BIT);

#define B1_F_CONFIG    port_mode_out(B1_F_PORT,B1_F_BIT)
#define B1_F_ON        port_set     (B1_F_PORT,B1_F_BIT);
#define B1_F_OFF       port_reset   (B1_F_PORT,B1_F_BIT);


#define B1_Z                        B1_Z_ON;    B1_F_OFF;
#define B1_F                        B1_Z_OFF;   B1_F_ON;
#define B1_STOP                     B1_Z_ON;    B1_F_ON;
#define B1_BRAKE                    B1_Z_OFF;   B1_F_OFF;
//
#define B2_Z_CONFIG     port_mode_out(B2_Z_PORT,B2_Z_BIT)
#define B2_Z_ON         port_set     (B2_Z_PORT,B2_Z_BIT);
#define B2_Z_OFF        port_reset   (B2_Z_PORT,B2_Z_BIT);

#define B2_F_CONFIG     port_mode_out(B2_F_PORT,B2_F_BIT)
#define B2_F_ON         port_set     (B2_F_PORT,B2_F_BIT);
#define B2_F_OFF        port_reset   (B2_F_PORT,B2_F_BIT);


#define B2_Z                        B2_Z_ON;    B2_F_OFF;
#define B2_F                        B2_Z_OFF;   B2_F_ON;
#define B2_STOP                     B2_Z_ON;    B2_F_ON;
#define B2_BRAKE                    B2_Z_OFF;   B2_F_OFF;
//
#define B3_Z_CONFIG     port_mode_out(B3_Z_PORT,B3_Z_BIT)
#define B3_Z_ON         port_set     (B3_Z_PORT,B3_Z_BIT);
#define B3_Z_OFF        port_reset   (B3_Z_PORT,B3_Z_BIT);

#define B3_F_CONFIG     port_mode_out(B3_F_PORT,B3_F_BIT)
#define B3_F_ON         port_set     (B3_F_PORT,B3_F_BIT);
#define B3_F_OFF        port_reset   (B3_F_PORT,B3_F_BIT);


#define B3_Z                        B3_Z_ON;    B3_F_OFF;
#define B3_F                        B3_Z_OFF;   B3_F_ON;
#define B3_STOP                     B3_Z_ON;    B3_F_ON;
#define B3_BRAKE                    B3_Z_OFF;   B3_F_OFF;
//
#define B4_Z_CONFIG     port_mode_out(B4_Z_PORT,B4_Z_BIT)
#define B4_Z_ON         port_set     (B4_Z_PORT,B4_Z_BIT);
#define B4_Z_OFF        port_reset   (B4_Z_PORT,B4_Z_BIT);

#define B4_F_CONFIG    port_mode_out(B4_F_PORT,B4_F_BIT)
#define B4_F_ON        port_set     (B4_F_PORT,B4_F_BIT);
#define B4_F_OFF       port_reset   (B4_F_PORT,B4_F_BIT);


#define B4_Z                        B4_Z_ON;    B4_F_OFF;
#define B4_F                        B4_Z_OFF;   B4_F_ON;
#define B4_STOP                     B4_Z_ON;    B4_F_ON;
#define B4_BRAKE                    B4_Z_OFF;   B4_F_OFF;
//---------------------------------------------------------
#define L_FOOT_Z_CONFIG     port_mode_out(L_FOOT_Z_PORT,L_FOOT_Z_BIT);
#define L_FOOT_Z_ON         port_set     (L_FOOT_Z_PORT,L_FOOT_Z_BIT);
#define L_FOOT_Z_OFF        port_reset   (L_FOOT_Z_PORT,L_FOOT_Z_BIT);

#define L_FOOT_F_CONFIG     port_mode_out(L_FOOT_F_PORT,L_FOOT_F_BIT);
#define L_FOOT_F_ON         port_set     (L_FOOT_F_PORT,L_FOOT_F_BIT);
#define L_FOOT_F_OFF        port_reset   (L_FOOT_F_PORT,L_FOOT_F_BIT);

#define L_FOOT_Z                     L_FOOT_Z_ON;    L_FOOT_F_OFF;
#define L_FOOT_F                     L_FOOT_Z_OFF;   L_FOOT_F_ON;
#define L_FOOT_STOP                  L_FOOT_Z_ON;    L_FOOT_F_ON;
//
#define R_FOOT_Z_CONFIG     port_mode_out(R_FOOT_Z_PORT,R_FOOT_Z_BIT);
#define R_FOOT_Z_ON         port_set     (R_FOOT_Z_PORT,R_FOOT_Z_BIT);
#define R_FOOT_Z_OFF        port_reset   (R_FOOT_Z_PORT,R_FOOT_Z_BIT);

#define R_FOOT_F_CONFIG     port_mode_out(R_FOOT_F_PORT,R_FOOT_F_BIT);
#define R_FOOT_F_ON         port_set     (R_FOOT_F_PORT,R_FOOT_F_BIT);
#define R_FOOT_F_OFF        port_reset   (R_FOOT_F_PORT,R_FOOT_F_BIT);


#define R_FOOT_Z                     R_FOOT_Z_ON;    R_FOOT_F_OFF;
#define R_FOOT_F                     R_FOOT_Z_OFF;   R_FOOT_F_ON;
#define R_FOOT_STOP                  R_FOOT_Z_ON;    R_FOOT_F_ON;
//
#define CZ_Z_CONFIG     port_mode_out(CZ_Z_PORT,CZ_Z_BIT);
#define CZ_Z_ON         port_set     (CZ_Z_PORT,CZ_Z_BIT);
#define CZ_Z_OFF        port_reset   (CZ_Z_PORT,CZ_Z_BIT);

#define CZ_F_CONFIG     port_mode_out(CZ_F_PORT,CZ_F_BIT);
#define CZ_F_ON         port_set     (CZ_F_PORT,CZ_F_BIT);
#define CZ_F_OFF        port_reset   (CZ_F_PORT,CZ_F_BIT);


#define CZ_Z                     CZ_Z_ON;    CZ_F_OFF;
#define CZ_F                     CZ_Z_OFF;   CZ_F_ON;
#define CZ_STOP                  CZ_Z_ON;    CZ_F_ON;
//------------------------------------------------------------------------//
//LED
//------------------------------------------------------------------------//
#define LED_PORT        PORTA
#define LED_BIT         b11
#define LED_CONFIG     port_mode_out(LED_PORT,LED_BIT);
#define LED_ON         port_set     (LED_PORT,LED_BIT);
#define LED_OFF        port_reset   (LED_PORT,LED_BIT);
//------------------------------------------------------------------------//
typedef enum
{
    c_key_release = 0x00,
    //----------------------------------//
    c_key_left_up       = 0x0B,
    c_key_left_down     = 0x0C,
    c_key_left_left     = 0x17,
    c_key_left_rgiht    = 0x0A,
    c_key_left_mid      = 0x08,
    //
    c_key_right_up      = 0x1A,
    c_key_right_down    = 0x02,
    c_key_right_left    = 0x09,
    c_key_right_rgiht   = 0x19,
    c_key_right_mid     = 0x08,
    //
    c_key_demo       = 0x07,
    c_key_program    = 0x03,
    //-------------- ���� -------------//
    //1
    c_key_gripper       = 0x04,
    c_key_vol_add       = 0x01,
    c_key_vol_del       = 0x15,
    c_key_shoot         = 0x14,
    //2
    c_key_left_arm_up       = 0x05,
    c_key_left_arm_down     = 0x16,
    c_key_stop              = 0x18,
    c_key_right_arm_up      = 0x0F,
    c_key_right_arm_down    = 0x1C,
    //3
    c_key_talk1         = 0x11,
    c_key_dance         = 0x10,
    c_key_battle        = 0x12,
    c_key_talk2         = 0x0D,
    c_key_light         = 0x13,
    c_key_move_talk     = 0x0E,
    //----------------------------------//


}T_REMOTEC_CODE;




//
#define  B1_KEY_R   c_key_talk1
#define  B1_KEY_L   c_key_left_arm_up
#define  B2_KEY_L  c_key_left_arm_down
#define  B2_KEY_R  c_key_dance
#define  B3_KEY_L  c_key_right_arm_up
#define  B3_KEY_R  c_key_light
#define  B4_KEY_L c_key_right_arm_down
#define  B4_KEY_R c_key_move_talk

#define  KEY_FOOT_L_FORWORD     c_key_left_up
#define  KEY_FOOT_L_BACK        c_key_left_down
#define  KEY_FOOT_R_FORWORD     c_key_right_up
#define  KEY_FOOT_R_BACK        c_key_right_down
#define  KEY_CZ_R_OPEN           c_key_battle
#define  KEY_CZ_R_CLOSE          c_key_talk2


typedef enum
{
        c_motor_stop,
        up,
        down    ,
}T_MOTOR_WORK_FLAG;
typedef struct
{
    T_MOTOR_WORK_FLAG work_flag;
    u8 up_timer;
    u8 dn_timer;
    u32* pos_pin;
    u32* neg_pin;

}T_MOTOR_CTRL;
extern T_MOTOR_CTRL dj1,dj2,dj3,dj4,cz,l_foot,r_foot;
extern u32 adc_value[4];

extern u8 rf_key();


// void  motor_greeting_test(void);
void motor_init(void);
void motor1_t_run(void);
void motor2_t_run(void);
void motor3_t_run(void);
void motor4_t_run(void);

// 跨文件访问的全局变量声明
extern volatile bool greeting_pause_flag;
volatile bool poweron_delay_status;
extern volatile u32 greeting_pause_counter;
extern volatile u8 greeting_delay_counter;
extern volatile bool greeting_delay_active;
extern  u8 ORIGIN_FINISH_STATUS;
extern volatile u8 ORIGIN_FINISH_DELAY_COUNT;
extern volatile u8 origin_delay_status;
#define GREETING_PAUSE_TARGET 5
//-------------------------------------------------------------------------//
//pwm
// 定义电机控制状态变量
typedef struct {
    uint8_t direction;   // 方向控制: 1=正转, 0=反转
    uint8_t num;         // 当前PWM占空比
    uint8_t val;         // 目标PWM占空比
    uint8_t running;     // 运行状态: 1=运行, 0=停止
    uint8_t pwm;         // 当前PWM输出
} MotorPWMControl;

// 声明电机控制变量（使用extern关键字）
extern MotorPWMControl motor1;
extern MotorPWMControl motor2;
extern MotorPWMControl motor3;
extern MotorPWMControl motor4;

typedef enum {
    STATE_FORWARD,
    STATE_LEFT_FORWARD,
    STATE_RIGHT_FORWARD,
    STATE_RIGHT_BACK,
    STATE_LEFT_BACK,
    STATE_BACK,
    STATE_DELAY,       // 新增：延时状态
    STATE_COMPLETE
} MotorStage;

extern volatile bool action_completed;
extern volatile bool action_delay;
extern volatile u32 delay_counter;  // 延时计数器
extern volatile MotorStage current_state;
extern volatile u32 ir_change_count;       // 红外变化次数计数
extern volatile u32 last_ir_value;         // 上一次红外值
extern volatile bool ir_detection_enabled; // 红外检测使能标志
//-------------------------------------------------------------------------//


void motor1_z_run_40(void);
void motor2_z_run_40(void);
void motor3_z_run_40(void);
void motor4_z_run_40(void);
void motor_forward(void);
void motor_back(void);
void motor_head_rotate_left(void);
void motor_head_rotate_right(void);
void motor_head_return(void);
void motor_head_rotate_left_to_angle(void);
void motor_head_rotate_right_to_angle(void);
void motor_forward_check(void);
void motor_forward_check_10cm(void);
void motor_back_check(void);
void motor_back_check_10cm(void);
void motor_left_forward_check_90(void);
void motor_right_forward_check_90(void);
void motor_left_back_check_90(void);
void motor_right_back_check_90(void);
void motor_left_forward_check_45(void);
void motor_right_forward_check_45(void);
void motor_demo_execute(void);

// 声明函数

void update_motor_pwm(void);
void motor_ctrl(void);

//-------------------------------------------------------------------------//
//LED
//-------------------------------------------------------------------------//
void LED_Init(void);

// #define DELAY_TARGET 10

#endif

