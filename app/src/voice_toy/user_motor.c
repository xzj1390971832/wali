
#include "typedef.h"
#include "user_config.h"
#include "user_motor.h"
#include <stdlib.h>  // 添加abs()函数所需的头文件

#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"

#include "mcpwm.h"
#include "timer_drv.h"
#include "uart_my.h"
#include "uart_dev.h"
///////////
//===========================================================================//
//电位器数据定义
u32 temp_val = 0;


u32 motor1_stop_time = 0;//电位器波动10内，计数
u32 motor1_up_move = 0;//上5次电位器度数
u32 motor1_get_time = 0;//扫描间隔
//
u32 motor2_stop_time = 0;//电位器波动10内，计数
u32 motor2_up_move = 0;//上5次电位器度数
u32 motor2_get_time = 0;//扫描间隔
//
u32 motor3_stop_time = 0;//电位器波动10内，计数
u32 motor3_up_move = 0;//上5次电位器度数
u32 motor3_get_time = 0;//扫描间隔
//
u32 motor4_stop_time = 0;//电位器波动10内，计数
u32 motor4_up_move = 0;//上5次电位器度数
u32 motor4_get_time = 0;//扫描间隔
//
u8  adc_log_speed;
u32 adc_value[4];
u32 adc_value_temp[4];

//===========================================================================//

//上电后延时标志位
volatile bool poweron_delay_status = false;

//复位标志位
bool need_reset_status = true;

//复位延时
volatile u8 origin_delay_status = 0;

#define CONTINUOUS_CHECK   5     // 连续有效判断次数
#define FOOT_COUNT         1     // 电位器/电机数量
#define FILTER_THRESHOLD   50     // 连续判断次数阈值
#define MOTOR_TARGET1      950

// 格式：{原点中心值, 公差范围}
static const struct {
    u32 origin_value;    // 原点中心AD值
    u32 tolerance;       // 公差范围（±tolerance）
} foot_origin[FOOT_COUNT] = {
    {530, 10},  // 电机1原点

};

// 红外检测阈值定义
#define IR_THRESHOLD 300   // 红外1遮挡阈值
#define IR2_THRESHOLD 100   // 红外2遮挡阈值
#define IR_CHANGE_COUNT 3 // 红外变化次数阈值

// 红外检测状态变量
volatile u32 ir_change_count = 0;       // 红外变化次数计数
volatile u32 last_ir_value = 0;         // 上一次红外值
volatile bool ir_detection_enabled = false; // 红外检测使能标志

//动作完成标志位

bool forward_complete_flag = 0;
bool back_complete_flag = 0;
bool forward_left_complete_flag = 0;
bool forward_right_complete_flag = 0;
bool back_left_complete_flag = 0;
bool back_right_complete_flag = 0;

// 电机状态枚举
typedef enum {
    MOTOR_STATE_STOP,    // 停止状态（可重新判断方向）
    MOTOR_STATE_FORWARD, // 正转状态（方向锁定）
    MOTOR_STATE_REVERSE  // 反转状态（方向锁定）
} MotorState;

// 每个电位器独立状态
typedef struct {
    u8 continuous_count;  // 连续在原点的次数（0~CONTINUOUS_CHECK）
    bool is_homed;        // 是否已归位（停止电机）
    MotorState state;     // 电机当前状态
    u8 filter_count;      // 方向判断滤波计数器
} FootStatus;

FootStatus foot_status[FOOT_COUNT] = {0};  // 初始化：计数0，未归位
u8 ORIGIN_FINISH_STATUS = 0;               // 复位完成标志位
u8 volatile ORIGIN_FINISH_DELAY_COUNT = 0;           // 复位延时标志位
 

volatile bool motor_reached[4] = {false, false, false, false}; // 分别对应电机1~4
volatile bool motor_started[4] = {false, false, false, false}; // 分别对应电机1~4
//-------------------------------------------------------------------------//
// 四个电机的控制结构
MotorPWMControl motor1 = {0, 0, 0, 0, 0};
MotorPWMControl motor2 = {0, 0, 0, 0, 0};
MotorPWMControl motor3 = {0, 0, 0, 0, 0};
MotorPWMControl motor4 = {0, 0, 0, 0, 0};


volatile MotorStage current_state = STATE_FORWARD;
volatile bool sequence_started = false;
volatile bool action_completed = false;
volatile bool action_delay = false;

//-------------------------------------------------------------------------//
//led
static LED_flash_flag = 0;
//-------------------------------------------------------------------------//
//2.4G

// 声明外部全局变量
extern volatile RemoteControlStatus remote_control_status;

u8 last_remote_control_status = 0;

// 遥控电机运行状态枚举
typedef enum {
    MOTOR_REMOTE_STATE_IDLE,
    MOTOR_REMOTE_STATE_FORWARD,
    MOTOR_REMOTE_STATE_FORWARD_20,
    MOTOR_REMOTE_STATE_BACK,
    MOTOR_REMOTE_STATE_BACK_20,
    MOTOR_REMOTE_STATE_LEFT_forward,
    MOTOR_REMOTE_STATE_RIGHT_forward,
    MOTOR_REMOTE_STATE_LEFT_forward_45,
    MOTOR_REMOTE_STATE_RIGHT_forward_45,
    MOTOR_REMOTE_STATE_LEFT_back,
    MOTOR_REMOTE_STATE_RIGHT_back,
    MOTOR_REMOTE_demo,
    MOTOR_REMOTE_HEAD_RESET,
    MOTOR_REMOTE_HEAD_LEFT,
    MOTOR_REMOTE_HEAD_RIGHT,
    
    MOTOR_REMOTE_PROGRAM,

    MOTOR_REMOTE_STOP
} current_remote_control_status;

current_remote_control_status current_motor_state = MOTOR_REMOTE_STATE_IDLE;

//-------------------------------------------------------------------------//
//切换状态标志位
//-------------------------------------------------------------------------//
u8 remote_control_distance_flag = 0;
u8 remote_control_angle_flag = 0;

//-------------------------------------------------------------------------//
//编程变量
//-------------------------------------------------------------------------//
// 全局变量
#define MAX_RECORD_LENGTH 50                        // 最大记录动作数量

typedef enum {                      
    PROGRAM_MODE_IDLE,                              // 编程空闲模式
    PROGRAM_MODE_RECORDING,                         // 编程记录模式
    PROGRAM_MODE_PLAYING                            // 编程播放模式
} ProgramMode;

// 全局变量
ProgramMode program_mode = PROGRAM_MODE_IDLE;
u8 record_buffer[MAX_RECORD_LENGTH] = {0};          // 存储记录的动作
u8 record_index = 0;                                // 记录索引
u8 play_index = 0;                                  // 播放索引
bool program_running = false;                       // 编程执行中标志

//-------------------------------------------------------------------------//
// 演示步骤枚举
typedef enum {
    DEMO_STEP_FORWARD,        // 前进
    DEMO_STEP_BACK,           // 后退
    DEMO_STEP_LEFT,           // 左转
    DEMO_STEP_RIGHT,          // 右转
    DEMO_STEP_HEAD_LEFT,      // 头部左转
    DEMO_STEP_HEAD_RIGHT,     // 头部右转
    DEMO_STEP_HEAD_RESET,     // 头部复位
    DEMO_STEP_COMPLETE        // 演示完成
} DemoStep;

static DemoStep current_demo_step = DEMO_STEP_COMPLETE;  // 当前演示步骤
static u32 demo_action_timer = 0;                   // 演示动作计时器
static bool demo_in_progress = false;                    // 演示是否进行中

//发送停止码
static u8 Uart_stop_code[10] = {0xC3,0x03,0xFF,0xFF,0xAA,0x00,0x00,0x00,0x00,0x00};
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
// //滤波
// void m1_filer(u32 value){


//         if(adc1_val == 0){
//                 adc1_val =  value;
//         }else{
//                 adc1_val = (adc1_val + value) / 2;
//         }
// }
//堵转检测
//m1
void motor1_dz_check(){
return;//先不检测
        if(++motor1_get_time == 50)
        {
            motor1_get_time = 0;

                //
                if(motor1_stop_time < 6){

                    temp_val = adc_value[1];

                    if(temp_val < motor1_up_move+10 && temp_val > motor1_up_move-10){ //adc_value[0]对比上次波动10以内，表示卡死
                         motor1_stop_time++;

                        //在一定时间内，变化很小，认为堵死。要停止电机。
                        if(motor1_stop_time > 5){
                                motor1_stop_time = 5+1;
                                motor1_t_run();
                        }
                    }else{
                        motor1_stop_time = 0;

                        log_info("motor 0000");
                    }

                    log_info("motor %d    %d     %d",temp_val,motor1_up_move,motor1_stop_time);


                    motor1_up_move = temp_val;
                }




        }


}
//m2
void motor2_dz_check(){
return;//先不检测
        if(++motor2_get_time == 50)
        {
            motor2_get_time = 0;

                //
                if(motor2_stop_time < 6)
                {

                    temp_val = adc_value[1];

                    if(temp_val < motor2_up_move+10 && temp_val > motor2_up_move-10){ //adc_value[0]对比上次波动10以内，表示卡死
                         motor2_stop_time++;

                        if(motor2_stop_time > 5){
                                motor2_stop_time = 5+1;
                                motor2_t_run();
                        }
                    }else{
                        motor2_stop_time = 0;

                        log_info("motor 0000");
                    }

                    log_info("motor %d    %d     %d",temp_val,motor2_up_move,motor2_stop_time);


                    motor2_up_move = temp_val;
                }




        }


}
//m3
void motor3_dz_check(){
return;//先不检测
        if(++motor3_get_time == 50)
        {
            motor3_get_time = 0;

                //
                if(motor3_stop_time < 6){

                    temp_val = adc_value[1];

                    if(temp_val < motor3_up_move+10 && temp_val > motor3_up_move-10){ //adc_value[0]对比上次波动10以内，表示卡死
                         motor3_stop_time++;

                        if(motor3_stop_time > 5){
                                motor3_stop_time = 5+1;
                                motor3_t_run();
                        }
                    }else{
                        motor3_stop_time = 0;

                        log_info("motor 0000");
                    }

                    log_info("motor %d    %d     %d",temp_val,motor3_up_move,motor3_stop_time);


                    motor3_up_move = temp_val;
                }




        }


}
//m4
void motor4_dz_check(){
return;//先不检测
        if(++motor4_get_time == 50)
        {
            motor4_get_time = 0;

                //
                if(motor4_stop_time < 6){

                    temp_val = adc_value[1];

                    if(temp_val < motor4_up_move+10 && temp_val > motor4_up_move-10){ //adc_value[0]对比上次波动10以内，表示卡死
                         motor4_stop_time++;

                        if(motor4_stop_time > 5){
                                motor4_stop_time = 5+1;
                                motor4_t_run();
                        }
                    }else{
                        motor4_stop_time = 0;

                        log_info("motor 0000");
                    }

                    log_info("motor %d    %d     %d",temp_val,motor4_up_move,motor4_stop_time);


                    motor4_up_move = temp_val;
                }




        }


}
//------------------------------停·正·反-------------------------------------------//
//b1
// void motor1_t_run(){//停止
//        B1_STOP;
// }

void motor1_t_run(){//停止
    motor1.running = 0;
    motor1.num = 0;
    motor1.val = 0;
    B1_STOP;
}

void motor1_b_run(){//停止
    motor1.running = 0;
    motor1.num = 0;
    motor1.val = 0;
    B1_BRAKE;
}

// void motor1_z_run(){//正转
//     motor1_dz_check();
//     if(motor1_stop_time > 5){//30次电位器波动在10内，停止
//         motor1_t_run();
//     }else
//     {
//        B1_Z;
//     }
// }

void motor1_z_run(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 10;       // 最大占空比
        motor1.num = 10;        // 从0开始缓启动
    }
}

void motor1_z_run_hqd(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 10;       // 最大占空比
        motor1.num = 0;        // 从0开始缓启动
    }
}

void motor1_z_run_hqd80(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 8;       // 最大占空比
        motor1.num = 0;        // 从0开始缓启动
    }
}

void motor1_z_run_60(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 6;       // 最大占空比
        motor1.num = 6;        // 从0开始缓启动
    }
}

void motor1_z_run_40(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 4;       // 最大占空比
        motor1.num = 4;        // 从0开始缓启动
    }
}

void motor1_z_run_30(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 3;       // 最大占空比
        motor1.num = 3;        // 从0开始缓启动
    }
}


void motor1_z_run_10(){//正转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 1;    // 正转
        motor1.val = 4;       // 最大占空比
        motor1.num = 1;        // 从0开始缓启动
    }
}
// void motor1_f_run(){
//     motor1_dz_check();
//     if(motor1_stop_time > 5){//30次电位器波动在10内，停止
//         motor1_t_run();
//     }else
//     {
//        B1_F;
//     }
// }

void motor1_f_run(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 10;       // 最大占空比
        motor1.num = 10;        // 从0开始缓启动
    }
}

void motor1_f_run_hqd(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 10;       // 最大占空比
        motor1.num = 0;        // 从0开始缓启动
    }
}

void motor1_f_run_hqd5(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 5;       // 最大占空比
        motor1.num = 1;        // 从0开始缓启动
    }
}

void motor1_f_run_60(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 6;       // 最大占空比
        motor1.num = 6;        // 从0开始缓启动
    }
}

void motor1_f_run_40(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 4;       // 最大占空比
        motor1.num = 4;        // 从0开始缓启动
    }
}

void motor1_f_run_30(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 3;       // 最大占空比
        motor1.num = 3;        // 从0开始缓启动
    }
}

void motor1_f_run_10(){//反转
    motor1_dz_check();
    if(motor1_stop_time > 5){//30次电位器波动在10内，停止
        motor1_t_run();
    }else
    {
        motor1.running = 1;
        motor1.direction = 0;    // 反转
        motor1.val = 1;       // 最大占空比
        motor1.num = 1;        // 从0开始缓启动
    }
}
//b2
// void motor2_t_run(){//停止
//        B2_STOP;
// }

void motor2_t_run(){//停止
    motor2.running = 0;
    motor2.num = 0;
    motor2.val = 0;
    B2_STOP;
}

void motor2_b_run(){//停止
    motor2.running = 0;
    motor2.num = 0;
    motor2.val = 0;
    B2_BRAKE;
}
// void motor2_z_run(){//正转
//     motor2_dz_check();
//     if(motor2_stop_time < 6){//30次电位器波动在10内，停止
//         B2_Z;
//     }
// }

void motor2_z_run(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 10;       // 最大占空比
        motor2.num = 10;        // 从0开始缓启动
    }
}

void motor2_z_run_hqd(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 10;       // 最大占空比
        motor2.num = 0;        // 从0开始缓启动
    }
}

void motor2_z_run_hqd50(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 5;       // 最大占空比
        motor2.num = 0;        // 从0开始缓启动
    }
}

void motor2_z_run_60(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 6;       // 最大占空比
        motor2.num = 6;        // 从0开始缓启动
    }
}

void motor2_z_run_40(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 4;       // 最大占空比
        motor2.num = 4;        // 从0开始缓启动
    }
}

void motor2_z_run_30(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 3;       // 最大占空比
        motor2.num = 3;        // 从0开始缓启动
    }
}


void motor2_z_run_10(){//正转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 1;    // 正转
        motor2.val = 1;       // 最大占空比
        motor2.num = 1;        // 从0开始缓启动
    }
}
// void motor2_f_run(){//反转
//     motor2_dz_check();
//     if(motor2_stop_time < 6){//30次电位器波动在10内，停止
//         B2_F;
//     }
// }

void motor2_f_run(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 10;       // 最大占空比
        motor2.num = 10;        // 从0开始缓启动
    }
}

void motor2_f_run_hqd(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 10;       // 最大占空比
        motor2.num = 0;        // 从0开始缓启动
    }
}

void motor2_f_run_hqd5(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 5;       // 最大占空比
        motor2.num = 0;        // 从0开始缓启动
    }
}


void motor2_f_run_60(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 6;       // 最大占空比
        motor2.num = 6;        // 从0开始缓启动
    }
}

void motor2_f_run_40(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 4;       // 最大占空比
        motor2.num = 4;        // 从0开始缓启动
    }
}

void motor2_f_run_30(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 3;       // 最大占空比
        motor2.num = 3;        // 从0开始缓启动
    }
}

void motor2_f_run_10(){//反转
    motor2_dz_check();
    if(motor2_stop_time < 6){//30次电位器波动在10内，停止
        motor2.running = 1;
        motor2.direction = 0;    // 反转
        motor2.val = 1;       // 最大占空比
        motor2.num = 1;        // 从0开始缓启动
    }
}
//b3
// void motor3_t_run(){//停止
//        B3_STOP;
// }
void motor3_t_run(){//停止
    motor3.running = 0;
    motor3.num = 0;
    motor3.val = 0;
    B3_STOP;
}

void motor3_b_run(){//停止
    motor3.running = 0;
    motor3.num = 0;
    motor3.val = 0;
    B3_BRAKE;
}
// void motor3_z_run(){//正转
//     motor3_dz_check();
//     if(motor3_stop_time > 5){//30次电位器波动在10内，停止
//         motor3_t_run();
//     }else
//     {
//        B3_Z;
//     }
// }

void motor3_z_run(){//正转
    // motor3_dz_check();
    // if(motor3_stop_time > 5){//30次电位器波动在10内，停止
    //     motor3_t_run();
    // }else
    // {
        motor3.running = 1;
        motor3.direction = 1;    // 正转
        motor3.val = 10;       // 最大占空比
        motor3.num = 10;        // 从0开始缓启动
    // }
}

void motor3_z_run_hqd(){//正转
    // motor3_dz_check();
    // if(motor3_stop_time > 5){//30次电位器波动在10内，停止
    //     motor3_t_run();
    // }else
    // {
        motor3.running = 1;
        motor3.direction = 1;    // 正转
        motor3.val = 10;       // 最大占空比
        motor3.num = 0;        // 从0开始缓启动
    // }
}

void motor3_z_run_60(){//正转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else
    {
        motor3.running = 1;
        motor3.direction = 1;    // 正转
        motor3.val = 6;       // 最大占空比
        motor3.num = 6;        // 从0开始缓启动
    }
}

void motor3_z_run_40(){//正转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else
    {
        motor3.running = 1;
        motor3.direction = 1;    // 正转
        motor3.val = 4;       // 最大占空比
        motor3.num = 4;        // 从0开始缓启动
    }
}

void motor3_z_run_30(){//正转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else
    {
        motor3.running = 1;
        motor3.direction = 1;    // 正转
        motor3.val = 3;       // 最大占空比
        motor3.num = 3;        // 从0开始缓启动
    }
}

void motor3_z_run_10(){//正转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else
    {
        motor3.running = 1;
        motor3.direction = 1;    // 正转
        motor3.val = 1;       // 最大占空比
        motor3.num = 1;        // 从0开始缓启动
    }
}
// void motor3_f_run(){//反转
//     motor3_dz_check();
//     if(motor3_stop_time > 5){//30次电位器波动在10内，停止
//         motor3_t_run();
//     }else{
//        B3_F;
//     }
// }

void motor3_f_run(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 10;       // 最大占空比
        motor3.num = 10;        // 从0开始缓启动
    }
}

void motor3_f_run_hqd(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 10;       // 最大占空比
        motor3.num = 0;        // 从0开始缓启动
    }
}

void motor3_f_run_hqd5(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 5;       // 最大占空比
        motor3.num = 0;        // 从0开始缓启动
    }
}

void motor3_f_run_60(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 6;       // 最大占空比
        motor3.num = 6;        // 从0开始缓启动
    }
}

void motor3_f_run_40(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 4;       // 最大占空比
        motor3.num = 4;        // 从0开始缓启动
    }
}

void motor3_f_run_30(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 3;       // 最大占空比
        motor3.num = 3;        // 从0开始缓启动
    }
}

void motor3_f_run_20(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 2;       // 最大占空比
        motor3.num = 2;        // 从0开始缓启动
    }
}

void motor3_f_run_10(){//反转
    motor3_dz_check();
    if(motor3_stop_time > 5){//30次电位器波动在10内，停止
        motor3_t_run();
    }else{
        motor3.running = 1;
        motor3.direction = 0;    // 反转
        motor3.val = 1;       // 最大占空比
        motor3.num = 1;        // 从0开始缓启动
    }
}
//b4
// void motor4_t_run(){//停止
//        B4_STOP;
// }
void motor4_t_run(){//停止
    motor4.running = 0;
    motor4.num = 0;
    motor4.val = 0;
    B4_STOP;
}

void motor4_b_run(){//停止
    motor4.running = 0;
    motor4.num = 0;
    motor4.val = 0;
    B4_BRAKE;
}
// void motor4_z_run(){//正转
//     motor4_dz_check();
//     if(motor4_stop_time > 5){//30次电位器波动在10内，停止
//         motor4_t_run();
//     }else
//     {
//        B4_Z;
//     }
// }

void motor4_z_run_hqd(){//正转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else
    {
        motor4.running = 1;
        motor4.direction = 1;    // 正转
        motor4.val = 8;       // 最大占空比
        motor4.num = 0;        // 从0开始缓启动
    }
}

void motor4_z_run_60(){//正转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else
    {
        motor4.running = 1;
        motor4.direction = 1;    // 正转
        motor4.val = 6;       // 最大占空比
        motor4.num = 6;        // 从0开始缓启动
    }
}

void motor4_z_run_40(){//正转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else
    {
        motor4.running = 1;
        motor4.direction = 1;    // 正转
        motor4.val = 4;       // 最大占空比
        motor4.num = 4;        // 从0开始缓启动
    }
}

void motor4_z_run_30(){//正转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else
    {
        motor4.running = 1;
        motor4.direction = 1;    // 正转
        motor4.val = 3;       // 最大占空比
        motor4.num = 3;        // 从0开始缓启动
    }
}

void motor4_z_run_10(){//正转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else
    {
        motor4.running = 1;
        motor4.direction = 1;    // 正转
        motor4.val = 1;       // 最大占空比
        motor4.num = 1;        // 从0开始缓启动
    }
}
// void motor4_f_run(){//反转

//     motor4_dz_check();
//     if(motor4_stop_time > 5){//30次电位器波动在10内，停止
//         motor4_t_run();
//     }else{
//        B4_F;
//     }
// }

void motor4_f_run_hqd(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 8;       // 最大占空比
        motor4.num = 0;        // 从0开始缓启动
    }
}

void motor4_f_run_hqd5(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 5;       // 最大占空比
        motor4.num = 0;        // 从0开始缓启动
    }
}

void motor4_f_run_60(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 6;       // 最大占空比
        motor4.num = 6;       
    }
}

void motor4_f_run_40(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 4;       // 最大占空比
        motor4.num = 4;        
    }
}

void motor4_f_run_30(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 3;       // 最大占空比
        motor4.num = 3;        
    }
}

void motor4_f_run_20(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 2;       // 最大占空比
        motor4.num = 2;        
    }
}

void motor4_f_run_10(){//反转
    motor4_dz_check();
    if(motor4_stop_time > 5){//30次电位器波动在10内，停止
        motor4_t_run();
    }else{
        motor4.running = 1;
        motor4.direction = 0;    // 反转
        motor4.val = 1;       // 最大占空比
        motor4.num = 1;        
    }
}

void stop_all_motors(void) {
    motor1_t_run();
    motor2_t_run();
    motor3_t_run();
    motor4_t_run();
}

//--------------------------------------------------------------------//
// 电机控制函数
void update_motor_pwm() {
    // 电机1控制
    if (motor1.running) {
        if (motor1.direction) {
            if (motor1.pwm) {
                B1_Z_ON;
                B1_F_OFF;
            } else {
                B1_Z_OFF;
                B1_F_OFF;
            }
        } else {
            if (motor1.pwm) {
                B1_F_ON;
                B1_Z_OFF;
            } else {
                B1_F_OFF;
                B1_Z_OFF;
            }
        }
    } else {
        B1_STOP;
    }
    
    // 电机2控制
    if (motor2.running) {
        if (motor2.direction) {
            if (motor2.pwm) {
                B2_Z_ON;
                B2_F_OFF;
            } else {
                B2_Z_OFF;
                B2_F_OFF;
            }
        } else {
            if (motor2.pwm) {
                B2_F_ON;
                B2_Z_OFF;
            } else {
                B2_F_OFF;
                B2_Z_OFF;
            }
        }
    } else {
        B2_STOP;
    }
    
    // 电机3控制
    if (motor3.running) {
        if (motor3.direction) {
            if (motor3.pwm) {
                B3_Z_ON;
                B3_F_OFF;
            } else {
                B3_Z_OFF;
                B3_F_OFF;
            }
        } else {
            if (motor3.pwm) {
                B3_F_ON;
                B3_Z_OFF;
            } else {
                B3_F_OFF;
                B3_Z_OFF;
            }
        }
    } else {
        B3_STOP;
    }
    
    // 电机4控制
    if (motor4.running) {
        if (motor4.direction) {
            if (motor4.pwm) {
                B4_Z_ON;
                B4_F_OFF;
            } else {
                B4_Z_OFF;
                B4_F_OFF;
            }
        } else {
            if (motor4.pwm) {
                B4_F_ON;
                B4_Z_OFF;
            } else {
                B4_F_OFF;
                B4_Z_OFF;
            }
        }
    } else {
        B4_STOP;
    }
}

//--------------------------------------------------------------------//
void  motor_init(void)
{
	//电机初始化
	B1_STOP;
	B1_Z_CONFIG;
	B1_F_CONFIG;

	B2_STOP;
	B2_Z_CONFIG;
	B2_F_CONFIG;

	B3_STOP;
	B3_Z_CONFIG;
	B3_F_CONFIG;

}

void motor_ctrl(void)
{
    if (remote_control_status != last_remote_control_status)
    {
        last_remote_control_status = remote_control_status;
        action_completed = false;
        ir_change_count = 0;
        motor_started[0] = false;           //电机复位

        // 编程模式处理逻辑
        if (remote_control_status == remote_control_program)
        {
            switch (program_mode)
            {
                case PROGRAM_MODE_IDLE: //  空闲状态处理
                    // 进入记录模式
                    program_mode = PROGRAM_MODE_RECORDING; //  切换到记录模式
                    record_index = 0;  // 重置记录索引
                    log_info("enter program\n");
                    break;
                    
                case PROGRAM_MODE_RECORDING: //  记录模式处理
                    // 退出记录模式，开始播放
                    program_mode = PROGRAM_MODE_PLAYING; //  切换到播放模式
                    play_index = 0;    // 重置播放索引
                    log_info("execute program\n");
                    break;
                    
                case PROGRAM_MODE_PLAYING: /* * 处理编程模式下的按键逻辑 * 当处于播放模式时，再次按下编程键会停止播放 * 当处于记录模式时，按键会被记录但不执行 */
                    // 播放中再次按下编程键，停止播放
                    program_mode = PROGRAM_MODE_IDLE; //  将编程模式切换为空闲状态
                    current_motor_state = MOTOR_REMOTE_STOP; //  设置电机状态为远程停止
                    record_index = 0;  // 重置记录索引（视为清空）
                    play_index = 0;    // 重置播放索引
                    log_info("stop program and clear record\n");  // 更新日志提示
                    break;
            }
            return;  // 编程键不参与动作记录
        }

        // 记录模式下，存储动作而不执行
        if (program_mode == PROGRAM_MODE_RECORDING) //  判断当前是否处于记录模式
        {
            // 优先处理停止键：直接退出编程模式，不记录
            if (remote_control_status == remote_control_head_reset)
            {
                program_mode = PROGRAM_MODE_IDLE;
                record_index = 0;
                play_index = 0;
                memset(record_buffer, 0, sizeof(record_buffer)); // 清空缓冲区
                current_motor_state = MOTOR_REMOTE_STOP;
                log_info("stop key in record mode: exit program\n");
                return; // 立即返回，不执行后续记录逻辑
            }
            // 检查是否是有效的动作按键
            bool is_action_key = false;
            switch (remote_control_status)
            {
                case remote_control_forward:
                case remote_control_forward_20:
                case remote_control_back:
                case remote_control_back_20:
                case remote_control_left_forward:
                case remote_control_right_forward:
                case remote_control_left_back:
                case remote_control_right_back:
                // case remote_control_head_reset:
                case remote_control_head_left:
                case remote_control_head_right:
                    is_action_key = true;
                    break;
                // case remote_control_head_reset:
                //     is_action_key = false;
                //     break;
                default:
                    is_action_key = false;
                    break;
            }
            
            // 如果是有效动作且缓冲区未满，则记录
            if (is_action_key && record_index < MAX_RECORD_LENGTH)
            {
                record_buffer[record_index++] = remote_control_status;
                log_info("record action: %d, total of %d action\n", remote_control_status, record_index);
            }
            return;  // 记录模式下不执行动作
        }

        // 非编程模式或编程播放模式下，正常处理遥控指令
        switch (remote_control_status)
        {
        case remote_control_forward:
            if (remote_control_distance_flag)
            {
                current_motor_state = MOTOR_REMOTE_STATE_FORWARD_20;
            }else
            {
                current_motor_state = MOTOR_REMOTE_STATE_FORWARD;
            }
            break;

        case remote_control_forward_20:
            current_motor_state = MOTOR_REMOTE_STATE_FORWARD_20;
            break;

        case remote_control_back:
            if (remote_control_distance_flag)
            {
                current_motor_state = MOTOR_REMOTE_STATE_BACK_20;
            }else
            {
                current_motor_state = MOTOR_REMOTE_STATE_BACK;
            }
            break;

        case remote_control_back_20:
            current_motor_state = MOTOR_REMOTE_STATE_BACK_20;
            break;

        case remote_control_left_forward:
            if (remote_control_angle_flag)
            {
                current_motor_state = MOTOR_REMOTE_STATE_LEFT_forward_45;
            }else
            {
                current_motor_state = MOTOR_REMOTE_STATE_LEFT_forward;
            }
            break;
            
        case remote_control_right_forward:
            if (remote_control_angle_flag)
            {
                current_motor_state = MOTOR_REMOTE_STATE_RIGHT_forward_45;
            }else
            {
                current_motor_state = MOTOR_REMOTE_STATE_RIGHT_forward;
            }
            break;

        case remote_control_left_back:
            current_motor_state = MOTOR_REMOTE_STATE_LEFT_back;
            break;

        case remote_control_right_back:
            current_motor_state = MOTOR_REMOTE_STATE_RIGHT_back;
            break;

        case remote_control_demo:
            current_motor_state = MOTOR_REMOTE_demo;
            break;

        case remote_control_head_reset:
            // current_motor_state = MOTOR_REMOTE_HEAD_RESET;
            // 仅在非记录模式下执行头部复位（记录模式下已被拦截为停止键）
            if (program_mode != PROGRAM_MODE_RECORDING)
            {
                current_motor_state = MOTOR_REMOTE_HEAD_RESET;
            }
            break;

        case remote_control_head_left:
            current_motor_state = MOTOR_REMOTE_HEAD_LEFT;
            break;

        case remote_control_head_right:
            current_motor_state = MOTOR_REMOTE_HEAD_RIGHT;
            break;

        case remote_control_distance_switch:   
            // 取反距离控制标志位
            remote_control_distance_flag = !remote_control_distance_flag;
            break;

        case remote_control_angle_switch:
            // 取反角度控制标志位
            remote_control_angle_flag = !remote_control_angle_flag;
            break;

        case remote_control_stop:
            log_info("data2 stop\n");
            current_motor_state = MOTOR_REMOTE_STOP;

            if (program_mode == PROGRAM_MODE_RECORDING || program_mode == PROGRAM_MODE_PLAYING)
            {
                program_mode = PROGRAM_MODE_IDLE;  
                record_index = 0;                  
                play_index = 0;                    
                memset(record_buffer, 0, sizeof(record_buffer)); // 优化清空方式
                log_info("stop key clears program and resets\n");
            }
            break;

        case receive_remove_control_idle:
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            break;

        default:
            // current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            break;
        }
    }

    // 编程播放模式处理
    if (program_mode == PROGRAM_MODE_PLAYING)
    { 
        // 打印当前状态，方便调试
        log_info("Playing: state=%d, play_idx=%d, record_idx=%d\n", 
            current_motor_state, play_index, record_index);

        // 只有当前动作完成（空闲）且有未执行动作时，才执行下一个
        if (current_motor_state == MOTOR_REMOTE_STATE_IDLE && play_index < record_index)
        {
            u8 recorded_action = record_buffer[play_index++];
            log_info("execute action %d/%d: %d\n", play_index, record_index, recorded_action);
            // 设置新动作前，强制重置完成标志
            action_completed = false;
            
            // 根据记录的动作设置电机状态
            switch (recorded_action)
            {
                case remote_control_forward:
                    current_motor_state = MOTOR_REMOTE_STATE_FORWARD;
                    break;
                case remote_control_forward_20:
                    current_motor_state = MOTOR_REMOTE_STATE_FORWARD_20;
                    break;
                case remote_control_back:
                    current_motor_state = MOTOR_REMOTE_STATE_BACK;
                    break;
                case remote_control_back_20:
                    current_motor_state = MOTOR_REMOTE_STATE_BACK_20;
                    break;
                case remote_control_left_forward:
                    current_motor_state = MOTOR_REMOTE_STATE_LEFT_forward;
                    break;
                case remote_control_right_forward:
                    current_motor_state = MOTOR_REMOTE_STATE_RIGHT_forward;
                    break;
                case remote_control_left_back:
                    current_motor_state = MOTOR_REMOTE_STATE_LEFT_back;
                    break;
                case remote_control_right_back:
                    current_motor_state = MOTOR_REMOTE_STATE_RIGHT_back;
                    break;
                case remote_control_head_reset:
                    current_motor_state = MOTOR_REMOTE_HEAD_RESET;
                    break;
                case remote_control_head_left:
                    current_motor_state = MOTOR_REMOTE_HEAD_LEFT;
                    break;
                case remote_control_head_right:
                    current_motor_state = MOTOR_REMOTE_HEAD_RIGHT;
                    break;
                default:
                    break;
            }
            
            // 如果所有动作都已执行完毕，退出播放模式
            if (play_index >= record_index)
            {
                program_mode = PROGRAM_MODE_IDLE;
                log_info("execution complete\n");
            }
        }
    }

    //底部电机控制
    switch (current_motor_state)
    {
    case MOTOR_REMOTE_STATE_FORWARD:
        motor_forward_check_10cm();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Forward 10cm completed\n");  // 增加调试日志
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_FORWARD_20:
        motor_forward_check();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Forward 20cm completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_BACK:
        motor_back_check_10cm();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Back 10cm completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_BACK_20:
        motor_back_check();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Back 20cm completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    // 左转、右转等其他动作同理，增加完成日志和状态切换检查
    case MOTOR_REMOTE_STATE_LEFT_forward:
        motor_left_forward_check_90();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Left forward completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_RIGHT_forward:
        motor_right_forward_check_90();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Right forward completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_LEFT_forward_45:
        motor_left_forward_check_45();
        log_info("left 45\n");
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_RIGHT_forward_45:
        motor_right_forward_check_45();
        log_info("right 45\n");
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_LEFT_back:
        motor_left_back_check_90();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STATE_RIGHT_back:
        motor_right_back_check_90();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_demo:
        motor_demo_execute();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;
    
    case MOTOR_REMOTE_HEAD_RESET:
        //清除编程状态
        program_mode = PROGRAM_MODE_IDLE;               //  将编程模式切换为空闲状态
        current_motor_state = MOTOR_REMOTE_STOP;        //  设置电机状态为远程停止
        record_index = 0;                               //  重置记录索引（视为清空）
        play_index = 0;                                 //  重置播放索引
        demo_in_progress = false;                       //  重置演示状态
        log_info("stop program and clear record\n");    //  更新日志提示
        stop_all_motors();
        current_motor_state = MOTOR_REMOTE_STATE_IDLE;
        need_reset_status = TRUE;
        // 检查复位是否完成
        if (ORIGIN_FINISH_STATUS == 1) {
            action_completed = true;
        }

        if (action_completed)
        {
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    // 其他动作（头左转、头右转等）同样增加完成日志和状态切换
    case MOTOR_REMOTE_HEAD_LEFT:
        motor_head_rotate_left_to_angle();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Head left completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_HEAD_RIGHT:
        motor_head_rotate_right_to_angle();
        if (action_completed)
        {
            UT1_write_buf(Uart_stop_code, 10);
            log_info("Head right completed\n");
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
            action_completed = false;
        }
        break;

    case MOTOR_REMOTE_STOP:
            log_info("stop \n");
            stop_all_motors();
            UT1_write_buf(Uart_stop_code, 10);
            // UT1_write_buf(Uart_stop_code, 10);
            action_completed = true;  // 标记动作为完成
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
        break;
        

    case MOTOR_REMOTE_STATE_IDLE:
        /* code */
        break;
    
    default:
        break;
    }
    return;
}



void motor_forward(void)
{   
    motor1_z_run();
    motor2_z_run();
    return;
}

void motor_back(void)
{
    motor1_f_run();
    motor2_f_run();
    return;
}

void motor_motor2_forward(void)
{
    motor1_t_run();
    motor2_z_run();
    return;
}

void motor_motor2_back(void)
{ 
    motor1_t_run();
    motor2_f_run();
    return;
}


void motor_motor1_forward(void)
{
    motor1_z_run();
    motor2_t_run();
    return;
}

void motor_motor1_back(void)
{
    motor1_f_run();
    motor2_t_run();
    return;
}

void motor_forward_check(void)
{   
    if (action_completed)
    {
        return;
    }
    //20cm
    motor_forward();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 75) {
        // log_info("IR change count reached, stopping motors");
        motor1_t_run();
        motor2_t_run();
        // forward_complete_flag = true;
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

void motor_forward_check_10cm(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_forward();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 38) {
        // log_info("IR change count reached, stopping motors");
        motor1_t_run();
        motor2_t_run();
        // forward_complete_flag = true;
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

//后退
void motor_back_check(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_back();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 75) {
        // log_info("IR change count reached, stopping motors");
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

void motor_back_check_10cm(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_back();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 38) {
        // log_info("IR change count reached, stopping motors");
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

//左前90°
void motor_left_forward_check_90(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor1_forward();
    u32 current_ir = adc_value[0];  

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 42) {
        // log_info("IR change count reached, stopping motors");
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

//左前45°
void motor_left_forward_check_45(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor1_forward();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 21) {
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}


//右后90°
void motor_right_back_check_90(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor1_back();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }

    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 45) {
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

//右后45°
void motor_right_back_check_45(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor1_back();
    u32 current_ir = adc_value[0]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 21) {
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

//右前90°
void motor_right_forward_check_90(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor2_forward();
    u32 current_ir = adc_value[1]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }

    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 50) {
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

//右前45°
void motor_right_forward_check_45(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor2_forward();
    u32 current_ir = adc_value[1]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 25) {
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}


//左后
void motor_left_back_check_90(void)
{   
    if (action_completed)
    {
        return;
    }
    
    motor_motor2_back();
    u32 current_ir = adc_value[1]; 

    // 检测到红外值变化（从非遮挡到遮挡）
    if (last_ir_value < IR_THRESHOLD && current_ir >= IR_THRESHOLD) {
        ir_change_count++;
    }
    last_ir_value = current_ir;

    // 达到变化次数阈值，停止前进
    if (ir_change_count >= 50) {
        motor1_t_run();
        motor2_t_run();
        ir_change_count = 0;
        action_completed = true;
    }
    return;
}

void motor_demo_execute(void)
{
    // 从tick_timer_sys.c获取50ms标志位
    extern u8 tick_50ms_cnt;

    // 仅在演示模式下执行
    if (current_motor_state != MOTOR_REMOTE_demo) {
        // 退出演示模式时重置状态
        if (demo_in_progress) {
            current_demo_step = DEMO_STEP_COMPLETE;
            demo_action_timer = 0;
            demo_in_progress = false;
            current_motor_state = MOTOR_REMOTE_STATE_IDLE;
        }
        return;
    }

    if (!demo_in_progress) {
        log_info("Start demo execution\n");
        current_demo_step = DEMO_STEP_FORWARD;
        demo_in_progress = true;
        motor_forward();
        log_info("Demo step: Forward\n");   // 补充第一个动作日志
        tick_50ms_cnt = 0;                  // 第一个动作开始时清零计数器
        return;
    }
    // 仅对基础动作（前/后/左/右）使用定时切换，头部动作通过完成标志切换
    if (current_demo_step < DEMO_STEP_HEAD_LEFT) {
        // 检查计数器是否达到40（40*50ms=2000ms=2秒）
        if (tick_50ms_cnt >= 40) {
            tick_50ms_cnt = 0;              // 清零计数器
            current_demo_step++;            // 切换到下一步

            switch (current_demo_step) {
                case DEMO_STEP_BACK:
                    motor_back();
                    log_info("Demo step: Backward\n");
                    break;
                case DEMO_STEP_LEFT:
                    motor_motor2_forward();
                    log_info("Demo step: Left turn\n");
                    break;
                case DEMO_STEP_RIGHT:
                    motor_motor1_forward();
                    log_info("Demo step: Right turn\n");
                    break;
                default:
                    break;
            }
        }
    }
    // =================================================================================
    else {
        switch (current_demo_step) {
            // 头部左转：等待到达指定位置后切换
            case DEMO_STEP_HEAD_LEFT:
                // 确保基础电机已停止
                motor1_t_run();
                motor2_t_run();
                motor_head_rotate_left_to_angle();      // 执行左转到指定位置
                
                // 首次进入该步骤时启动动作
                static bool head_left_started = false;

                if (!head_left_started) {
                    motor_started[0] = false;           //电机复位
                    action_completed = false;
                    head_left_started = true;
                    log_info("Demo step: Head left (executing)\n");
                }
                
                // 检查是否到达指定位置（通过action_completed标志）
                if (action_completed) {
                    head_left_started = false;          // 重置标志
                    action_completed = false;           // 清除完成标志
                    current_demo_step++;                // 进入下一步
                    log_info("Demo step: Head left (completed)\n");
                }
                break;

            // 头部右转：等待到达指定位置后切换
            case DEMO_STEP_HEAD_RIGHT:
                // 首次进入该步骤时启动动作
                motor_head_rotate_right_to_angle();     // 执行右转到指定位置
                static bool head_right_started = false;
                if (!head_right_started) {
                    motor_started[0] = false;           //电机复位
                    action_completed = false;
                    head_right_started = true;
                    log_info("Demo step: Head right (executing)\n");
                }
                
                // 检查是否到达指定位置
                if (action_completed) {
                    motor_started[0] = false;           //电机复位
                    head_right_started = false;
                    action_completed = false;
                    current_demo_step++;
                    log_info("Demo step: Head right (completed)\n");
                }
                break;

            // 头部复位：等待复位完成后结束
            case DEMO_STEP_HEAD_RESET:
                // 首次进入该步骤时启动复位
                static bool head_reset_started = false;
                if (!head_reset_started) {
                    motor_started[0] = false;           //电机复位
                    need_reset_status = TRUE;
                    action_completed = false;
                    head_reset_started = true;
                    log_info("Demo step: Head reset (executing)\n");
                }
                
                // 检查复位是否完成（通过ORIGIN_FINISH_STATUS标志）
                if (ORIGIN_FINISH_STATUS == 1) {
                    head_reset_started = false;
                    action_completed = false;
                    current_demo_step++;            // 进入完成步骤
                    log_info("Demo step: Head reset (completed)\n");
                }
                break;

            // 演示完成
            case DEMO_STEP_COMPLETE:
                current_motor_state = MOTOR_REMOTE_STATE_IDLE;
                demo_in_progress = false;
                log_info("Demo execution complete\n");
                break;

            default:
                break;
        }
    } 
}


void motor_head_rotate_left(void)
{
    //头部左旋转
    motor3_z_run_hqd();
    return;
}

/**
 * @brief 控制头部电机向左旋转到指定角度
 * 
 * 该函数用于控制头部电机(电机3)向左旋转到预设角度。函数会检查电机复位状态和延迟计数，
 * 确保安全启动。当电机启动后，会持续监测ADC值来判断是否达到目标角度。
 * 
 * @note 函数依赖以下全局变量：
 *       - ORIGIN_FINISH_STATUS: 复位完成状态标志
 *       - ORIGIN_FINISH_DELAY_COUNT: 复位延迟计数
 *       - motor_started[0]: 电机启动状态
 *       - motor_reached[0]: 电机到达目标状态
 *       - adc_value[3]: 电机3的ADC采样值
 * 
 * @return 无返回值
 */
void motor_head_rotate_left_to_angle(void)
{   
    // // 检查复位状态和延迟计数
    if (action_completed)
    {
        return;
    }
    
    // 检查电机是否已启动
    if (!motor_started[0]) {
        motor3_z_run_hqd();
        log_info("dj3:%d", adc_value[2]);
        log_info("left process");
        motor_started[0] = true; 
    }
    // 检查电机是否到达目标角度
    if (adc_value[2] > 410) {
        // 低于阈值，继续旋转
    } else {        
        log_info("left complete");
        log_info("dj3:%d", adc_value[2]);
        motor3_t_run();
        action_completed = true;
    }
    return;
}

void motor_head_rotate_right(void)
{
    //头部右旋转
    motor3_f_run_hqd();
    return;
}

/**
 * @brief 控制头部电机向右旋转到指定角度
 * 
 * 该函数控制电机3(头部电机)向右旋转，直到达到目标角度。
 * 通过ADC值来判断是否到达目标位置，当ADC值超过阈值时停止旋转。
 * 
 * @note 函数会检查复位状态和延迟计数，确保系统处于就绪状态
 * @note 使用motor_started和motor_reached数组来跟踪电机状态
 * 
 * @param void 无参数
 * @return void 无返回值
 */
void motor_head_rotate_right_to_angle(void)
{
    if (action_completed)
    {
        return;
    }
    
    if (!motor_started[0]) {
        motor3_f_run_hqd();
        log_info("dj3:%d", adc_value[2]);
        log_info("right process");
        motor_started[0] = true; 
    }

    // 检查是否到达目标位置
    if (adc_value[2] < 690) {
        // 低于阈值，继续旋转
    } else {        
        // 达到阈值，停止旋转
        log_info("Right rotation completed");
        log_info("Final Motor 3 ADC value: %d", adc_value[2]);
        motor3_t_run();
        action_completed = true;
    }
    return;
}

/**
 * @brief 电机头部回归原点函数
 * 
 * 该函数控制电机头部回归到原点位置。通过ADC值检测当前位置，
 * 控制电机正转或反转直到到达原点范围。函数包含状态机控制、
 * 滤波处理和稳定确认机制。
 * 
 * @note 函数会检查所有电机是否都已归位，全部归位后设置完成标志
 * 
 * @param void 无参数
 * @return void 无返回值
 */
void motor_head_return(void)
{
    //头部回归
    if (need_reset_status) {
        // 清除复位完成标志
        ORIGIN_FINISH_STATUS = 0;
        for (u8 i = 0; i < FOOT_COUNT; i++) {
            foot_status[i].is_homed = false;
            foot_status[i].continuous_count = 0;
            foot_status[i].filter_count = 0;
            foot_status[i].state = MOTOR_STATE_STOP;
        }
        // 清除需要复位标志（只需清除一次）
        need_reset_status = false;
    }

    //完成时跳过复位
    if (ORIGIN_FINISH_STATUS == 1) {
        return;
    }

    // 遍历每个电位器，更新独立状态
    for (u8 i = 0; i < FOOT_COUNT; i++) 
    {
        // u32 ad_val = adc_value[i];
        u32 ad_val = adc_value[2];

        //获取当前电位器的独立原点参数
        u32 origin = foot_origin[i].origin_value;
        u32 tolerance = foot_origin[i].tolerance;
        bool in_origin = (ad_val >= origin - tolerance) && 
                         (ad_val <= origin + tolerance);
                    
        // 处于原点范围：停止电机并确认稳定
        if (in_origin) 
        {
            // 如果当前不是停止状态，则更新状态并停止电机
            if (foot_status[i].state != MOTOR_STATE_STOP) 
            {
                foot_status[i].state = MOTOR_STATE_STOP;
                // 执行停止操作
                switch (i) 
                {
                    case 0: motor3_t_run(); break;
                }
            }

            foot_status[i].filter_count = 0;  // 重置滤波计数
            /* 停止后进行稳定确认 - 无论是否状态变化都计数 */
            foot_status[i].continuous_count++;
            if (foot_status[i].continuous_count >= CONTINUOUS_CHECK) 
            {
                foot_status[i].is_homed = true;  // 确认最终归位
                foot_status[i].continuous_count = 0;
            }
        }
            // 偏离原点
            else 
            {
                foot_status[i].continuous_count = 0; // 重置稳定计数
                foot_status[i].is_homed = false;     // 确保未归位

                // 计算当前位置与原点范围的偏差
                bool need_forward = (ad_val < (origin - tolerance));
                bool need_reverse = (ad_val > (origin + tolerance));

                // 仅在需要移动时处理方向判断
                if (need_forward || need_reverse) 
                {
                    // 连续检测方向需求
                    if (++foot_status[i].filter_count >= FILTER_THRESHOLD) 
                    {    
                        // 关键修改：即使已在运动状态，也重新检查方向需求
                        if (need_forward) 
                        {
                            // 如果当前不是正转状态，则改变方向
                            if (foot_status[i].state != MOTOR_STATE_FORWARD) 
                            {
                                foot_status[i].state = MOTOR_STATE_FORWARD;
                                // 启动对应电机正转
                                switch (i) 
                                {
                                    case 0: motor3_f_run(); break;
                                }
                            }
                        }
                        else if (need_reverse) 
                        {
                            // 如果当前不是反转状态，则改变方向
                            if (foot_status[i].state != MOTOR_STATE_REVERSE) 
                            {
                                foot_status[i].state = MOTOR_STATE_REVERSE;
                                // 启动对应电机反转
                                switch (i) 
                                {
                                    case 0: motor3_z_run(); break;
                                }
                            }
                        }
                        foot_status[i].filter_count = 0;  // 执行后重置计数
                    }
                } 
                else 
                {
                    // 当前位置在原点附近但未达到原点条件，重置滤波计数
                    foot_status[i].filter_count = 0;
                    
                    /* 仅在确实不需要移动时停止 */
                    // 如果当前在移动但不需要移动，则停止
                    if (foot_status[i].state != MOTOR_STATE_STOP) 
                    {
                        foot_status[i].state = MOTOR_STATE_STOP;
                        switch (i) 
                        {
                            case 0: motor3_t_run(); break;
                        }
                    }
                }
            }
        }  // end for循环

         // 检查所有脚是否均已归位 (移到循环外部)
         bool all_homed = true;
         for (u8 i = 0; i < FOOT_COUNT; i++) 
         {
             if (!foot_status[i].is_homed) 
             {
                 all_homed = false;
                 break;
             }
         }
         
         if (all_homed) 
         {
            ORIGIN_FINISH_STATUS = 1; // 完成标志位
            ORIGIN_FINISH_DELAY_COUNT = 1;
            log_info("reset complete");
            log_info("dj3:%d ", adc_value[2]);
            
            // 所有电机停止
            for (u8 i = 0; i < FOOT_COUNT; i++) 
            {
                foot_status[i].state = MOTOR_STATE_STOP;
                switch (i) 
                {
                    case 0: motor3_t_run(); break;
                }
             }
         }
}

//===========================================================================//
//LED
//===========================================================================//
void LED_Init()
{
    //LED初始化
    LED_OFF;
	LED_CONFIG;
    LED_ON;
}

void LED_flash_Control()
{
    if (LED_flash_flag)
    {
        LED_ON;
    }
    else{
        LED_OFF;
    }
}
//===========================================================================//