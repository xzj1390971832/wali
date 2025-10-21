#ifndef _UART_MY_H_
#define _UART_MY_H_








void usr_uart_init(void);
void uart_send();

void usr_uart0_init(void);


void uart1_data_handler(void);
void usr_uart_24data_process(u8*data);
void uart1_write_data(const u8 *buf, u32 len);
u8 rf_key();

typedef enum {
    //2.4g
    receive_remove_control_idle,        //空闲
    remote_control_forward,             // 前进
    remote_control_back,                // 后退
    remote_control_left_forward,        // 左转
    remote_control_right_forward,       // 右转
    remote_control_left_back,           // 左转
    remote_control_right_back,          // 右转
    remote_control_forward_20,          // 前进20
    remote_control_back_20,             // 后退20
    remote_control_head_reset,          // 头部复位
    remote_control_head_left,           // 头部左转
    remote_control_head_right,          // 头部右转
    remote_control_program,             // 编程
    remote_control_demo,                // 演示
    remote_control_stop,                // 停止
    

    remote_control_distance_switch,     //距离切换
    remote_control_angle_switch,        //角度切换
    //ble
    //ble左
    remote_control_ble_left_up,         // 左上
    remote_control_ble_left_down,       // 左下
    remote_control_ble_left_left,       // 左左
    remote_control_ble_left_right,      // 左右
    //ble右
    remote_control_ble_right_up,        // 右上
    remote_control_ble_right_down,      // 右下
    remote_control_ble_right_left,      // 右左
    remote_control_ble_right_right,     // 右右
    remote_control_ble_stop             // 停止
} RemoteControlStatus;

extern volatile  RemoteControlStatus remote_control_status;

////////////////////////////////////////////////////////////////////////////////
#endif

