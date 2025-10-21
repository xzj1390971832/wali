//=========================================================================//
//修改日志  2022-08-03  V1.0    1.二次封装了相关的端口操作
//=========================================================================//
#include "user_config.h"

//
#include "dac_api.h"


#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"





//==========================================================//
//====================== 端口配置 ==========================//
//==========================================================//
//


//输入
void port_mode_ipu(JL_PORT_TypeDef* port, u16 pin) {
    port->DIR |= pin;
    port->DIE |= pin;
    port->DIEH |= pin;
    port->PU |= pin;
    port->PD &= ~pin;
}


void port_mode_ipd(JL_PORT_TypeDef* port, u16 pin) {
    port->DIR |= pin;
    port->DIE |= pin;
    port->DIEH |= pin;
    port->PU &= ~pin;
    port->PD |= pin;
}

void port_mode_floating(JL_PORT_TypeDef* port, u16 pin) {
    port->DIR |= pin;
    port->DIE |= pin;
    port->DIEH |= pin;
    port->PU &= ~pin;
    port->PD &= ~pin;
}
void port_mode_ain(JL_PORT_TypeDef* port, u16 pin) {
    port->DIR |= pin;
    port->DIE &= ~pin;
    port->DIEH &= ~pin;
    port->PU &= ~pin;
    port->PD &= ~pin;
}
    u16 port_read(JL_PORT_TypeDef* port, u16 pin) {
    return port->IN & pin;
}


//输出
void port_mode_out(JL_PORT_TypeDef* port, u16 pin) {
    port->DIR &= ~pin;
    port->DIE |= pin;
    port->DIEH |= pin;
    port->PU &= ~pin;
    port->PD &= ~pin;
}
    void port_write(JL_PORT_TypeDef* port, u16 value) {
    port->OUT = value;
}
    void port_set(JL_PORT_TypeDef* port, u16 pin) {
    port->OUT |= pin;
}
    void port_reset(JL_PORT_TypeDef* port, u16 pin) {
    port->OUT &= ~pin;
}
    void port_toggle(JL_PORT_TypeDef* port, u16 pin) {
    port->OUT ^= pin;
}

//大电流输出
    void port_hsink(JL_PORT_TypeDef* port, u16 pin) {
    port->HD0 |= pin;
    port->HD1 |= pin;
}

    void port_hsink_1(JL_PORT_TypeDef* port, u16 pin) {
    port->HD0 |= pin;
    port->HD1 &= ~pin;
}
    void port_hsink_2(JL_PORT_TypeDef* port, u16 pin) {
    port->HD0 &= ~pin;
    port->HD1 |= pin;
}
    void port_hsink_3(JL_PORT_TypeDef* port, u16 pin) {
    port->HD0 |= pin;
    port->HD1 |= pin;
}
//===================================================================//








