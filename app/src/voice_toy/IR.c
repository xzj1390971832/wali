#include "typedef.h"
#include "user_config.h"
#include "IR.h"

#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"


void  ir_init(void)
{
	//红外发射初始化
	IR1_TX_OFF;
	IR1_TX_CONFIG;
	IR1_TX_ON;

	IR2_TX_OFF;
	IR2_TX_CONFIG;
	IR2_TX_ON;

}