#include "cpu.h"
#include "config.h"
#include "typedef.h"
#include "errno-base.h"
#include "app_config.h"
#include "uart_dev.h"

#define LOG_TAG_CONST       NORM
#define LOG_TAG             "[normal]"
#include "log.h"

#include "mcu_app.h"


void app(void)
{
    log_info(" mcu app run\n");

    /* test_openfile_by_file(); */
    //test1();

    while (1) {
        wdt_clear();
    }
}



