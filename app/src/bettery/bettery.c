/**
 * @file bettery.c
 *
 * bettery mananger module.
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "bq40z50.h"
#include "bettery.h"
#include <stdio.h>

#define BATTERY_TASK_RUNNING_CYCLE    (100)
#define BATTERY_SMBUS_ADDR            (0x16)

BETT_REG_ENTRY bettery_regs[] = 
{
    {"Temperature", 0x08, BETT_REG_LEN_WORD, 0},
    {"Voltage", 0x09, BETT_REG_LEN_WORD, 0},
    {"Current", 0x0a, BETT_REG_LEN_WORD, 0},
    {"AverageCurrent", 0x0b, BETT_REG_LEN_WORD, 0},
    {"RelativeStateOfChange", 0x0d, BETT_REG_LEN_WORD, 0},
    {"RemainingCapacity", 0x0f, BETT_REG_LEN_WORD, 0},
    {"BatteryStatus", 0x16, BETT_REG_LEN_WORD, 0},
    {"Cell 1", 0x3f, BETT_REG_LEN_WORD, 0},
    {"Cell 2", 0x3e, BETT_REG_LEN_WORD, 0},
    {"Cell 3", 0x3d, BETT_REG_LEN_WORD, 0},
    {"Cell 4", 0x3c, BETT_REG_LEN_WORD, 0}
};

uint32_t battery_regs_size = sizeof(bettery_regs) / sizeof(bettery_regs[0]);

osThreadId batteryTaskHandle;

void batteryTask(void const * argument);
void batteryInfoUpdate(void);


void batteryInit(void)
{
    bq40z50_init();
    
    osThreadDef(batteryTaskThread, batteryTask, osPriorityHigh, 0, 1024);
    batteryTaskHandle = osThreadCreate(osThread(batteryTaskThread), NULL);
    if (NULL == batteryTaskHandle)
    {
        printf("[%s, L%d] osThreadCreate failed!\r\n", __FILE__, __LINE__);
    }

    return;
}

void batteryTask(void const * argument)
{    
    argument = argument;
    
    for(;;)
    {
        osDelay(BATTERY_TASK_RUNNING_CYCLE);
        batteryInfoUpdate();
    }
}

void batteryInfoUpdate(void)
{
    uint16_t i;
    
    for (i = 0; i < battery_regs_size; i++)
    {
        switch (bettery_regs[i].len)
        {
            case BETT_REG_LEN_WORD:
                bettery_regs[i].value = 
                    bq40z50_word_read(BATTERY_SMBUS_ADDR, bettery_regs[i].cmd);
                break;
            default:
                break;
        }
    }    
}

int16_t batteryInfoGet(uint16_t addr, uint8_t *buffer, uint16_t buf_len)
{
    if (addr >= battery_regs_size)
    {
        return -1;
    }
    
    memcpy(buffer, &bettery_regs[addr].value, sizeof(uint16_t));

    return sizeof(uint16_t);
}

void batteryInfoShow(void)
{
    uint16_t i;

    printf("battery info:\r\n");
    
    for (i = 0; i < battery_regs_size; i++)
    {
        printf("%s : %d\r\n", bettery_regs[i].name, bettery_regs[i].value);
    }

    return;
}

