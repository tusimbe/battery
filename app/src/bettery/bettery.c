/**
 * @file bettery.c
 *
 * bettery mananger module.
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "bq40z50.h"
#include "bettery.h"
#include "errno.h"
#include <stdio.h>
#include <string.h>

#define BATTERY_TASK_RUNNING_CYCLE    (1007)
#define BATTERY_SMBUS_ADDR            (0x16)

#define BATTERY_BLOCK_BUF_LEN         (32)

BETT_REG_ENTRY bettery_regs[] =
{
    //{ "Temperature", 0x72, 14, 0, BETT_REG_TYPE_BLOCK, 65536, 0 },
    { "Voltage", 0x09, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 65536, 0 },
    { "Current", 0x0a, BETT_REG_LEN_WORD, 0, 1, 32768, 0 },
    { "AverageCurrent", 0x0b, BETT_REG_LEN_WORD, 0, 1, 32768, 0 },
    { "RelativeStateOfChange", 0x0d, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 100, 0 },
    { "RemainingCapacity", 0x0f, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 65536, 0 },
    { "Cell 1", 0x3f, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 65535, 0 },
    { "Cell 2", 0x3e, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 65535, 0 },
    { "Cell 3", 0x3d, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 65536, 0 },
    { "Cell 4", 0x3c, BETT_REG_LEN_WORD, 0, BETT_REG_TYPE_UWORD, 65536, 0 },
    //{ "Device name", 0x21, 7, 0, BETT_REG_TYPE_BLOCK, 65536, 0 }
};

uint32_t battery_regs_size = sizeof(bettery_regs) / sizeof(bettery_regs[0]);

osThreadId batteryTaskHandle;

void batteryTask(void const *argument);
void batteryInfoUpdate(void);


void batteryInit(void)
{
    bq40z50_init();

    osThreadDef(batteryTaskThread, batteryTask, osPriorityAboveNormal, 0, 1024);
    batteryTaskHandle = osThreadCreate(osThread(batteryTaskThread), NULL);
    if (NULL == batteryTaskHandle)
    {
        printf("[%s, L%d] osThreadCreate failed!\r\n", __FILE__, __LINE__);
    }

    return;
}

void batteryTask(void const *argument)
{
    argument = argument;

    for (;;)
    {
        osDelay(BATTERY_TASK_RUNNING_CYCLE);
        batteryInfoUpdate();
    }
}

void batteryInfoUpdate(void)
{
    uint16_t i;
    int16_t ret;
    int16_t val;
    int32_t tmp;
    uint8_t buf[BATTERY_BLOCK_BUF_LEN];

    for (i = 0; i < battery_regs_size; i++)
    {
        switch (bettery_regs[i].type)
        {
        case BETT_REG_TYPE_UWORD:
        case BETT_REG_TYPE_SWORD:
            ret = bq40z50_word_read(BATTERY_SMBUS_ADDR, bettery_regs[i].cmd, &val);
            if (ret == 0)
            {
                bettery_regs[i].value = val;

                if (bettery_regs[i].type == 0)
                {
                    tmp = (uint16_t)val;
                }
                else
                {
                    tmp = val;
                }

                if (tmp > bettery_regs[i].max)
                {
                    bettery_regs[i].max = tmp;
                }

                if (tmp < bettery_regs[i].min)
                {
                    bettery_regs[i].min = tmp;
                }
            }
            break;
        case BETT_REG_TYPE_BLOCK:
            ret = bq40z50_block_read(BATTERY_SMBUS_ADDR, bettery_regs[i].cmd, buf, BATTERY_BLOCK_BUF_LEN, 1);
            if (ret > 0)
            {
                if (ret == bettery_regs[i].len)
                {
                    val = *(uint16_t*)&buf[12];
                    bettery_regs[i].value = val;

                    if (bettery_regs[i].type == 0)
                    {
                        tmp = (uint16_t)val;
                    }
                    else
                    {
                        tmp = val;
                    }

                    if (tmp > bettery_regs[i].max)
                    {
                        bettery_regs[i].max = tmp;
                    }

                    if (tmp < bettery_regs[i].min)
                    {
                        bettery_regs[i].min = tmp;
                    }
                }
                else
                {
                    printf("read block len error, ret len %d\r\n", ret);
                }
            }
            else
            {
                printf("read block failed ret %d\r\n", ret);
            }
            break;
        default:
            break;
        }
    }
}

int16_t batteryInfoGet(uint16_t addr, uint8_t *buffer, uint16_t buf_len)
{
    buf_len = buf_len;

    if (addr >= battery_regs_size)
    {
        return -1;
    }

    memcpy(buffer, &bettery_regs[addr].value, sizeof(uint16_t));

    return sizeof(uint16_t);
}

int16_t batteryEnterShutdown(void)
{
    uint16_t word;
    int16_t  ret;

    word = 0x2706;
    ret = bq40z50_word_write(BATTERY_SMBUS_ADDR, 0x00, word);
    if (OK != ret)
    {
        printf("[%s, L%d] bq40z50_word_write ret %d\r\n", __FILE__, __LINE__, ret);
        return ret;
    }
    else
    {
        word = 0x043d;
        ret = bq40z50_word_write(BATTERY_SMBUS_ADDR, 0x00, word);
        if (ret != OK)
        {
            printf("[%s, L%d] bq40z50_word_write ret %d\r\n", __FILE__, __LINE__, ret);
            return ret;
        }
    }

    return OK;
}

int16_t batteryExitShutdown(void)
{
    uint16_t word;
    int16_t  ret;

    word = 0x23a7;
    ret = bq40z50_word_write(BATTERY_SMBUS_ADDR, 0x00, word);
    if (OK != ret)
    {
        printf("[%s, L%d] bq40z50_word_write ret %d\r\n", __FILE__, __LINE__, ret);
        return ret;
    }

    return OK;
}

void batteryInfoShow(void)
{
    uint16_t i;

    printf("************** battery info **************\r\n");
    for (i = 0; i < battery_regs_size; i++)
    {
        switch (bettery_regs[i].type)
        {
        case BETT_REG_TYPE_UWORD:
        case BETT_REG_TYPE_SWORD:
        case BETT_REG_TYPE_BLOCK:
            printf("%s : %d(min:%d, max:%d)\r\n", bettery_regs[i].name,
                   (int)bettery_regs[i].value, (int)bettery_regs[i].min, (int)bettery_regs[i].max);
            break;
        default:
            break;
        }
    }

    printf("******************************************\r\n");

    return;
}

