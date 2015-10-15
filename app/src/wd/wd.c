/**
 * @file wd.c
 *
 * watchdog implement file.
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "wd.h"
#include "errno.h"

#define WD_REFRESH_CYCLE    (300)

IWDG_HandleTypeDef hiwdg;
osThreadId wd_taskHandler;

void wd_refreshTask(void const *argument);

void wd_init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Reload = 4095;

    HAL_IWDG_Init(&hiwdg);
    return;
}

int32_t wd_start(void)
{
    osThreadDef(wdTaskThread, wd_refreshTask, osPriorityRealtime, 0, 512);
    wd_taskHandler = osThreadCreate(osThread(wdTaskThread), NULL);
    if (NULL == wd_taskHandler)
    {
        printf("[%s, L%d] osThreadCreate failed!\r\n", __FILE__, __LINE__);
        return -EIO;
    }

    return OK;
}


void wd_refreshTask(void const *argument)
{
    argument = argument;

    HAL_IWDG_Start(&hiwdg);

    for (;;)
    {
        osDelay(WD_REFRESH_CYCLE);
        HAL_IWDG_Refresh(&hiwdg);
    }
}
