#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "key.h"
#include "errno.h"
#include "bettery.h"
#include <stdio.h>

#define KEY_UNSTEABLE_TIME      (2)        /* 20ms */
#define KEY_LONG_PRESS_TIME     (200)       /* 2s   */

/* local functions */
int16_t key_release(uint32_t key, uint32_t timer_cnt);
int16_t key_read(uint32_t key);
int16_t key_waiting(uint32_t key, uint32_t timer_cnt);
void    key_scan_task(void const *argument);

KEY_INFO g_keys[KEY_NUM_IN_SYSTEM];
int32_t battery_status = 0;
osThreadId key_task_handler;

void key_init(KEY_CONF key_conf[], uint32_t num_of_key)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    uint32_t i;

    for (i = 0; i < num_of_key; i++)
    {
        g_keys[i].pin = key_conf[i].pin;
        g_keys[i].GPIO = key_conf[i].GPIO;
        g_keys[i].state = KEY_STATE_NO_KEY;
        g_keys[i].timer_cnt = 0;
        g_keys[i].read_func = key_read;
        g_keys[i].press_cb_func = NULL;
        g_keys[i].waiting_cb_func = key_waiting;
        g_keys[i].release_cb_func = NULL;

        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pin = key_conf[i].pin;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(key_conf[i].GPIO, &GPIO_InitStruct);

        g_keys[i].valid = 1;
    }

    osThreadDef(key_task, key_scan_task, osPriorityNormal, 0, 512);
    key_task_handler = osThreadCreate(osThread(key_task), NULL);
    if (NULL == key_task_handler)
    {
        printf("[%s, L%d] create thread failed!\r\n", __FILE__, __LINE__);
        return;
    }
 
    return;
}

void key_scan_task(void const *argument)
{
    argument = argument;
    for (;;)
    {
        osDelay(10);
        key_scan();
    }
}

int16_t key_read(uint32_t key)
{
    GPIO_PinState st;
    if (key >= KEY_NUM_IN_SYSTEM)
    {
        return -ENXIO;
    }

    if (!g_keys[key].valid)
    {
        return -ENODEV;
    }

    st = HAL_GPIO_ReadPin(g_keys[key].GPIO, g_keys[key].pin);
    if (st == GPIO_PIN_SET)
    {
        return KEY_DOWN;
    }
    else
    {
        return KEY_UP;
    }
}

int16_t key_waiting(uint32_t key, uint32_t timer_cnt)
{
    //printf("key waiting cnt: %d\r\n", (int)timer_cnt);
    if (timer_cnt == KEY_LONG_PRESS_TIME)
    {
        printf("catch key %d long press!\r\n", (int)key);
        printf("Led turn.\r\n");
        batteryLedDisplayEnDis();
        osDelay(1000);
        printf("trun fet.\r\n");
        batteryEnterShutdown();
    }

    return OK;
}

int16_t key_release(uint32_t key, uint32_t timer_cnt)
{
    uint32_t isLongPress = 0;

    if (timer_cnt >= KEY_LONG_PRESS_TIME)
    {
        isLongPress = 1;
    }

    if (isLongPress)
    {
        printf("catch key %d long press!\r\n", (int)key);
        printf("Led turn.\r\n");
        batteryLedDisplayEnDis();
        //osDelay(1000);
        printf("trun fet.\r\n");
        batteryEnterShutdown();
    }
    else
    {
        printf("catch key %d short press!\r\n", (int)key);
    }

    return OK;
}

int16_t key_scan(void)
{
    uint32_t i;

    for (i = 0; i < KEY_NUM_IN_SYSTEM; i++)
    {
        int16_t key_st;

        if (!g_keys[i].valid)
        {
            continue;
        }

        if (g_keys[i].read_func == NULL)
        {
            continue;
        }

        key_st = g_keys[i].read_func(i);
        if (key_st < 0)
        {
            printf("read key %d return errno %d\r\n", (int)i, (int)key_st);
        }

        switch (g_keys[i].state)
        {
        case KEY_STATE_NO_KEY:
            if (key_st == KEY_DOWN)
            {
                g_keys[i].state = KEY_STATE_PRESS;
                g_keys[i].timer_cnt = 0;
            }
            break;
        case KEY_STATE_PRESS:
            g_keys[i].timer_cnt++;
            if (g_keys[i].timer_cnt >= KEY_UNSTEABLE_TIME)
            {
                if (key_st == KEY_DOWN)
                {
                    if (g_keys[i].press_cb_func != NULL)
                    {
                        g_keys[i].press_cb_func(i);
                    }
                    g_keys[i].state = KEY_STATE_WAITING;
                }
                else
                {
                    g_keys[i].state = KEY_STATE_NO_KEY;
                }
            }
            break;
        case KEY_STATE_WAITING:
            g_keys[i].timer_cnt++;
            if (g_keys[i].waiting_cb_func != NULL)
            {
                g_keys[i].waiting_cb_func(i, g_keys[i].timer_cnt);
            }

            if (key_st == KEY_UP)
            {
                g_keys[i].state = KEY_STATE_RELEASE;
            }
            break;
        case KEY_STATE_RELEASE:
            if (g_keys[i].release_cb_func != NULL)
            {
                g_keys[i].release_cb_func(i, g_keys[i].timer_cnt);
            }
            g_keys[i].state = KEY_STATE_NO_KEY;
            break;
        default:
            return -ENXIO;
            break;
        }
    }

    return OK;
}

