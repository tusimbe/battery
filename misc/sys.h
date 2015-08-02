#ifndef __SYS_H__
#define __SYS_H__

/*
#define SYS_INTERRUPTS_DISABLE(flag) \
    do                               \
    {                                \
        flag = __get_PRIMASK();      \
        __disable_irq();             \
    } while (0)

#define SYS_INTERRUPTS_ENABLE(flag) \
    do                              \
    {                               \
        if (flag)                   \
        {                           \
            __enable_irq();         \
        }                           \
    } while (0)
*/

#define SYS_INTERRUPTS_DISABLE()  __disable_irq()
#define SYS_INTERRUPTS_ENABLE()   __enable_irq()

#endif
