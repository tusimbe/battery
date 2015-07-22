/**
 * @file    chqueues.c
 * @brief   I/O Queues code.
 *
 * @addtogroup io_queues
 * @details ChibiOS/RT queues are mostly used in serial-like device drivers.
 *          The device drivers are usually designed to have a lower side
 *          (lower driver, it is usually an interrupt service routine) and an
 *          upper side (upper driver, accessed by the application threads).<br>
 *          There are several kind of queues:<br>
 *          - <b>Input queue</b>, unidirectional queue where the writer is the
 *            lower side and the reader is the upper side.
 *          - <b>Output queue</b>, unidirectional queue where the writer is the
 *            upper side and the reader is the lower side.
 *          - <b>Full duplex queue</b>, bidirectional queue. Full duplex queues
 *            are implemented by pairing an input queue and an output queue
 *            together.
 *          .
 * @pre     In order to use the I/O queues the @p CH_USE_QUEUES option must
 *          be enabled in @p chconf.h.
 * @{
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "simpleQueue.h"
#include "sys.h"
#include <stdio.h>

/**
 * @brief   Initializes an input queue.
 * @details A Semaphore is internally initialized and works as a counter of
 *          the bytes contained in the queue.
 * @note    The callback is invoked from within the S-Locked system state,
 *          see @ref system_states.
 *
 * @param[out] iqp      pointer to an @p InputQueue structure
 * @param[in] bp        pointer to a memory area allocated as queue buffer
 * @param[in] size      size of the queue buffer
 * @param[in] infy      pointer to a callback function that is invoked when
 *                      data is read from the queue. The value can be @p NULL.
 * @param[in] link      application defined pointer
 *
 * @init
 */
void chIQInit(InputQueue *iqp, uint8_t *bp, size_t size, qnotify_t infy) 
{
    iqp->q_counter = 0;
    iqp->q_buffer = iqp->q_rdptr = iqp->q_wrptr = bp;
    iqp->q_top = bp + size;
    iqp->q_notify = infy;
}

/**
 * @brief   Resets an input queue.
 * @details All the data in the input queue is erased and lost, any waiting
 *          thread is resumed with status @p Q_RESET.
 * @note    A reset operation can be used by a low level driver in order to
 *          obtain immediate attention from the high level layers.
 *
 * @param[in] iqp       pointer to an @p InputQueue structure
 *
 * @iclass
 */
void chIQResetI(InputQueue *iqp) 
{
    uint32_t flag;

    SYS_INTERRUPTS_DISABLE(flag);
    iqp->q_rdptr = iqp->q_wrptr = iqp->q_buffer;
    iqp->q_counter = 0;
    SYS_INTERRUPTS_ENABLE(flag);
}

/**
 * @brief   Input queue write.
 * @details A byte value is written into the low end of an input queue.
 *
 * @param[in] iqp       pointer to an @p InputQueue structure
 * @param[in] b         the byte value to be written in the queue
 * @return              The operation status.
 * @retval Q_OK         if the operation has been completed with success.
 * @retval Q_FULL       if the queue is full and the operation cannot be
 *                      completed.
 *
 * @iclass
 */
int32_t chIQPutI(InputQueue *iqp, uint8_t b) 
{
    qnotify_t cb = iqp->q_notify;

    if (chIQIsFullI(iqp))
    {
        return Q_FULL;
    }

    iqp->q_counter++;
    *iqp->q_wrptr++ = b;
    if (iqp->q_wrptr >= iqp->q_top)
    {
        iqp->q_wrptr = iqp->q_buffer;
    }

    if (cb)
    {
        cb(iqp);
    }

    return Q_OK;
}

/**
 * @brief   Input queue read with timeout.
 * @details This function reads a byte value from an input queue. If the queue
 *          is empty then the calling thread is suspended until a byte arrives
 *          in the queue or a timeout occurs.
 * @note    The callback is invoked before reading the character from the
 *          buffer or before entering the state @p THD_STATE_WTQUEUE.
 *
 * @param[in] iqp       pointer to an @p InputQueue structure
 * @param[in] time      the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              A byte value from the queue.
 * @retval Q_TIMEOUT    if the specified time expired.
 * @retval Q_RESET      if the queue has been reset.
 *
 * @api
 */
int32_t chIQGet(InputQueue *iqp) 
{
    uint8_t b;
    uint32_t flag;

    SYS_INTERRUPTS_DISABLE(flag);

    if (chIQIsEmptyI(iqp)) 
    {
        SYS_INTERRUPTS_ENABLE(flag);
        return Q_EMPTY;
    }

    iqp->q_counter--;
    b = *iqp->q_rdptr++;
    if (iqp->q_rdptr >= iqp->q_top)
    {
        iqp->q_rdptr = iqp->q_buffer;
    }

    SYS_INTERRUPTS_ENABLE(flag);
    return b;
}

/**
 * @brief   Input queue read with timeout.
 * @details The function reads data from an input queue into a buffer. The
 *          operation completes when the specified amount of data has been
 *          transferred or after the specified timeout or if the queue has
 *          been reset.
 * @note    The function is not atomic, if you need atomicity it is suggested
 *          to use a semaphore or a mutex for mutual exclusion.
 * @note    The callback is invoked before reading each character from the
 *          buffer or before entering the state @p THD_STATE_WTQUEUE.
 *
 * @param[in] iqp       pointer to an @p InputQueue structure
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred, the
 *                      value 0 is reserved
 * @param[in] time      the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The number of bytes effectively transferred.
 *
 * @api
 */
size_t chIQRead(InputQueue *iqp, uint8_t *bp, size_t n) 
{
    size_t r = 0;
    uint32_t flag;

    SYS_INTERRUPTS_DISABLE(flag);

    if (chQSpaceI(iqp) < n) 
    {
        SYS_INTERRUPTS_ENABLE(flag);
        return Q_EMPTY;
    }

    while (1)
    {
        iqp->q_counter--;
        *bp++ = *iqp->q_rdptr++;
        if (iqp->q_rdptr >= iqp->q_top)
        {
            iqp->q_rdptr = iqp->q_buffer;
        }

        SYS_INTERRUPTS_ENABLE(flag); /* Gives a preemption chance in a controlled point.*/
        r++;
        if (--n == 0)
        {
            return r;
        }

        SYS_INTERRUPTS_DISABLE(flag);
    }
}

/** @} */
