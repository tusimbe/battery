
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "errno.h"
#include "i2c.h"
#include "stm32_i2c.h"
#include <stdio.h>

#define I2C_RETRY_TIMES     (3)
struct i2c_dev_s	*dev;

int32_t i2c_init(uint8_t i2c_dev_num)
{
    dev = up_i2cinitialize(i2c_dev_num);
    if (NULL == dev)
    {
        printf("up_i2cinitialize failed!\r\n");
        return -EIO;
    }
    else
    {
        printf("up_i2cinitialize success.\r\n");
    }

    return I2C_RET_OK;
}

int32_t i2c_transfer(uint8_t addr, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	struct i2c_msg_s msgv[2];
	unsigned msgs;
	int ret;
	unsigned retry_count = 0;

	do {
		//	debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		msgs = 0;

		if (send_len > 0) {
			msgv[msgs].addr = addr;
			msgv[msgs].flags = 0;
			msgv[msgs].buffer = (uint8_t *)(send);
			msgv[msgs].length = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].addr = addr;
			msgv[msgs].flags = I2C_M_READ;
			msgv[msgs].buffer = recv;
			msgv[msgs].length = recv_len;
			msgs++;
		}

		if (msgs == 0)
			return -EINVAL;

		/*
		 * I2C architecture means there is an unavoidable race here
		 * if there are any devices on the bus with a different frequency
		 * preference.  Really, this is pointless.
		 */
		I2C_SETFREQUENCY(dev, 50000);
		ret = I2C_TRANSFER(dev, &msgv[0], msgs);

		/* success */
		if (ret == OK)
			break;

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if (retry_count >= I2C_RETRY_TIMES)
        {
            osDelay(1000);
			up_i2creset(dev);
        }

	} while (retry_count++ < I2C_RETRY_TIMES);

	return ret;

}
