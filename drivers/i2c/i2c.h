#ifndef __I2C_H__
#define __I2C_H__

#define  I2C_RET_OK             (0)
#define  I2C_RET_PARAM          (-1)
#define  I2c_RET_NO_RESROUCE    (-2)

#define  I2C_PIN_DIR_IN         (0)
#define  I2C_PIN_DIR_OUT        (1)


int32_t i2c_init(uint8_t i2c_dev_num);
int32_t i2c_transfer(uint8_t addr, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

#endif
