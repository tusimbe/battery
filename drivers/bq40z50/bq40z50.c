
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "errno.h"
#include "bq40z50.h"
#include "i2c.h"
#include "sys.h"
#include <stdio.h>
#include <string.h>

#define BATT_SMBUS_PEC_POLYNOMIAL   (0x7)
#define GUGAS_BLOCK_BUFF_MAX        (42)
#define BATT_SMBUS_NUM              (2)

uint8_t smb_getPec(uint8_t addr, uint8_t cmd, uint8_t reading, uint8_t *buf, uint8_t len);

void bq40z50_init(void)
{
    int32_t ret;

    ret = i2c_init(BATT_SMBUS_NUM);
    if (I2C_RET_OK != ret)
    {
        printf("[%s, L%d] i2c_init failed, ret %d.\r\n",
               __FILE__, __LINE__, (int)ret);
    }

    return;
}

int16_t bq40z50_word_read(uint8_t addr, uint8_t cmd, int16_t *val)
{
    uint8_t buff[3];    // 2 bytes of data + PEC

    // read from register
    int ret = i2c_transfer(addr, &cmd, 1, buff, 3);
    if (ret == OK)
    {
        // check PEC
        uint8_t pec = smb_getPec(addr, cmd, 1, buff, 2);
        if (pec == buff[2])
        {
            *val = (uint16_t)buff[1] << 8 | (uint16_t)buff[0];
        }
        else
        {
            printf("pec verify failed (%x, %x)\r\n", pec, buff[2]);
            ret = ENOTTY;
        }
    }
    else
    {
        printf("i2c_transfer failed return %d\r\n", ret);
    }

    // return success or failure
    return ret;
}

// read_block - returns number of characters read if successful, zero if unsuccessful
int16_t bq40z50_block_read(uint8_t addr, uint8_t cmd, uint8_t *buf, uint8_t buf_len, uint8_t append_zero)
{
    uint8_t buff[GUGAS_BLOCK_BUFF_MAX];    // buffer to hold results
    uint8_t pec;

    if (buf_len + 2 > GUGAS_BLOCK_BUFF_MAX)
    {
        return -ENOMEM;
    }

    // read bytes including PEC
    int ret = i2c_transfer(addr, &cmd, 1, buff, buf_len + 2);

    // return zero on failure
    if (ret != OK)
    {
        return ret;
    }

    // get length
    uint8_t bufflen = buff[0];

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > buf_len)
    {
        return -ENOMEM;
    }

    // check PEC
    pec = smb_getPec(addr, cmd, 1, buff, bufflen + 1);
    if (pec != buff[bufflen + 1])
    {
        // debug
        printf("CurrPEC:%x vs MyPec:%x", (int)buff[bufflen + 1], (int)pec);
        return 0;
    }
    else
    {

    }

    // copy data
    memcpy(buf, &buff[1], bufflen);

    // optionally add zero to end
    if (append_zero)
    {
        buf[bufflen] = '\0';
    }

    // return success
    return bufflen;
}

void bq40z50_word_write(uint8_t addr, uint8_t cmd, uint8_t *buf)
{
    addr = addr;
    cmd = cmd;
    buf = buf;
    return;
}

void bq40z50_block_write(uint8_t addr, uint8_t cmd, uint8_t *buf, uint8_t len)
{
    addr = addr;
    cmd = cmd;
    buf = buf;
    len = len;
    return;
}

void bq40z50_command_send(uint8_t addr, uint8_t cmd)
{
    addr = addr;
    cmd = cmd;
    return;
}

int16_t bq40z50_manufacAccess_read(uint8_t addr, uint16_t ma_addr, uint8_t *buf, uint8_t buf_len)
{
    uint16_t len;
    uint16_t ma_addr_read;

    bq40z50_block_write(addr, 0x44, (uint8_t *)&ma_addr, 2);

    len = bq40z50_block_read(addr, 0x44, buf, buf_len, 1);
    if (len > 2)
    {
        ma_addr_read = *(uint16_t *)buf;
        if (ma_addr_read == ma_addr)
        {
            memcpy(buf, &buf[2], len - 2);
            return len - 2;
        }
        else
        {
            printf("manufac read cmd 0x%x\r\n", ma_addr_read);
            return -1;
        }
    }
    else
    {
        printf("manufac read fail ret %d\r\n", len);
        return -1;
    }
}

uint8_t smb_getPec(uint8_t addr, uint8_t cmd, uint8_t reading, uint8_t *buf, uint8_t len)
{
    uint8_t tmp_buff[32];
    uint8_t i, j;

    if (len + 3 > 32)
    {
        return 0;
    }

    tmp_buff[0] = addr;
    tmp_buff[1] = cmd;
    tmp_buff[2] = tmp_buff[0] | reading;

    memcpy(&tmp_buff[3], buf, len);

    // initialise crc to zero
    uint8_t crc = 0;
    uint8_t shift_reg = 0;
    uint8_t do_invert;

    // for each byte in the stream
    for (i = 0; i < len + 3; i++)
    {
        // load next data byte into the shift register
        shift_reg = tmp_buff[i];
        // for each bit in the current byte
        for (j = 0; j < 8; j++)
        {
            do_invert = (crc ^ shift_reg) & 0x80;
            crc <<= 1;
            shift_reg <<= 1;
            if (do_invert)
            {
                crc ^= BATT_SMBUS_PEC_POLYNOMIAL;
            }
        }
    }
    return crc;
}

