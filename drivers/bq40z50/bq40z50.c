
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "bq40z50.h"
#include "i2c.h"
#include "sys.h"
#include <stdio.h>
#include <string.h>

#define SDAH(i2c)  HAL_GPIO_WritePin(i2c->wire.sda.port, i2c->wire.sda.port_num, GPIO_PIN_SET)
#define SDAL(i2c)  HAL_GPIO_WritePin(i2c->wire.sda.port, i2c->wire.sda.port_num, GPIO_PIN_RESET)

#define SDA(i2c)       HAL_GPIO_ReadPin(i2c->wire.sda.port, i2c->wire.sda.port_num)
#define SDA_IN(i2c)    i2c_pin_dir_set(i2c->wire.sda.port, i2c->wire.sda.port_num, I2C_PIN_DIR_IN)
#define SDA_OUT(i2c)   i2c_pin_dir_set(i2c->wire.sda.port, i2c->wire.sda.port_num, I2C_PIN_DIR_OUT)

#define SCLH(i2c)  HAL_GPIO_WritePin(i2c->wire.scl.port, i2c->wire.scl.port_num, GPIO_PIN_SET)
#define SCLL(i2c)  HAL_GPIO_WritePin(i2c->wire.scl.port, i2c->wire.scl.port_num, GPIO_PIN_RESET)

#define SCL(i2c)      HAL_GPIO_ReadPin(i2c->wire.scl.port, i2c->wire.scl.port_num)
#define SCL_IN(i2c)   i2c_pin_dir_set(i2c->wire.scl.port, i2c->wire.scl.port_num, I2C_PIN_DIR_IN)
#define SCL_OUT(i2c)  i2c_pin_dir_set(i2c->wire.scl.port, i2c->wire.scl.port_num, I2C_PIN_DIR_OUT)

#define STM32_DELAY_US_MULT         (12)
#define BATT_SMBUS_PEC_POLYNOMIAL   (0x7)

I2C_INSTANCE_STRU bq40z50_i2c;

void smb_delay_us(unsigned int us);
void smb_Start(I2C_INSTANCE_STRU *i2c);
void smb_Stop(I2C_INSTANCE_STRU *i2c);
void smb_Ack(I2C_INSTANCE_STRU *i2c);
void smb_NoAck(I2C_INSTANCE_STRU *i2c);
void smb_SendByte(I2C_INSTANCE_STRU *i2c, uint8_t SendByte);
uint8_t smb_ReceiveByte(I2C_INSTANCE_STRU *i2c);
void smb_WaitAck(I2C_INSTANCE_STRU *i2c);
uint8_t smb_getPec(uint8_t addr, uint8_t cmd, uint8_t reading, uint8_t *buf, uint8_t len);

void smb_delay_us(unsigned int us)
{
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r"(us)
                 : "r0");
}

void smb_Start(I2C_INSTANCE_STRU *i2c)
{
    SDA_OUT(i2c);
    SCL_OUT(i2c);
    SDAH(i2c);
    SCLH(i2c);
    smb_delay_us(15);
    SDAL(i2c);
    smb_delay_us(15);
    SCLL(i2c);
    smb_delay_us(15);
}

void smb_Stop(I2C_INSTANCE_STRU *i2c)
{
    SDA_OUT(i2c);
    SCLL(i2c);
    SDAL(i2c);
    smb_delay_us(15);
    SCLH(i2c);
    smb_delay_us(15);
    SDAH(i2c);
    smb_delay_us(15);
    osDelay(2);
}

void smb_Ack(I2C_INSTANCE_STRU *i2c)
{
    smb_delay_us(50);
    SCLL(i2c);
    SDA_OUT(i2c);
    SDAL(i2c);

    smb_delay_us(8);
    SCLH(i2c);
    smb_delay_us(8);
    SCLL(i2c);
    smb_delay_us(8);
    SDAH(i2c);
    smb_delay_us(50);
}

void smb_NoAck(I2C_INSTANCE_STRU *i2c)
{
    smb_delay_us(50);
    SCLL(i2c);
    SDA_OUT(i2c);
    SDAH(i2c);

    smb_delay_us(8);
    SCLH(i2c);
    smb_delay_us(8);
    SCLL(i2c);
    smb_delay_us(8);
    SDAL(i2c);
    smb_delay_us(50);
}

void smb_SendByte(I2C_INSTANCE_STRU *i2c, uint8_t SendByte)
{
    uint8_t i;
    uint8_t txd = SendByte;

    SDA_OUT(i2c);
    SCL_OUT(i2c);
    SCLL(i2c);

    for (i = 0; i < 8; i++)
    {
        if (txd & 0x80)
        {
            SDAH(i2c);
        }
        else
        {
            SDAL(i2c);
        }

        txd <<= 1;
        smb_delay_us(8);
        SCLH(i2c);
        smb_delay_us(8);
        SCLL(i2c);
        //smb_delay_us(8);
    }

    SCLL(i2c);
    smb_delay_us(8);
}

uint8_t smb_ReceiveByte(I2C_INSTANCE_STRU *i2c)
{
    uint8_t i;
    uint8_t ReceiveByte = 0;

    SDA_IN(i2c);
    SCL_OUT(i2c);

    for (i = 0; i < 8; i++)
    {
        SCLH(i2c);
        smb_delay_us(8);

        if (SDA(i2c))
        {
            ReceiveByte++;
        }

        if (i < 7)
        {
            ReceiveByte <<= 1;
        }

        SCLL(i2c);
        smb_delay_us(8);
    }
    return ReceiveByte;
}

void smb_WaitAck(I2C_INSTANCE_STRU *i2c)
{
    uint16_t cnt = 0;

    smb_delay_us(8);
    SDAH(i2c);
    SDA_IN(i2c);
    //SCL_IN(i2c);
    smb_delay_us(100);

    SCL_OUT(i2c);
    SCLH(i2c);
    smb_delay_us(8);
    SCLL(i2c);

    while (SDA(i2c) == 1)
    {
        cnt++;
        if (cnt > 3000)
        {
            i2c->error_num = 1;
            i2c->error_cnt++;
            I2C_Stop(i2c);
            return;
        }
    }

    //smb_delay_us(4);
    //SCL_OUT(i2c);
    //SCLL(i2c);
    smb_delay_us(100);

    return;
}


void bq40z50_init(void)
{
    int32_t ret;

    I2C_WIRE_STRU i2c_wire;
    i2c_wire.scl.port = GPIOC;
    i2c_wire.scl.port_num = GPIO_PIN_12;
    i2c_wire.sda.port = GPIOC;
    i2c_wire.sda.port_num = GPIO_PIN_11;

    ret = i2c_init(&bq40z50_i2c, &i2c_wire);
    if (I2C_RET_OK != ret)
    {
        printf("[%s, L%d] i2c_init failed, ret %d.\r\n",
               __FILE__, __LINE__, (int)ret);
    }

    return;
}

int16_t bq40z50_word_read(uint8_t addr, uint8_t cmd, uint16_t *val)
{
    uint8_t buf[2];
    uint8_t  pec;

    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, addr);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, cmd);
    smb_WaitAck(&bq40z50_i2c);

    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, (addr) | 0x01);
    smb_WaitAck(&bq40z50_i2c);

    buf[0] = smb_ReceiveByte(&bq40z50_i2c);
    smb_Ack(&bq40z50_i2c);

    buf[1] = smb_ReceiveByte(&bq40z50_i2c);
    smb_Ack(&bq40z50_i2c);

    pec = smb_ReceiveByte(&bq40z50_i2c);
    smb_NoAck(&bq40z50_i2c);

    smb_Stop(&bq40z50_i2c);

    if (pec == smb_getPec(addr, cmd, 1, buf, 2))
    {
        *val = (buf[1] << 8) | buf[0];
        return 0;
    }
    else
    {
        *val = 0xffff;
        return -1;
    }
}

uint8_t bq40z50_block_read(uint8_t addr, uint8_t cmd, uint8_t *buf)
{
    uint8_t len = 0;
    uint8_t i;
    uint8_t *ptr = buf;

    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, addr);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, cmd);
    smb_WaitAck(&bq40z50_i2c);

    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, (addr) | 0x01);
    smb_WaitAck(&bq40z50_i2c);

    len = smb_ReceiveByte(&bq40z50_i2c);
    smb_Ack(&bq40z50_i2c);
    for (i = 0; i < len; i++)
    {
        if (i < len - 1)
        {
            *ptr = smb_ReceiveByte(&bq40z50_i2c);
            smb_Ack(&bq40z50_i2c);
        }
        else
        {
            *ptr = smb_ReceiveByte(&bq40z50_i2c);
            smb_NoAck(&bq40z50_i2c);
        }
        ptr++;
    }

    smb_Stop(&bq40z50_i2c);
    return len;
}

void bq40z50_word_write(uint8_t addr, uint8_t cmd, uint8_t *buf)
{
    uint8_t *ptr = buf;

    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, addr);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, cmd);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, *ptr);
    smb_WaitAck(&bq40z50_i2c);
    ptr++;

    smb_SendByte(&bq40z50_i2c, *ptr);
    smb_WaitAck(&bq40z50_i2c);

    smb_Stop(&bq40z50_i2c);
}

void bq40z50_block_write(uint8_t addr, uint8_t cmd, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t *ptr = buf;

    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, addr);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, cmd);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, len);
    smb_WaitAck(&bq40z50_i2c);

    for (i = 0; i < len; i++)
    {
        smb_SendByte(&bq40z50_i2c, *ptr);
        smb_WaitAck(&bq40z50_i2c);
        ptr++;
    }

    smb_Stop(&bq40z50_i2c);
    return;
}

void bq40z50_command_send(uint8_t addr, uint8_t cmd)
{
    smb_Start(&bq40z50_i2c);
    smb_SendByte(&bq40z50_i2c, addr);
    smb_WaitAck(&bq40z50_i2c);

    smb_SendByte(&bq40z50_i2c, cmd);
    smb_WaitAck(&bq40z50_i2c);

    smb_Stop(&bq40z50_i2c);

    return;
}

void smb_errorShow(void)
{
    printf("smb err cnt:%u\r\n", (unsigned int)bq40z50_i2c.error_cnt);
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
