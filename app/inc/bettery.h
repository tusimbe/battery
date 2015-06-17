#ifndef __BETTERY_H__
#define __BETTERY_H__

#define  BETT_REG_LEN_WORD            (2)
typedef struct 
{
    char *name;
    uint8_t cmd;
    uint8_t len;
    uint16_t value;
} BETT_REG_ENTRY;

void batteryInit(void);
void batteryInfoShow(void);
int16_t batteryInfoGet(uint16_t addr, uint8_t *buffer, uint16_t buf_len);

#endif
