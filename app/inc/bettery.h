#ifndef __BETTERY_H__
#define __BETTERY_H__

#define  BETT_REG_LEN_WORD            (2)

#define  BETT_REG_TYPE_UWORD          (0)
#define  BETT_REG_TYPE_SWORD          (1)
#define  BETT_REG_TYPE_BLOCK          (2)

typedef struct 
{
    char *name;
    uint8_t cmd;
    uint8_t len;
    int16_t value;
    uint16_t type;
    int32_t min;
    int32_t max;
} BETT_REG_ENTRY;

void batteryInit(void);
void batteryInfoShow(void);
int16_t batteryInfoGet(uint16_t addr, uint8_t *buffer, uint16_t buf_len);

#endif
