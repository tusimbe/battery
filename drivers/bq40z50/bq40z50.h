#ifndef __BQ40Z50_H__
#define __BQ40Z50_H__

void bq40z50_init(void);
void bq40z50_command_send(uint8_t addr, uint8_t cmd);
int16_t bq40z50_word_read(uint8_t addr, uint8_t cmd, int16_t *val);
int16_t bq40z50_block_read(uint8_t addr, uint8_t cmd, uint8_t *buf, uint8_t buf_len, uint8_t append_zero);
int16_t bq40z50_word_write(uint8_t addr, uint8_t cmd, uint16_t word);
int16_t bq40z50_block_write(uint8_t addr, uint8_t cmd, uint8_t *buf, uint8_t len);
int16_t bq40z50_manufacAccess_read(uint8_t addr, uint16_t ma_addr, uint8_t *buf, uint8_t buf_len);
void smb_errorShow(void);

#endif
