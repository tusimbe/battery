#ifndef  __COMM_H__
#define  __COMM_H__

#define COMM_STATE_READY       (0)
#define COMM_STATE_MESSAGE     (1)


void comm_spi_init(void);
void comm_spi_enable(void);

#endif
