/**
 * @file bettery.c
 *
 * bettery mananger module.
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "comm.h"
#include "bettery.h"
#include "simpleQueue.h"
#include "sys.h"

/* C libraries: */
#include <string.h>
#include <stdio.h>

/* Predefined COMM responses. */
#define COMM_RESP_OK         "_OK_"
#define COMM_RESP_FAIL       "FAIL"

/* COMM start-of-frame signature. */
#define COMM_MSG_SOF         0xBD
/* Empty message ID. */
#define COMM_MSG_NOMSG       0x00
/* COMM buffer size in bytes.  */
#define COMM_BUFFER_SIZE     0x80
/* COMM message header size in bytes.  */
#define COMM_MSG_HDR_SIZE    0x04
/* COMM message checksum size in bytes.  */
#define COMM_MSG_CRC_SIZE    0x04
/* COMM message header + crc size in bytes.  */
#define COMM_MSG_SVC_SIZE    ( COMM_MSG_HDR_SIZE + COMM_MSG_CRC_SIZE )

#define COMM_IQP_BUF_SIZE     512

/* COMM message structure. */
typedef struct tagMessage {
    uint8_t sof;      /* Start of frame signature. */
    uint8_t msg_id;   /* COMM message ID. */
    uint8_t size;     /* Size of whole COMM message including header and additional data. */
    uint8_t res;      /* Reserved. Set to 0. */
    uint8_t data[COMM_BUFFER_SIZE];
    uint32_t crc;
} __attribute__((packed)) Message, *PMessage;

/**
 * Global variables
 */
/* comm input queue buf */
uint8_t g_comm_iqp_buf[COMM_IQP_BUF_SIZE];

InputQueue g_iqp;

UART_HandleTypeDef huart2;

CRC_HandleTypeDef hcrc;

osSemaphoreId commSema;

osThreadId CommTaskHandle;

uint32_t g_dbgUartRxCnt;
/**
 * Local variables
 */
/* COMM message. */
static Message msg = {
    COMM_MSG_SOF,
    COMM_MSG_NOMSG,
    COMM_MSG_SVC_SIZE,
    0,
    { 0 },
    0xFFFFFFFF
};
static uint8_t *msgPos = (uint8_t *)&msg;

static size_t bytesRequired = COMM_MSG_HDR_SIZE;

/* fucntion */
void CommRxCallBack(GenericQueue *iqp);
void CommUartTxIrqProcess(UART_HandleTypeDef *huart);
void CommUartRxIrqProcess(UART_HandleTypeDef *huart);
void CommUartIrqHandler(UART_HandleTypeDef *huart);
void USART2_IRQHandler(void);
void CommStartTask(void const *argument);
void CommPrintMsg(size_t len);

/**
 * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit).
 * @param  pBuf: pointer to the buffer containing the data to be computed
 * @param  length: length of the buffer to be computed
 * @retval 32-bit CRC
 */
static inline uint32_t crcCRC32(uint32_t pBuf[], uint32_t length)
{
    return HAL_CRC_Calculate(&hcrc, pBuf, length);
}

/**
 * @brief  Calculates CRC32 checksum of received data buffer.
 * @param  pMsg - pointer to COMM message structure.
 * @return CRC32 checksum of received zero-padded data buffer.
 */
static uint32_t CommGetCRC32Checksum(const PMessage pMsg)
{
    uint32_t crc_length = (pMsg->size - COMM_MSG_CRC_SIZE) / sizeof(uint32_t);
    if ((pMsg->size - COMM_MSG_CRC_SIZE) % sizeof(uint32_t))
    {
        crc_length++;
    }
    return crcCRC32((uint32_t *)pMsg, crc_length);
}

static void CommWrite(uint8_t *pBuf, uint32_t len)
{
    (void)HAL_UART_Transmit(&huart2, pBuf, len, 5000);
    return;
}

/**
 * @brief  Sends data to selected serial port.
 * @param  pMsg - pointer to COMM message structure to be sent.
 * @return none.
 */
static void CommSendSerialData(const PMessage pMsg)
{
    /* Sends message header and actual data if any. */
    CommWrite((uint8_t *)pMsg, pMsg->size - COMM_MSG_CRC_SIZE);
    /* Sends cyclic redundancy checksum. */
    CommWrite((uint8_t *)&pMsg->crc, COMM_MSG_CRC_SIZE);
}

/**
 * @brief  Prepares positive COMM response.
 * @param  pMsg - pointer to COMM message structure.
 * @return none.
 */
static void CommPositiveResponse(const PMessage pMsg)
{
    memcpy((void *)pMsg->data, (void *)COMM_RESP_OK, sizeof(COMM_RESP_OK) - 1);
    pMsg->size = sizeof(COMM_RESP_OK) + COMM_MSG_SVC_SIZE - 1;
    pMsg->crc  = CommGetCRC32Checksum(pMsg);
}

/**
 * @brief  Prepares negative COMM response.
 * @param  pMsg - pointer to COMM message structure.
 * @return none.
 */
static void CommNegativeResponse(const PMessage pMsg)
{
    memcpy((void *)pMsg->data, (void *)COMM_RESP_FAIL, sizeof(COMM_RESP_FAIL) - 1);
    pMsg->size = sizeof(COMM_RESP_FAIL) + COMM_MSG_SVC_SIZE - 1;
    pMsg->crc  = CommGetCRC32Checksum(pMsg);
}

/**
 * @brief  Command processor.
 * @param  pMsg - pointer to COMM message structure to be processed.
 * @return none.
 */
static void CommProcessCommand(const PMessage pMsg)
{
    uint16_t regValue;

    switch (pMsg->msg_id)
    {
    case 'v': /* Reads new sensor settings; */
        batteryInfoGet(0, (uint8_t *)&regValue, sizeof(uint16_t));
        memset(&pMsg->data, 0, COMM_BUFFER_SIZE);
        memcpy(&pMsg->data, &regValue, sizeof(uint16_t));
        pMsg->size = sizeof(uint16_t) + COMM_MSG_SVC_SIZE;
        pMsg->crc = CommGetCRC32Checksum(pMsg);
        break;
    case 'c':
        batteryInfoGet(2, (uint8_t *)&regValue, sizeof(uint16_t));
        memset(&pMsg->data, 0, COMM_BUFFER_SIZE);
        memcpy(&pMsg->data, &regValue, sizeof(uint16_t));
        pMsg->size = sizeof(uint16_t) + COMM_MSG_SVC_SIZE;
        pMsg->crc = CommGetCRC32Checksum(pMsg);
        break;
    case 'p':
        batteryInfoGet(4, (uint8_t *)&regValue, sizeof(uint16_t));
        memset(&pMsg->data, 0, COMM_BUFFER_SIZE);
        memcpy(&pMsg->data, &regValue, sizeof(uint16_t));
        pMsg->size = sizeof(uint16_t) + COMM_MSG_SVC_SIZE;
        pMsg->crc = CommGetCRC32Checksum(pMsg);
        break;
    case 'r':
        batteryInfoGet(5, (uint8_t *)&regValue, sizeof(uint16_t));
        memset(&pMsg->data, 0, COMM_BUFFER_SIZE);
        memcpy(&pMsg->data, &regValue, sizeof(uint16_t));
        pMsg->size = sizeof(uint16_t) + COMM_MSG_SVC_SIZE;
        pMsg->crc = CommGetCRC32Checksum(pMsg);
        break;         
    case 's':
        CommPositiveResponse(pMsg);
        break;
    default: /* Unknown command. */
        CommNegativeResponse(pMsg);
        break;
    }
    pMsg->sof = COMM_MSG_SOF;
    pMsg->res = 0;
    CommSendSerialData(pMsg);
}

#define IS_MSG_VALID() \
  ((msg.sof == COMM_MSG_SOF) &&\
  (msg.size >= COMM_MSG_SVC_SIZE) &&\
  (msg.size <= COMM_BUFFER_SIZE))

/**
 * @brief: Try to recover from bad input data stream
 * @note: Author of this expects that most of the time, this will be caused
 *        by some junk stuff coming from OS (e.g. tty settings). We'll just
 *        sync to next SOF and throw away any after packet. This means the next
 *        (few) packet(s) may be dropped - still better than no comm...
 */
static void CommReadSerialDataResync(uint8_t len)
{
    uint8_t i;

    while (len)
    {
        for (i = 1; i < len; i++)
        {
            if (((uint8_t *)&msg)[i] == COMM_MSG_SOF)
            {
                break;
            }
        }

        if (len - i > 0)
        {
            memmove(&msg, &((uint8_t *)&msg)[i], len - i);
        }
        len -= i;
        msgPos = (uint8_t *)&msg + len;

        if (len < COMM_MSG_HDR_SIZE)
        {
            /* ...wait for header to get completed */
            bytesRequired = COMM_MSG_HDR_SIZE - len;
            break;
        }
        else
        {
            if (IS_MSG_VALID())
            {
                if (msg.size <= len)
                {
                    /* This may throw away some data at the tail of buffer...*/
                    bytesRequired = 0;
                }
                else
                {
                    bytesRequired = msg.size - len;
                }
                break;
            }
        }
    }
}

/**
 * @brief  Process messages received from generic serial interface driver.
 * @return none.
 */
static void CommReadSerialData(void)
{
    uint32_t crc_packet;

    SYS_INTERRUPTS_DISABLE();
    /* The following function must be called from within a system lock zone. */
    size_t bytesAvailable = chQSpaceI(&g_iqp);
    SYS_INTERRUPTS_ENABLE();

    while (bytesAvailable)
    {
        if (bytesAvailable >= bytesRequired)
        {
            if (bytesRequired > 0)
            {
                if (chIQRead(&g_iqp, msgPos, bytesRequired) < 0)
                {
                    printf("[%s, L%d] read queue failed.\r\n", __FILE__, __LINE__);
                }
                msgPos += bytesRequired;
                bytesAvailable -= bytesRequired;
                bytesRequired = 0;
            }
        }
        else
        {
            if (chIQRead(&g_iqp, msgPos, bytesAvailable) < 0)
            {
                printf("[%s, L%d] read queue failed.\r\n", __FILE__, __LINE__);
            }
            msgPos += bytesAvailable;
            bytesRequired -= bytesAvailable;
            break;
        }

        size_t curReadLen = msgPos - (uint8_t *)&msg;
        if (!IS_MSG_VALID())
        {
            CommReadSerialDataResync(curReadLen);
        }
        else if (curReadLen == COMM_MSG_HDR_SIZE)
        {
            bytesRequired = msg.size - COMM_MSG_HDR_SIZE;
        }
        else if (bytesRequired == 0)
        {
            /* Whole packet is read, check and process it... */
            /* Move CRC */
            memmove(&msg.crc, (uint8_t *)&msg + msg.size - COMM_MSG_CRC_SIZE,
                    COMM_MSG_CRC_SIZE);
            /* Zero-out unused data for crc32 calculation. */
            memset(&msg.data[msg.size - COMM_MSG_SVC_SIZE], 0,
                   COMM_BUFFER_SIZE - msg.size + COMM_MSG_SVC_SIZE);

            if (msg.crc == (crc_packet = CommGetCRC32Checksum(&msg)))
            {
                CommProcessCommand(&msg);
            }
            else
            {
                CommPrintMsg(curReadLen);
                printf("crc error:0x%08x 0x%08x\r\n", (unsigned int)msg.crc, (unsigned int)crc_packet);
            }

            /* Read another packet...*/
            bytesRequired = COMM_MSG_HDR_SIZE;
            msgPos = (uint8_t *)&msg;
        }
    }
}

void CommRxCallBack(GenericQueue *iqp)
{
    iqp = iqp;
    
    (void)osSemaphoreRelease(commSema);
    return;
}

void CommStartTask(void const *argument)
{
    argument = argument;
    for (;;)
    {
        osDelay(10);
        CommReadSerialData();
    }
}

void CommInit(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 57600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    hcrc.Instance = CRC;
    HAL_CRC_Init(&hcrc);

    chIQInit(&g_iqp, g_comm_iqp_buf, COMM_IQP_BUF_SIZE, NULL);

    osSemaphoreDef(COMM_SEMA);
    commSema = osSemaphoreEmptyCreate(osSemaphore(COMM_SEMA));
    if (NULL == commSema)
    {
        printf("[%s, L%d] create semaphore failed!\r\n", __FILE__, __LINE__);
        return;
    }

    osThreadDef(CommTask, CommStartTask, osPriorityNormal, 0, 1024);
    CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);
    if (NULL == CommTaskHandle)
    {
        printf("[%s, L%d] create thread failed!\r\n", __FILE__, __LINE__);
        return;
    }

    HAL_NVIC_SetPriority(USART2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    return;
}

void CommUartIrqHandler(UART_HandleTypeDef *huart)
{
    uint32_t tmp_flag = 0, tmp_it_source = 0;

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);
    /* UART parity error interrupt occurred ------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
    /* UART frame error interrupt occurred -------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        __HAL_UART_CLEAR_FEFLAG(huart);

        huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
    /* UART noise error interrupt occurred -------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        __HAL_UART_CLEAR_NEFLAG(huart);

        huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
    /* UART Over-Run interrupt occurred ----------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        __HAL_UART_CLEAR_OREFLAG(huart);

        huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        CommUartRxIrqProcess(huart);
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
    /* UART in mode Transmitter ------------------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        CommUartTxIrqProcess(huart);
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TC);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
    /* UART in mode Transmitter end --------------------------------------------*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET))
    {

    }

    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        /* Set the UART state ready to be able to start again the process */
        huart->State = HAL_UART_STATE_READY;

        HAL_UART_ErrorCallback(huart);
    }
}

void CommUartTxIrqProcess(UART_HandleTypeDef *huart)
{
    huart = huart;
    return;
}

void CommUartRxIrqProcess(UART_HandleTypeDef *huart)
{
    uint8_t b;

    g_dbgUartRxCnt++;
    b = huart->Instance->DR & 0xff;
    if (Q_OK != chIQPutI(&g_iqp, b))
    {
        printf("enqueue failed.\r\n");
    }
    return;
}

void CommShowCnt(void)
{
    printf("Uart rx bytes:%lu\r\n", g_dbgUartRxCnt);
    return;
}

void USART2_IRQHandler(void)
{
    uint16_t sr = huart2.Instance->SR;

    while (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE |
               USART_SR_PE)) 
    {
        uint8_t b;

        /* Error condition detection.*/
        if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE  | USART_SR_PE))
        {
            printf("uart error 0x%x!\r\n", sr);
        }

        b = huart2.Instance->DR;
        if (sr & USART_SR_RXNE)
        {
            if (Q_OK != chIQPutI(&g_iqp, b))
            {
                printf("enqueue failed.\r\n");
            }
        }
        sr = huart2.Instance->SR;
    }
    //CommUartIrqHandler(&huart2);
}

void CommPrintMsg(size_t len)
{
    printf("msglen:%d, msg:%02x %02x %02x %02x %08x\r\n", 
           len, (unsigned int)msg.sof, (unsigned int)msg.msg_id, 
           (unsigned int)msg.size, (unsigned int)msg.res, (unsigned int)msg.crc);
}
