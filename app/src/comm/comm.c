/**
 * @file bettery.c
 *
 * bettery mananger module.
 */
 
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "comm.h"
#include "bettery.h"
#include <stdio.h>

#define  COMM_DATA_BUF_MAX           (32)

SPI_HandleTypeDef hspi1;
uint16_t comm_state = COMM_STATE_READY;
uint32_t comm_spi_intr_cnt;
uint32_t comm_spi_rx_cnt;
uint32_t comm_spi_tx_cnt;

uint8_t  comm_data[COMM_DATA_BUF_MAX];
uint16_t  comm_data_len;

void comm_spi_rx_isr(SPI_HandleTypeDef *hspi);
void comm_spi_tx_isr(SPI_HandleTypeDef *hspi);
void comm_spi_intr_hnd(SPI_HandleTypeDef *hspi);
void SPI2_IRQHandler(void);

void comm_spi_init(void)
{
    hspi1.Instance = SPI2;
    hspi1.Init.Mode = SPI_MODE_SLAVE;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    HAL_SPI_Init(&hspi1);

    __HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_RXNE | SPI_IT_ERR));
}

void comm_spi_enable(void)
{
    HAL_NVIC_SetPriority(SPI2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);

    __HAL_SPI_ENABLE(&hspi1);
}

void comm_spi_rx_isr(SPI_HandleTypeDef *hspi)
{
    uint8_t addr;
    int16_t ret;

    comm_spi_rx_cnt++;
    addr = hspi->Instance->DR;
    switch (comm_state)
    {
        case COMM_STATE_READY:
            ret = batteryInfoGet(addr, comm_data, COMM_DATA_BUF_MAX);
            if (ret > 0)
            {
                comm_data_len = ret;
                hspi->Instance->DR = comm_data[0];
                comm_data_len--;
                comm_state = COMM_STATE_MESSAGE;
            }
            break;
        case COMM_STATE_MESSAGE:
            if (comm_data_len == 0)
            {
                comm_state = COMM_STATE_READY;
            }
            else
            {
                hspi->Instance->DR = comm_data[1];
                comm_data_len--;
            }
            break;
        default:
            break;
    }

    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    
}

void comm_spi_tx_isr(SPI_HandleTypeDef *hspi)
{
    comm_spi_tx_cnt++;
    hspi->Instance->DR = 0x22;
    /*
    switch (comm_state)
    {
        case COMM_STATE_READY:
            hspi->Instance->DR = comm_data[1];
            break;
        case COMM_STATE_MESSAGE:
            if (comm_data_len == 0)
            {
                hspi->Instance->DR = comm_data[1];
                //__HAL_SPI_DISABLE_IT(&hspi1, SPI_IT_TXE);
                //comm_state = COMM_STATE_READY;
            }
            else
            {
                hspi->Instance->DR = 0x11;
                //hspi->pTxBuffPtr++;
                comm_data_len--;
            }
            break;
        default:
            hspi->Instance->DR = 0xff;
            break;
    }
    */
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
}

void comm_spi_intr_hnd(SPI_HandleTypeDef *hspi)
{
    /* SPI in mode Receiver and Overrun not occurred ---------------------------*/
    if((__HAL_SPI_GET_IT_SOURCE(hspi, SPI_IT_RXNE) != RESET) 
    && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) != RESET) 
    && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_OVR) == RESET))
    {
        //printf("r");
        comm_spi_rx_isr(hspi);
        return;
    }

    /* SPI in mode Tramitter ---------------------------------------------------*/
    if((__HAL_SPI_GET_IT_SOURCE(hspi, SPI_IT_TXE) != RESET) 
    && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) != RESET))
    {
        comm_spi_tx_isr(hspi);
        //printf("t");
        return;
    }

    if(__HAL_SPI_GET_IT_SOURCE(hspi, SPI_IT_ERR) != RESET)
    {
        //printf("e");
        /* SPI CRC error interrupt occurred ---------------------------------------*/
        if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
            __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
        }
        
        /* SPI Mode Fault error interrupt occurred --------------------------------*/
        if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_MODF) != RESET)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
            __HAL_SPI_CLEAR_MODFFLAG(hspi);
        }

        /* SPI Overrun error interrupt occurred -----------------------------------*/
        if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_OVR) != RESET)
        {
            if(hspi->State != HAL_SPI_STATE_BUSY_TX)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
                __HAL_SPI_CLEAR_OVRFLAG(hspi);      
            }
        }

        /* Call the Error call Back in case of Errors */
        if(hspi->ErrorCode!=HAL_SPI_ERROR_NONE)
        {
            //printf("dis intr\r\n");
            //__HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
            hspi->State = HAL_SPI_STATE_READY;
            HAL_SPI_ErrorCallback(hspi);
        }
    }
}


void SPI2_IRQHandler(void)
{
    comm_spi_intr_cnt++;
    comm_spi_intr_hnd(&hspi1);
}
