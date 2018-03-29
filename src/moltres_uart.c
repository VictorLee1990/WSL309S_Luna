#include "moltres_uart.h"

uint8_t uart2_rx_buff[BUFFER_SIZE];
//uint8_t uart2_fifo_buff[UART2_BUFF_SIZE];
//app_fifo_t uart2Fifo;
extern uint8_t lora_rx_fifo_buff[BUFFER_SIZE];
extern app_fifo_t lora_rxFifo;
extern void uart2_event_handle_schedule(void *p_event_data, uint16_t event_size);

void start_uart2_uart(void)
{
	app_fifo_init(&lora_rxFifo,lora_rx_fifo_buff,BUFFER_SIZE);
	HAL_UART_RxCpltCallback(&huart2);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		uint32_t uart2_rx_rec_size;

		uart2_rx_rec_size = huart->RxXferSize - huart->hdmarx->Instance->CNDTR;
		if(uart2_rx_rec_size)
		{
			app_fifo_write(&lora_rxFifo,uart2_rx_buff,&uart2_rx_rec_size);
			app_sched_event_put(NULL, NULL, uart2_event_handle_schedule);				
		}

		HAL_UART_AbortReceive(huart);
		HAL_UART_Receive_DMA(huart,uart2_rx_buff,BUFFER_SIZE);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	}		
}

void MOLTRES_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
	
		  HAL_UART_RxCpltCallback(huart);
	}
}

uint8_t* FindSentence(uint8_t *pStr, uint16_t count)
{
    while (count>=2)
    {
        if (*pStr=='\r' && *(pStr+1)=='\n')
            return pStr;
        pStr++;
        count--;
    }
    return NULL;
}

