#include "usart.h"

static UART_HandleTypeDef* usart_handle = &huart1;

/**
 * @brief put_c function
 * @param ch put data
 * @retval None
 */
__attribute__((unused)) static int PutC(uint8_t ch)
{
    uint8_t temp[1] = {ch};

    HAL_UART_Transmit(usart_handle, temp, 1, 0xffff);

    return ch;
}

/**
 * @brief get_c function
 * @param None
 * @retval receive data
 */
__attribute__((unused)) static int GetC(void)
{
    uint8_t ch = 0;

    HAL_UART_Receive(usart_handle, &ch, 1, 0xffff);

    return ch;
}

/**
 * @brief put_str function
 * @param str put string data
 * @retval None
 */
static void PutStr(uint8_t* str)
{
    while (*str != 0) {
        PutC(*str);
        str++;
    }
}

/**
 * @brief printf function
 * @param such as printf format
 * @retval None
 */
void UsartPrintf(const char* fmt, ...)
{
    char buf[256];

    va_list vlist;
    va_start(vlist, fmt);
    vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
    PutStr((uint8_t*)buf);

    va_end(vlist);
}

/**
 * @brief usart put function
 * @param p_data put data Size is data size
 * @retval None
 */
void UsartPuts(uint8_t* p_data, uint16_t size)
{
    HAL_UART_Transmit(usart_handle, p_data, size, 100);

    while (__HAL_UART_GET_FLAG(usart_handle, UART_FLAG_TC) != SET)
        ;
}
