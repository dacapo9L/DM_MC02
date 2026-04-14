// #include "bsp_usart.h"
// #include <stdio.h>
// #include <string.h>

// /*usart1*/

// /** @brief 接收数据缓冲区 */
// static uint8_t receivedData_1[64];

// /** @brief 接收数据长度 */
// static uint16_t dataLength_1 = 0;

// /** @brief USART1标志位 */
// uint8_t usart1_flag = 0;

// /** @brief USART1模式 */
// uint8_t usart1_mode = 0;

// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart1_rx;
// extern DMA_HandleTypeDef hdma_usart1_tx;

// uint8_t usart1data[64];

// /*usart2*/

// /** @brief 接收数据缓冲区 */
// static uint8_t receivedData_2[64];

// /** @brief 接收数据长度 */
// static uint16_t dataLength_2 = 0;

// /** @brief USART2标志位 */
// uint8_t usart2_flag = 0;

// /** @brief USART2模式 */
// uint8_t usart2_mode = 0;

// extern UART_HandleTypeDef huart2;
// extern DMA_HandleTypeDef hdma_usart2_rx;
// extern DMA_HandleTypeDef hdma_usart2_tx;

// uint8_t usart2data[64];

// /*usart3*/

// /** @brief 接收数据缓冲区 */
// static uint8_t receivedData_3[64];

// /** @brief 接收数据长度 */
// static uint16_t dataLength_3 = 0;

// /** @brief USART3标志位 */
// uint8_t usart3_flag = 0;

// /** @brief USART3模式 */
// uint8_t usart3_mode = 0;

// extern UART_HandleTypeDef huart3;
// extern DMA_HandleTypeDef hdma_usart3_rx;
// extern DMA_HandleTypeDef hdma_usart3_tx;

// uint8_t usart3data[64];

// /*uart7*/

// /** @brief 接收数据缓冲区 */
// static uint8_t receivedData_7[64];

// /** @brief 接收数据长度 */
// static uint16_t dataLength_7 = 0;

// /** @brief UART7标志位 */
// uint8_t uart7_flag = 0;

// /** @brief UART7模式 */
// uint8_t uart7_mode = 0;

// extern UART_HandleTypeDef huart7;
// extern DMA_HandleTypeDef hdma_uart7_rx;
// extern DMA_HandleTypeDef hdma_uart7_tx;

// uint8_t uart7data[64];

// /*usart10*/

// /** @brief 接收数据缓冲区 */
// static uint8_t receivedData_10[64];

// /** @brief 接收数据长度 */
// static uint16_t dataLength_10 = 0;

// /** @brief USART10标志位 */
// uint8_t usart10_flag = 0;

// /** @brief USART10模式 */
// uint8_t usart10_mode = 0;

// extern UART_HandleTypeDef huart10;
// extern DMA_HandleTypeDef hdma_usart10_rx;
// extern DMA_HandleTypeDef hdma_usart10_tx;

// uint8_t usart10data[64];
