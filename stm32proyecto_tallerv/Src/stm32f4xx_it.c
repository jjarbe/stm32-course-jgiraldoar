/*
 ******************************************************************************
 * @file           : stm32f4xx_it.c   (PRUEBA 3 — I2S full-duplex)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Rutinas de servicio de interrupción
 ******************************************************************************
 *
 * Vectores utilizados:
 *   SysTick_Handler          → base de tiempo de 1 ms del HAL
 *   TIM1_UP_TIM10_IRQHandler → tick de 250 ms (LED de estado)
 *   DMA1_Stream4_IRQHandler  → TRANSMISIÓN hacia el amplificador
 *                              (petición SPI2_TX, canal 0)
 *   DMA1_Stream3_IRQHandler  → RECEPCIÓN desde el micrófono
 *                              (petición I2S2ext_RX, canal 3)
 *   USART2_IRQHandler        → comandos por el puerto serial
 *
 * El full-duplex necesita los DOS handlers de DMA: cada bloque del
 * periférico (el principal SPI2 y el auxiliar I2S2ext) tiene su propio
 * stream. Los callbacks conjuntos de la aplicación
 * (HAL_I2SEx_TxRxHalfCpltCallback y HAL_I2SEx_TxRxCpltCallback) los invoca
 * el HAL cuando ambas direcciones han alcanzado el mismo punto de la
 * transferencia, y para eso necesita ver el progreso de los dos streams.
 *
 * El I2C de la pantalla no aparece: funciona por sondeo.
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef  htim10;
extern DMA_HandleTypeDef  hdma_i2s2_tx;
extern DMA_HandleTypeDef  hdma_i2s2_rx;
extern UART_HandleTypeDef huart2;

/* Base de tiempo de 1 ms del HAL */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Tick de 250 ms: LED de estado */
void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim10);
}

/* Transmisión del DMA hacia el I2S2 (bloque principal) */
void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_i2s2_tx);
}

/* Recepción del DMA desde el I2S2ext (bloque auxiliar) */
void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_i2s2_rx);
}

/* Comandos por el puerto serial */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
