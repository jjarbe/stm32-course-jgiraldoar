/*
 ******************************************************************************
 * @file           : stm32f4xx_it.c   (PRUEBA 2 — micrófono I2S)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Rutinas de servicio de interrupción
 ******************************************************************************
 *
 * Vectores utilizados en esta prueba:
 *   SysTick_Handler          → base de tiempo de 1 ms del HAL
 *   TIM1_UP_TIM10_IRQHandler → tick de 250 ms de TIM10 (LED de estado)
 *   DMA1_Stream3_IRQHandler  → transferencias del DMA desde el I2S2.
 *                              Es el stream que el hardware tiene asignado
 *                              a la petición de RECEPCIÓN de SPI2/I2S2
 *                              (el Stream 4 es el de transmisión, usado en
 *                              la prueba del amplificador).
 *   USART2_IRQHandler        → puerto serial
 *
 * El I2C de la pantalla no aparece: se maneja por sondeo.
 *
 * Aunque el DMA mueve las muestras por sí solo, esta ISR es necesaria: es
 * la que permite al HAL seguir el progreso de la transferencia e invocar
 * los callbacks de media transferencia y transferencia completa, que son el
 * mecanismo de doble buffer de la aplicación.
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef  htim10;
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

/* Recepción del DMA desde el I2S2 (media transferencia y completa) */
void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_i2s2_rx);
}

/* Puerto serial */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
