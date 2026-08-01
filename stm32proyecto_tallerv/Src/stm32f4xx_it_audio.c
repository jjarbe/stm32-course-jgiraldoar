/*
 ******************************************************************************
 * @file           : stm32f4xx_it.c   (PRUEBA 1 — amplificador I2S)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Rutinas de servicio de interrupción
 ******************************************************************************
 *
 * Vectores utilizados en esta prueba:
 *   SysTick_Handler          → base de tiempo de 1 ms del HAL (HAL_Delay y
 *                              los timeouts de las transferencias I2C/UART)
 *   TIM1_UP_TIM10_IRQHandler → tick de 250 ms de TIM10 (LED de estado).
 *                              Vector compartido con el update de TIM1, que
 *                              en esta prueba no se usa.
 *   DMA1_Stream4_IRQHandler  → transferencias del DMA hacia el I2S2. Es el
 *                              stream que el hardware tiene asignado a la
 *                              petición de transmisión de SPI2/I2S2.
 *   USART2_IRQHandler        → recepción de caracteres del puerto serial
 *
 * Deliberadamente NO aparece aquí el I2C1: la pantalla se maneja por sondeo.
 *
 * Sobre la interrupción del DMA: aunque el tono se sostiene solo, el HAL
 * necesita esta ISR para mantener el estado del handle y para invocar los
 * callbacks de media transferencia y transferencia completa. Sin ella, el
 * DMA seguiría moviendo datos pero el HAL nunca se enteraría del progreso y
 * HAL_I2S_DMAStop() no funcionaría correctamente.
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"

/* Handles definidos en main.c. Se declaran 'extern' porque cada
 * HAL_xxx_IRQHandler() necesita recibir el handle para saber sobre qué
 * instancia y con qué configuración está trabajando.                       */
extern TIM_HandleTypeDef  htim10;
extern DMA_HandleTypeDef  hdma_i2s2_tx;
extern UART_HandleTypeDef huart2;

/* Base de tiempo de 1 ms del HAL: incrementa el contador interno uwTick */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Tick de 250 ms: LED de estado */
void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim10);
}

/* Transferencias del DMA hacia el I2S2 (media transferencia y completa) */
void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_i2s2_tx);
}

/* Recepción de caracteres (y eventos de error) del puerto serial */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
