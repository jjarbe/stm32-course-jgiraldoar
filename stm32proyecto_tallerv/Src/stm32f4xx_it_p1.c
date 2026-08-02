/*
 ******************************************************************************
 * @file           : stm32f4xx_it.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Rutinas de servicio de interrupción
 ******************************************************************************
 *
 * Vectores utilizados en esta etapa del proyecto:
 *   SysTick_Handler          → base de tiempo de 1 ms del HAL (HAL_Delay,
 *                              timeouts de las transferencias I2C y UART)
 *   TIM1_UP_TIM10_IRQHandler → tick de 250 ms de TIM10: blinky de estado y
 *                              base de tiempo del secuenciador de letras.
 *                              El vector es compartido con el update de
 *                              TIM1, pero TIM1 (encoder) no tiene
 *                              interrupciones habilitadas, así que no hay
 *                              ambigüedad sobre quién la generó.
 *   USART2_IRQHandler        → recepción de caracteres del puerto serial
 *
 * Deliberadamente NO aparecen aquí:
 *   - I2C1: la pantalla se maneja por sondeo, sin interrupciones.
 *   - TIM1: el encoder se decodifica por hardware y se lee por sondeo del
 *     registro CNT, así que no necesita ninguna interrupción.
 *   - PA10 (pulsador): se lee por sondeo con detección de flanco en el lazo
 *     principal; un botón no justifica una línea de interrupción externa.
 *
 * Cada handler solo delega en el HAL_xxx_IRQHandler() correspondiente, que
 * identifica la causa exacta, limpia las banderas del periférico y llama al
 * callback de la aplicación (definido en main.c).
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"

/* Handles definidos en main.c. Se declaran 'extern' porque los
 * HAL_xxx_IRQHandler() necesitan recibirlos para saber sobre qué instancia
 * y con qué configuración están trabajando.                                */
extern TIM_HandleTypeDef  htim10;
extern UART_HandleTypeDef huart2;

/* Base de tiempo de 1 ms del HAL: incrementa el contador interno uwTick */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Tick de 250 ms: blinky de estado y secuenciador de letras */
void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim10);
}

/* Recepción de caracteres (y eventos de error) del puerto serial */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
