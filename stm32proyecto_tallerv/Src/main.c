/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Main program body / stm32proyecto_tallerv
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


// 1. Includes
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdio.h>


//2. Definicion de variables
volatile uint8_t led_ok = 0;
volatile uint8_t cambio = 0;
uint8_t color = 0;
volatile uint8_t aumentar_Counter = 0;
uint16_t counter = 0;


//3. Definicion de funciones

void init_GPIO(void);
void init_Timers(void);
void init_exti(void);

// ================================================================
// MAIN
// ================================================================
int main(void){

	//4. Bloque de inicializacion
	init_GPIO();
	init_Timers();
	init_exti();

	//5. Loop forever / infinite loop
	while(1){

		//Bandera de counter
		if(aumentar_Counter == 1){
			counter = counter + 10;
			aumentar_Counter = 0;
		}

		if(led_ok){
			led_ok = 0;
			GPIOA->ODR ^= GPIO_ODR_OD5; //Conexion LED por GPIOA
		}

		if (cambio){
			cambio = 0;
			switch(color){
			case(0): //verde
									GPIOA->ODR |= GPIO_ODR_OD8;
			GPIOA->ODR &= ~(GPIO_ODR_OD7);
			GPIOA->ODR &= ~(GPIO_ODR_OD6);
			color++;
			break;

			case(1):
						GPIOA->ODR &= ~(GPIO_ODR_OD8);
			GPIOA->ODR |= (GPIO_ODR_OD7);
			GPIOA->ODR &= ~(GPIO_ODR_OD6);
			color++;
			break;

			case(2):
						GPIOA->ODR &= ~(GPIO_ODR_OD8);
			GPIOA->ODR &= ~(GPIO_ODR_OD7);
			GPIOA->ODR |= GPIO_ODR_OD6;
			color = 0;
			break;


			}
		}

	}

	return 0;
}

// ================================================================
// FUNCIONES
// ================================================================

void init_GPIO(void){

	// Señales de reloj
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enciende la señal del reloj para GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;


	// Configurando GPIOA(MODER, OTYPER, OSPEEDR, PUPDR, ODR)
	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= (GPIO_MODER_MODE5_0);

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);

	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_1);

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);

	GPIOA->ODR |= (GPIO_ODR_OD5);

	// Configurando GPIOB(MODER, OTYPER, OSPEEDR, PUPDR, ODR)
	GPIOB->MODER &= ~(GPIO_MODER_MODE5);
	GPIOB->MODER |= (GPIO_MODER_MODE5_0);

	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT5);

	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_1);

	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD5);

	GPIOB->ODR |= (GPIO_ODR_OD5);

	// Configurando GPIOC(MODER, OTYPER, OSPEEDR, PUPDR, ODR)
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
	GPIOC->MODER |= (GPIO_MODER_MODE13_0);

	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT13);

	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13);
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13_1);

	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);

	GPIOC->ODR |= (GPIO_ODR_OD13);

}

void init_Timers(void){

	/*
	 * TIM2
	 */

	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN);
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->PSC = 16000-1; //El prescaler se pone a 1600 debido a que al dividirse es 16000000 y nos da 0.001s
	TIM2->ARR = 1000-1; //1s

	TIM2->CNT = 0; //Contador en 0 (inicio)

	TIM2->DIER &= ~(TIM_DIER_UIE);
	TIM2->DIER |= TIM_DIER_UIE;

	__NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->CR1 &= ~(TIM_CR1_DIR);

	TIM2->CR1 &= ~(TIM_CR1_ARPE);
	TIM2->CR1 |= TIM_CR1_ARPE;

	TIM2->CR1 |= TIM_CR1_CEN;

	/*
	 * TIM3
	 */

	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN);
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->PSC = 16000-1; //El prescaler se pone a 1600 debido a que al dividirse es 16000000 y nos da 0.001s
	TIM3->ARR = 4000-1; //4s

	TIM3->CNT = 0; //COntador en 0 (inicio)

	TIM3->DIER &= ~(TIM_DIER_UIE);
	TIM3->DIER |= TIM_DIER_UIE;

	__NVIC_EnableIRQ(TIM3_IRQn);

	TIM3->CR1 &= ~(TIM_CR1_DIR);

	TIM3->CR1 &= ~(TIM_CR1_ARPE);
	TIM3->CR1 |= TIM_CR1_ARPE;

	TIM3->CR1 |= TIM_CR1_CEN;

}

// ================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ================================================================

void TIM2_IRQHandler(void){
	if (TIM2->SR & TIM_SR_UIF){
		TIM2->SR &= ~ TIM_SR_UIF;
		led_ok = 1;
	}

}

void TIM3_IRQHandler(void){
	if (TIM3->SR & TIM_SR_UIF){
		TIM3->SR &= ~ TIM_SR_UIF;
		cambio = 1;
	}
}

void init_exti(void){

	//Encendiendo reloj SYSCFG EXTI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	//Configuramos el canal del exti
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);
	//Config canal 1 del exti para el puerto C(pin c1)
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;

	//Seleccionando flanco
	EXTI->RTSR |= EXTI_RTSR_TR1;

	//Registrando en el NVIC la interrupcion para que la atienda
	NVIC_EnableIRQ(EXTI1_IRQn);

	//Bajamos la bandera
	EXTI->PR |= EXTI_PR_PR1;

	//Activamos la interrupcion
	EXTI->IMR |= EXTI_IMR_IM1;

}

//ISR para el EXTI con flanco de subida
void EXTI1_IRQHandler(void){
	if(EXTI->PR & EXTI_PR_PR1){ //vERIFICAMOS LA INTERRUPCION
		EXTI->PR = EXTI_PR_PR1;
		aumentar_Counter = 1;

	}
}
