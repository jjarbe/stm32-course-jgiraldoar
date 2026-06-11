/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Main program body
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

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdio.h>


//Definicion de variables
volatile uint8_t led_ok = 0;
volatile uint8_t cambio = 0;
uint8_t color = 0;

volatile uint8_t aumentar_Counter = 0;
uint16_t counter = 0;


//Definicion de funciones

void init_hardware(void);
void init_GPIO(void);
void init_exti(void);


//MAIN

int main(void){
	init_GPIO();
	init_hardware();

	//Encendiendo led LD2
	//RCC->AHB1ENR |= (1<<0);
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//Pin A5 como salida

	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= (GPIO_MODER_MODE5_0);
	//Pin A5 como salida push pull
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);
	//Limpiando posicion de los bits que deseo borrar
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);
	//Seleccionando velocidad fast
	GPIOA->OSPEEDR |= ~(GPIO_OSPEEDR_OSPEED5_1);

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);
	//Escribir un 1 en la posicion 5
	GPIOA->ODR |= (GPIO_ODR_OD5);



	GPIOC->MODER &= ~GPIO_MODER_MODE13;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13;


	/* Loop forever */
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



		//		if ((GPIOC->IDR & (0b1 << 13)) == 0){
		//			GPIOA->ODR |= GPIO_ODR_OD5;
	}

	//		else{
	//			GPIOA->ODR &= ~GPIO_ODR_OD5;
	//			  }

	//		GPIOA->ODR |= GPIO_ODR_OD5;

	//		for (volatile uint32_t i = 0; i<1000000; i++);

	//		GPIOA->ODR &= ~GPIO_ODR_OD5;

	//		for (volatile uint32_t i = 0; i<1000000; i++);


	//	}
	return 0;

}

//FUNCIONES

void init_hardware(void){

	RCC->AHB1ENR |= (0b1<<0); //Enciende la señal del reloj para GPIOA

	RCC->AHB1ENR |= (0b1<<2); //Enciende la señal del reloj para GPIOC

	/*
	 * Para PA5
	 */

	GPIOA->MODER &= ~ (0b11 << 5*2); //Ponemos en 0 (por precaución) estos registros.
	GPIOA->MODER |= (0b1 << 5*2); //Ponemos el MODER5 en [0,1


	GPIOA->OTYPER &= ~ (0b1 << 5); //Output push/pull

	GPIOA->OSPEEDR &= ~(0b11 << 5*2); // Limpia los bits 10 y 11
	GPIOA->OSPEEDR |= (0b1 << 11); //Fast Speed

	GPIOA->PUPDR &= ~ (0b11 << 5*2); //No PUPDR

	GPIOA->ODR |= (0b1 << 5); //Salida en AltoGPIOC->OTYPER &= ~ (0b1 << 5); //Output push/pull

	GPIOC->OSPEEDR &= ~(0b11 << 5*2); // Limpia los bits 10 y 11
	GPIOC->OSPEEDR |= (0b1 << 11); //Fast Speed

	GPIOC->ODR |= (0b1 << 5); //Salida en Alto



	/*
	 * Para PC13
	 */


	GPIOC->MODER &= ~ (0b11 << 13*2); //Ponemos en 0 (por precaución) estos registros. (input)
	GPIOC->PUPDR &= ~ (0b11 << 5*2); //No PUPDR

}

void init_GPIO(void){
	/*
	 * Señal de reloj
	 */
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


	/*
	 * PA5 -> PA8
	 */

	GPIOA->MODER &= ~ (GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
	GPIOA->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0);

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6| GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);

	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8);

	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR8_1);

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);

	GPIOA->ODR |= (GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD7 | GPIO_ODR_OD8);

	RCC->AHB1ENR |= (0b1<<0); //Enciende la señal del reloj para GPIOA

	RCC->AHB1ENR |= (0b1<<2); //Enciende la señal del reloj para GPIOC

	//GPIOC
	GPIOC->MODER &= ~(GPIO_MODER_MODE1); //Ponemos en 0 (por precaución) estos registros.
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1); //No PUPDR


	/*
	 * Para PC13
	 */


	GPIOC->MODER &= ~ (0b11 << 13*2); //Ponemos en 0 (por precaución) estos registros. (input)
	GPIOC->PUPDR &= ~ (0b11 << 5*2); //No PUPDR


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
	if(EXTI->PR && EXTI_PR_PR1){ //vERIFICAMOS LA INTERRUPCION
		if(EXTI->PR |= EXTI_PR_PR1);
		aumentar_Counter = 1;

	}
}
