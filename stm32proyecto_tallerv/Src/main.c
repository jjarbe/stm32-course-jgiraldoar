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

#include <stdint.h>
#include <stm32f4xx.h>


//Declaracion de variables homework
uint8_t my_variable = 0;

//Fin declaracion de variables homework



//Definicion de variables del sistema
uint8_t var_a = 0;
uint16_t var_b = 0;
uint32_t var_c = 0;

//Definicion de funciones

void init_hardware(void);


//MAIN

int main(void){
	init_hardware();

	//Inicio codigo homework

	//Fin codigo homework

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

		if ((GPIOC->IDR & (0b1 << 13)) == 0){
			GPIOA->ODR |= GPIO_ODR_OD5;
			  }

		else{
			GPIOA->ODR &= ~GPIO_ODR_OD5;
			  }

//		GPIOA->ODR |= GPIO_ODR_OD5;

//		for (volatile uint32_t i = 0; i<1000000; i++);

//		GPIOA->ODR &= ~GPIO_ODR_OD5;

//		for (volatile uint32_t i = 0; i<1000000; i++);


	}
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

	GPIOA->ODR |= (0b1 << 5); //Salida en Alto


	/*
	 * Para PC13
	 */


	GPIOC->MODER &= ~ (0b11 << 13*2); //Ponemos en 0 (por precaución) estos registros. (input)
	GPIOC->PUPDR &= ~ (0b11 << 5*2); //No PUPDR

}

