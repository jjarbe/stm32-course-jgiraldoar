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


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

//Declaracion de variables homework
uint8_t my_variable = 0;

uint8_t dec = 0;
uint8_t hex = 0;
uint8_t bin = 0;

uint8_t a = 0;
uint16_t b= 0;
uint32_t c = 0;
uint8_t d = 0;
uint8_t e = 0;


//Fin declaracion de variables homework



//Definicion de variables del sistema
uint8_t var_a = 0;
uint16_t var_b = 0;
uint32_t var_c = 0;

uint8_t overflow_demo  = 0;


uint16_t var_a_dec = 0;
uint16_t var_b_bin = 0;
uint16_t var_c_hex = 0;




int main(void)
{

	//Inicio codigo homework
	my_variable = 42;

	dec = 65;
	hex = 0x41;
	bin = 0b01000001;

	a = 255;
	b = 255;
	c = 255;
	d = 256;
	e = 257;

	//continuar en ejercicio 0.4

	//Fin codigo homework

	var_a = 100;
	var_b = 4968;
	var_c = 12345678;

	var_a_dec = 32;
	var_b_bin = 0b100000;
	var_c_hex = 0x20;

	//cargando el valor xxx en la variable yyy
	var_b_bin = var_b_bin << 3; //prediccion: 0b10000000 = 256
	var_b_bin = var_b_bin >> 3; //prediccion: 0b100 = 4 :c

	//exponiendo el caso de un overflow
	var_a = 255;
	var_b = 255;
	var_c = 255;

	//incremento el valor de la variable 8bit en 1 y lo cargo en la variable overflow_demo
	overflow_demo = var_a + 1;
	overflow_demo = overflow_demo + 1;

	overflow_demo = 735;
	overflow_demo = 0;

	for(uint16_t counter = 0; counter < 735; counter ++){
		overflow_demo++;
	}

    /* Loop forever */
	while(1){


	}

	return 0;
}
