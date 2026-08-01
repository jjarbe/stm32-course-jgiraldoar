/*
 ******************************************************************************
 * @file           : braille.h
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Tabla de traducción del alfabeto español a Braille
 ******************************************************************************
 *
 * REPRESENTACIÓN DEL SIGNO GENERADOR
 *
 * La celda Braille tiene 6 puntos numerados así (dos columnas de tres):
 *
 *        1  •  •  4
 *        2  •  •  5
 *        3  •  •  6
 *
 * Cada letra se guarda como una MÁSCARA DE BITS en un solo uint8_t:
 *
 *        bit 0 → punto 1        bit 3 → punto 4
 *        bit 1 → punto 2        bit 4 → punto 5
 *        bit 2 → punto 3        bit 5 → punto 6
 *
 * Ventaja de la máscara frente a una matriz de 26x6 booleanos:
 *   - 27 bytes en Flash en lugar de 162
 *   - consultar un punto es una operación de máscara: (m & BRAILLE_DOT_n)
 *   - el MISMO byte sirve para dibujar la celda en la OLED y para decidir
 *     la posición de los 6 servos, así que no hay dos representaciones que
 *     puedan desincronizarse
 ******************************************************************************
 */

#ifndef BRAILLE_H
#define BRAILLE_H

#include <stdint.h>

/* Máscaras de cada punto del signo generador */
#define BRAILLE_DOT_1   (1u << 0)
#define BRAILLE_DOT_2   (1u << 1)
#define BRAILLE_DOT_3   (1u << 2)
#define BRAILLE_DOT_4   (1u << 3)
#define BRAILLE_DOT_5   (1u << 4)
#define BRAILLE_DOT_6   (1u << 5)

/* Valor devuelto cuando el carácter no tiene traducción (espacios, signos) */
#define BRAILLE_INVALID 0xFFu

/* Índice reservado para la 'ñ' dentro de la tabla */
#define BRAILLE_IDX_ENIE 26u

/*
 * braille_GetMask
 * Traduce un carácter a su máscara de 6 bits.
 *   c : letra 'a'..'z' (acepta también mayúsculas) o el código especial
 *       de la 'ñ' (ver braille_GetMaskEnie)
 * Devuelve la máscara, o BRAILLE_INVALID si el carácter no es una letra.
 */
uint8_t braille_GetMask(char c);

/*
 * braille_GetMaskEnie
 * Devuelve la máscara de la 'ñ'. Existe como función aparte porque la 'ñ'
 * no es ASCII: por UART llega como la secuencia UTF-8 de dos bytes
 * 0xC3 0xB1, que se detecta en la recepción y no cabe en un char.
 */
uint8_t braille_GetMaskEnie(void);

#endif /* BRAILLE_H */
