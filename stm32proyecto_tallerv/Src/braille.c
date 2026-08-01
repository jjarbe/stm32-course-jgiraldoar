/*
 ******************************************************************************
 * @file           : braille.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Tabla de traducción del alfabeto español a Braille
 ******************************************************************************
 */

#include "braille.h"

/*
 * Tabla de traducción (Look-Up Table) del alfabeto español.
 *
 * El índice es la posición de la letra: 0='a', 1='b', ... 25='z', 26='ñ'.
 * Cada valor es la máscara de los puntos activos. La columna de comentario
 * muestra la numeración clásica de los puntos, para poder cotejar la tabla
 * de un vistazo contra cualquier alfabeto Braille de referencia.
 *
 * Vive en Flash (const) y no consume RAM.
 */
static const uint8_t braille_lut[27] =
{
    /* a */ BRAILLE_DOT_1,                                                      /* 1     */
    /* b */ BRAILLE_DOT_1 | BRAILLE_DOT_2,                                      /* 12    */
    /* c */ BRAILLE_DOT_1 | BRAILLE_DOT_4,                                      /* 14    */
    /* d */ BRAILLE_DOT_1 | BRAILLE_DOT_4 | BRAILLE_DOT_5,                      /* 145   */
    /* e */ BRAILLE_DOT_1 | BRAILLE_DOT_5,                                      /* 15    */
    /* f */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_4,                      /* 124   */
    /* g */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_4 | BRAILLE_DOT_5,      /* 1245  */
    /* h */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_5,                      /* 125   */
    /* i */ BRAILLE_DOT_2 | BRAILLE_DOT_4,                                      /* 24    */
    /* j */ BRAILLE_DOT_2 | BRAILLE_DOT_4 | BRAILLE_DOT_5,                      /* 245   */
    /* k */ BRAILLE_DOT_1 | BRAILLE_DOT_3,                                      /* 13    */
    /* l */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_3,                      /* 123   */
    /* m */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_4,                      /* 134   */
    /* n */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_4 | BRAILLE_DOT_5,      /* 1345  */
    /* o */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_5,                      /* 135   */
    /* p */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_3 | BRAILLE_DOT_4,      /* 1234  */
    /* q */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_3 |
            BRAILLE_DOT_4 | BRAILLE_DOT_5,                                      /* 12345 */
    /* r */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_3 | BRAILLE_DOT_5,      /* 1235  */
    /* s */ BRAILLE_DOT_2 | BRAILLE_DOT_3 | BRAILLE_DOT_4,                      /* 234   */
    /* t */ BRAILLE_DOT_2 | BRAILLE_DOT_3 | BRAILLE_DOT_4 | BRAILLE_DOT_5,      /* 2345  */
    /* u */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_6,                      /* 136   */
    /* v */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_3 | BRAILLE_DOT_6,      /* 1236  */
    /* w */ BRAILLE_DOT_2 | BRAILLE_DOT_4 | BRAILLE_DOT_5 | BRAILLE_DOT_6,      /* 2456  */
    /* x */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_4 | BRAILLE_DOT_6,      /* 1346  */
    /* y */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_4 |
            BRAILLE_DOT_5 | BRAILLE_DOT_6,                                      /* 13456 */
    /* z */ BRAILLE_DOT_1 | BRAILLE_DOT_3 | BRAILLE_DOT_5 | BRAILLE_DOT_6,      /* 1356  */
    /* ñ */ BRAILLE_DOT_1 | BRAILLE_DOT_2 | BRAILLE_DOT_4 |
            BRAILLE_DOT_5 | BRAILLE_DOT_6                                       /* 12456 */
};

/*
 * braille_GetMask
 * Convierte el carácter a índice de la tabla y devuelve su máscara.
 * Acepta mayúsculas y minúsculas (en Braille básico no hay distinción de
 * caja; el prefijo de mayúscula sería un signo aparte que no se usa aquí).
 */
uint8_t braille_GetMask(char c)
{
    /* Normalizar a minúscula: en ASCII, 'A'..'Z' está 32 posiciones antes
     * de 'a'..'z', así que sumar 32 convierte la mayúscula                 */
    if (c >= 'A' && c <= 'Z')
    {
        c = (char)(c + 32);
    }

    if (c >= 'a' && c <= 'z')
    {
        return braille_lut[c - 'a'];   /* 'a' es el índice 0 de la tabla    */
    }

    return BRAILLE_INVALID;            /* espacios, dígitos, signos...     */
}

/*
 * braille_GetMaskEnie
 * Máscara de la 'ñ', guardada en la última posición de la tabla.
 */
uint8_t braille_GetMaskEnie(void)
{
    return braille_lut[BRAILLE_IDX_ENIE];
}
