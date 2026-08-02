/*
 ******************************************************************************
 * @file           : clips_voz.h
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Clips de voz pregrabados para la confirmación sonora
 ******************************************************************************
 *
 * Las tres palabras se grabaron una sola vez con el programa de la grabadora
 * y quedaron incrustadas en el firmware como datos constantes. Al declararse
 * 'const', el compilador las ubica en Flash y no consumen RAM: solo se leen
 * muestra a muestra desde el callback de audio.
 *
 * Formato: muestras de 16 bits con signo a 16 kHz, mono. Es el mismo formato
 * en que las entrega el micrófono, así que reproducirlas es copiar valores
 * al buffer de salida sin ninguna conversión.
 ******************************************************************************
 */

#ifndef CLIPS_VOZ_H
#define CLIPS_VOZ_H

#include <stdint.h>

/* Palabra "uno" */
extern const int16_t  clip_uno[];
extern const uint32_t clip_uno_largo;

/* Palabra "dos" */
extern const int16_t  clip_dos[];
extern const uint32_t clip_dos_largo;

/* Palabra "tres" */
extern const int16_t  clip_tres[];
extern const uint32_t clip_tres_largo;

#endif /* CLIPS_VOZ_H */
