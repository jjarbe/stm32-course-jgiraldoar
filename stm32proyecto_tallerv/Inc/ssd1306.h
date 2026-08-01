/*
 ******************************************************************************
 * @file           : ssd1306.h
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Driver de la pantalla OLED SSD1306 128x64 por I2C
 ******************************************************************************
 *
 * El driver mantiene un framebuffer en RAM del MCU (1024 bytes = 128 columnas
 * x 8 páginas). Todas las funciones de dibujo escriben SOLO en ese buffer;
 * nada llega al panel hasta llamar a SSD1306_UpdateScreen(), que lo vuelca
 * por I2C. Esto evita parpadeos y reduce el tráfico del bus: se dibuja la
 * escena completa y se envía una sola vez.
 *
 * La comunicación es por sondeo (HAL_I2C_Mem_Write), sin interrupciones.
 ******************************************************************************
 */

#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx_hal.h"

/* Dimensiones del panel */
#define SSD1306_WIDTH    128
#define SSD1306_HEIGHT   64

/* Dirección I2C del controlador, ya desplazada a 8 bits (0x3C << 1).
 * El HAL espera la dirección desplazada; si el módulo no responde, la
 * alternativa de fábrica es 0x7A (0x3D << 1).                              */
#define SSD1306_ADDR     0x78

/* Comandos usados para el modo de ahorro de energía */
#define SSD1306_DISP_OFF 0xAE
#define SSD1306_DISP_ON  0xAF

/*
 * SSD1306_SetI2C
 * Registra el handle de I2C que usará el driver. Debe llamarse una vez,
 * después de inicializar el periférico y antes de SSD1306_Init().
 * Mantener el handle aquí evita que el driver dependa de una variable
 * global de main.c.
 */
void SSD1306_SetI2C(I2C_HandleTypeDef *hi2c);

/* Secuencia de arranque del controlador */
void SSD1306_Init(void);

/* Envía un byte de comando al controlador (D/C# = 0) */
void SSD1306_WriteCmd(uint8_t cmd);

/* Rellena todo el framebuffer: color 0 = apagado, distinto de 0 = encendido */
void SSD1306_Fill(uint8_t color);

/* Vuelca el framebuffer al panel por I2C (una transferencia por página) */
void SSD1306_UpdateScreen(void);

/* Enciende (color != 0) o apaga (color == 0) un píxel del framebuffer */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/* Escribe una cadena con la fuente 5x7 desde (x,y); avance de 6 px */
void SSD1306_WriteString(uint8_t x, uint8_t y, const char *str);

/*
 * Escribe un carácter ampliado por un factor entero: cada píxel de la
 * fuente se dibuja como un bloque de scale x scale. Con scale = 4 el
 * carácter mide 20x28 px, suficiente para leerse a distancia.
 */
void SSD1306_WriteCharScaled(uint8_t x, uint8_t y, char c, uint8_t scale);

/* Rectángulo sin relleno: (x,y) esquina superior izquierda, w x h de tamaño */
void SSD1306_DrawEmptyRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/* Círculo relleno de radio r centrado en (xc,yc) — puntos Braille activos */
void SSD1306_DrawFilledCircle(uint8_t xc, uint8_t yc, uint8_t r);

/* Línea horizontal de y constante, desde x0 hasta x1 inclusive */
void SSD1306_DrawHLine(uint8_t x0, uint8_t x1, uint8_t y);

/*
 * Desatasca el bus I2C generando pulsos de reloj manuales. Debe llamarse
 * ANTES de inicializar el periférico I2C. Recupera el sistema de un reset
 * ocurrido en mitad de una transferencia, situación en la que el módulo
 * deja la línea SDA bloqueada en nivel bajo.
 */
void SSD1306_BusRecover(GPIO_TypeDef *scl_port, uint16_t scl_pin,
                        GPIO_TypeDef *sda_port, uint16_t sda_pin);

#endif /* SSD1306_H */
