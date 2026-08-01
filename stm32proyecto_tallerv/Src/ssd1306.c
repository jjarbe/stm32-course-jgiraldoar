/*
 ******************************************************************************
 * @file           : ssd1306.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Driver de la pantalla OLED SSD1306 128x64 por I2C
 ******************************************************************************
 */

#include "ssd1306.h"
#include <string.h>

/* Handle de I2C con el que se habla al panel (lo registra SSD1306_SetI2C) */
static I2C_HandleTypeDef *ssd_i2c = NULL;

/*
 * Framebuffer: 128 columnas x 8 páginas. Cada byte cubre 8 filas de una
 * columna; dentro del byte, el bit 0 es la fila superior de la página.
 * Este formato es exactamente el que espera la GDDRAM del SSD1306, así que
 * el volcado es una copia directa sin conversiones.
 */
static uint8_t ssd_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/*
 * Fuente 5x7, ASCII imprimible de 32 (espacio) a 126 (~).
 * Cada glifo son 5 bytes = 5 columnas; en cada byte el bit 0 es la fila
 * superior. Vive en Flash (const).
 */
static const uint8_t Font5x7[][5] =
{
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x0C,0x02,0x0C,0x02,0x0C},
};

/* ═══════════════════════ Comunicación con el panel ════════════════════════ */

void SSD1306_SetI2C(I2C_HandleTypeDef *hi2c)
{
    ssd_i2c = hi2c;
}

/*
 * SSD1306_WriteCmd
 * Escribir en el "registro de control" 0x00 pone el bit D/C# a 0, lo que
 * indica al controlador que el byte siguiente es un COMANDO y no datos de
 * píxel. Timeout corto (10 ms): un comando son 3 bytes en el bus.
 */
void SSD1306_WriteCmd(uint8_t cmd)
{
    /* Timeout de 100 ms. El valor original de 10 ms resultó demasiado
     * ajustado: con el bus cargado por el resto del cableado, una
     * transferencia lenta se abortaba y el comando se perdía en silencio,
     * dejando el panel sin inicializar sin ninguna señal de error.        */
    HAL_I2C_Mem_Write(ssd_i2c, SSD1306_ADDR, 0x00, 1, &cmd, 1, 100);
}

/*
 * SSD1306_Init
 * Secuencia de arranque del SSD1306. Deja el controlador en modo de
 * direccionamiento por páginas, panel de 128x64, con la bomba de carga
 * interna activada (obligatoria: el módulo se alimenta solo del riel
 * lógico, no tiene alimentación de panel separada). El panel permanece
 * apagado hasta el último comando para no mostrar basura mientras se
 * configura.
 */
void SSD1306_Init(void)
{
    HAL_Delay(100);            /* estabilización tras el encendido          */

    SSD1306_WriteCmd(0xAE);    /* panel OFF durante la configuración        */
    SSD1306_WriteCmd(0x20);    /* modo de direccionamiento de memoria...    */
    SSD1306_WriteCmd(0x10);    /* ...por páginas                            */
    SSD1306_WriteCmd(0xB0);    /* página de inicio = 0                      */
    SSD1306_WriteCmd(0xC8);    /* escaneo COM invertido (orientación)       */
    SSD1306_WriteCmd(0x00);    /* columna de inicio, nibble bajo = 0        */
    SSD1306_WriteCmd(0x10);    /* columna de inicio, nibble alto = 0        */
    SSD1306_WriteCmd(0x40);    /* línea de inicio de display = 0            */
    SSD1306_WriteCmd(0x81);    /* control de contraste...                   */
    SSD1306_WriteCmd(0xFF);    /* ...máximo                                 */
    SSD1306_WriteCmd(0xA1);    /* remapeo de segmentos (orientación H)      */
    SSD1306_WriteCmd(0xA6);    /* display normal: bit 1 en RAM = píxel ON   */
    SSD1306_WriteCmd(0xA8);    /* razón de multiplexado...                  */
    SSD1306_WriteCmd(0x3F);    /* ...1:64 (panel de 64 filas)               */
    SSD1306_WriteCmd(0xA4);    /* mostrar el contenido de la GDDRAM         */
    SSD1306_WriteCmd(0xD3);    /* offset vertical...                        */
    SSD1306_WriteCmd(0x00);    /* ...ninguno                                */
    SSD1306_WriteCmd(0xD5);    /* divisor de reloj / frecuencia del osc...  */
    SSD1306_WriteCmd(0xF0);    /* ...frecuencia alta, divisor 1             */
    SSD1306_WriteCmd(0xD9);    /* periodo de precarga...                    */
    SSD1306_WriteCmd(0x22);    /* ...fase1 = 2 DCLK, fase2 = 2 DCLK         */
    SSD1306_WriteCmd(0xDA);    /* configuración de pines COM...             */
    SSD1306_WriteCmd(0x12);    /* ...alternativa, sin remapeo izq/der       */
    SSD1306_WriteCmd(0xDB);    /* nivel de deselección VCOMH...             */
    SSD1306_WriteCmd(0x20);    /* ...~0.77 x VCC                            */
    SSD1306_WriteCmd(0x8D);    /* bomba de carga...                         */
    SSD1306_WriteCmd(0x14);    /* ...habilitada                             */
    SSD1306_WriteCmd(0xAF);    /* panel ON                                  */
}

/*
 * SSD1306_UpdateScreen
 * Vuelca el framebuffer a la GDDRAM, una página (8 filas) por transferencia.
 * Antes de cada página se reposiciona el puntero de escritura del
 * controlador; escribir en el registro 0x40 pone D/C# a 1, indicando que
 * lo que sigue son DATOS de píxel. El puntero de columna autoincrementa,
 * así que los 128 bytes de la página van en una sola transferencia.
 */
void SSD1306_UpdateScreen(void)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        SSD1306_WriteCmd(0xB0 + page);   /* seleccionar página              */
        SSD1306_WriteCmd(0x00);          /* columna 0, nibble bajo          */
        SSD1306_WriteCmd(0x10);          /* columna 0, nibble alto          */

        /* Timeout de 200 ms: son 128 bytes por página, la transferencia
         * más larga del driver.                                           */
        HAL_I2C_Mem_Write(ssd_i2c, SSD1306_ADDR, 0x40, 1,
                          &ssd_buffer[SSD1306_WIDTH * page],
                          SSD1306_WIDTH, 200);
    }
}

/* ═══════════════════════ Primitivas de dibujo ═════════════════════════════ */

void SSD1306_Fill(uint8_t color)
{
    memset(ssd_buffer, (color == 0) ? 0x00 : 0xFF, sizeof(ssd_buffer));
}

/*
 * SSD1306_DrawPixel
 * Traduce la coordenada (x,y) a byte y bit del framebuffer:
 *   byte = x + (y/8) * 128   → columna x de la página y/8
 *   bit  = y % 8             → fila dentro de esa página
 * El chequeo de límites descarta silenciosamente lo que quede fuera de
 * pantalla, lo que permite que las funciones de dibujo (círculos, cruces)
 * no tengan que validar sus propios bordes.
 */
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        return;
    }

    if (color)
    {
        ssd_buffer[x + (y / 8) * SSD1306_WIDTH] |=  (uint8_t)(1u << (y % 8));
    }
    else
    {
        ssd_buffer[x + (y / 8) * SSD1306_WIDTH] &= (uint8_t)~(1u << (y % 8));
    }
}

/*
 * SSD1306_WriteString
 * Cada glifo ocupa 5 columnas de la fuente más 1 de separación, de ahí el
 * avance de 6 px por carácter. Los caracteres fuera del rango imprimible
 * se sustituyen por un espacio.
 */
void SSD1306_WriteString(uint8_t x, uint8_t y, const char *str)
{
    while (*str)
    {
        uint8_t idx = (*str >= 32 && *str <= 126) ? (uint8_t)(*str - 32) : 0;

        for (uint8_t col = 0; col < 5; col++)
        {
            uint8_t bits = Font5x7[idx][col];

            for (uint8_t row = 0; row < 8; row++)
            {
                if ((bits >> row) & 1u)
                {
                    SSD1306_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), 1);
                }
            }
        }
        x += 6;
        str++;
    }
}

/*
 * SSD1306_WriteCharScaled
 * Amplía el glifo dibujando cada píxel de la fuente como un bloque cuadrado
 * de scale x scale píxeles. No suaviza los bordes (es escalado por
 * replicación), pero para una sola letra grande el resultado es perfectamente
 * legible y no requiere una segunda fuente en Flash.
 */
void SSD1306_WriteCharScaled(uint8_t x, uint8_t y, char c, uint8_t scale)
{
    uint8_t idx = (c >= 32 && c <= 126) ? (uint8_t)(c - 32) : 0;

    if (scale == 0)
    {
        scale = 1;
    }

    for (uint8_t col = 0; col < 5; col++)
    {
        uint8_t bits = Font5x7[idx][col];

        for (uint8_t row = 0; row < 8; row++)
        {
            if ((bits >> row) & 1u)
            {
                /* bloque de scale x scale que reemplaza a un píxel */
                for (uint8_t dx = 0; dx < scale; dx++)
                {
                    for (uint8_t dy = 0; dy < scale; dy++)
                    {
                        SSD1306_DrawPixel((uint8_t)(x + col * scale + dx),
                                          (uint8_t)(y + row * scale + dy), 1);
                    }
                }
            }
        }
    }
}

/*
 * SSD1306_DrawEmptyRect
 * w y h son un TAMAÑO, no coordenadas finales; los bordes derecho e
 * inferior se calculan aquí.
 */
void SSD1306_DrawEmptyRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    uint8_t x_end = (uint8_t)(x + w);
    uint8_t y_end = (uint8_t)(y + h);

    for (uint8_t i = x; i <= x_end; i++)
    {
        SSD1306_DrawPixel(i, y,     1);   /* borde superior                 */
        SSD1306_DrawPixel(i, y_end, 1);   /* borde inferior                 */
    }
    for (uint8_t i = y; i <= y_end; i++)
    {
        SSD1306_DrawPixel(x,     i, 1);   /* borde izquierdo                */
        SSD1306_DrawPixel(x_end, i, 1);   /* borde derecho                  */
    }
}

/*
 * SSD1306_DrawFilledCircle
 * Recorre el cuadrado que envuelve al círculo y enciende los píxeles cuya
 * distancia al centro cabe en el radio (dx^2 + dy^2 <= r^2). Se usa
 * aritmética con signo para el recorrido y se deja que DrawPixel recorte
 * lo que se salga de pantalla.
 */
void SSD1306_DrawFilledCircle(uint8_t xc, uint8_t yc, uint8_t r)
{
    int16_t radius = (int16_t)r;

    for (int16_t dy = -radius; dy <= radius; dy++)
    {
        for (int16_t dx = -radius; dx <= radius; dx++)
        {
            if ((dx * dx + dy * dy) <= (radius * radius))
            {
                int16_t px = (int16_t)xc + dx;
                int16_t py = (int16_t)yc + dy;

                if (px >= 0 && py >= 0)
                {
                    SSD1306_DrawPixel((uint8_t)px, (uint8_t)py, 1);
                }
            }
        }
    }
}

void SSD1306_DrawHLine(uint8_t x0, uint8_t x1, uint8_t y)
{
    for (uint8_t x = x0; x <= x1; x++)
    {
        SSD1306_DrawPixel(x, y, 1);
    }
}

/*
 * SSD1306_BusRecover — desatasca el bus I2C antes de configurar el periférico.
 *
 * Si el microcontrolador se reinicia (reset, regrabado o cuelgue) en mitad
 * de una transferencia, el controlador de la pantalla se queda esperando
 * los pulsos de reloj que le faltan para terminar el byte, y mientras tanto
 * MANTIENE LA LÍNEA SDA EN BAJO. El bus nunca se ve libre y toda
 * transferencia posterior falla por timeout. Resetear el micro no lo
 * arregla: el módulo tiene su propia alimentación y conserva su estado.
 *
 * La solución es tomar los pines como GPIO en drenador abierto y generar a
 * mano hasta 9 pulsos de reloj (los que puede necesitar el esclavo para
 * completar un byte más el bit de reconocimiento). En cuanto termina,
 * libera SDA. Después se emite una condición de STOP y se devuelven los
 * pines al periférico.
 *
 * Debe llamarse ANTES de configurar el I2C: mientras los pines pertenecen
 * al periférico no se pueden manipular manualmente.
 *
 *   scl_port / scl_pin : puerto y pin de SCL (PB8 en este montaje)
 *   sda_port / sda_pin : puerto y pin de SDA (PB9 en este montaje)
 */
void SSD1306_BusRecover(GPIO_TypeDef *scl_port, uint16_t scl_pin,
                        GPIO_TypeDef *sda_port, uint16_t sda_pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Ambos pines como GPIO en drenador abierto, igual que los usa el bus:
     * solo se tira a nivel bajo y las pull-ups del módulo suben la línea. */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = scl_pin;
    HAL_GPIO_Init(scl_port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = sda_pin;
    HAL_GPIO_Init(sda_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    for (uint8_t i = 0; i < 9u; i++)
    {
        if (HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_SET)
        {
            break;                  /* SDA ya está libre: el bus está sano */
        }
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Condición de STOP: SDA sube mientras SCL está en alto */
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}
