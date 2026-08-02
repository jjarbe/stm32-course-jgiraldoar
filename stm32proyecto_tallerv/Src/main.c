/*
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Traductor de texto a Braille — versión integrada
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 ******************************************************************************
 *
 * DESCRIPCIÓN GENERAL
 * El sistema recibe una palabra por el puerto serial y la recorre letra por
 * letra a un ritmo configurable. En cada letra, la pantalla OLED muestra la
 * letra en grande junto a su traducción en una celda Braille de 3x2 puntos y
 * el tiempo de espera vigente. Un encoder rotativo ajusta ese tiempo entre 1,
 * 2 y 3 segundos, y el sistema confirma la selección en voz alta a través del
 * amplificador. El pulsador del encoder pausa y reanuda el recorrido.
 *
 * La máscara de 6 bits que hoy dibuja la celda en la pantalla será mañana la
 * que decida qué punto sube y cuál baja en los seis servomotores: una sola
 * fuente de verdad para las dos representaciones.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  TIM10  → Blinky en PH1 cada 250 ms y base de tiempo del
 *           secuenciador de letras                               [CON IRQ]
 *  TIM1   → Encoder rotativo, DT = PA8 / CLK = PA9 (AF1)         [SIN IRQ]
 *  GPIO   → PA10 pulsador del encoder (entrada con pull-up)
 *           PA11 pin SD del amplificador (habilitación / silencio)
 *           PH1  LED de estado
 *  I2S2   → Full-duplex, 16 kHz, 32 bit:
 *             CK  = PB13 (AF5)  → BCLK de micrófono y amplificador
 *             WS  = PB12 (AF5)  → LRC  de micrófono y amplificador
 *             SD  = PB15 (AF5)  → DIN del amplificador (salida)
 *             ext = PB14 (AF6)  → SD del micrófono (entrada)
 *  DMA1_S4→ Transmisión hacia el amplificador (SPI2_TX, canal 0) [CON IRQ]
 *  DMA1_S3→ Recepción desde el micrófono (I2S2ext_RX, canal 3)   [CON IRQ]
 *  I2C1   → OLED SSD1306, SCL = PB8, SDA = PB9 (AF4), 100 kHz    [SIN IRQ]
 *  USART2 → 115200-8N1 por el VCP del ST-LINK,
 *           TX = PA2 (AF7) por sondeo,
 *           RX = PA3 (AF7) por interrupción                      [CON IRQ]
 *
 * ─── Pines reservados para los servomotores (etapa siguiente) ───────────────
 *  PA6, PA7, PB0, PB1 → TIM3 canales 1..4
 *  PB6, PB7           → TIM4 canales 1..2
 *
 * ─── Uso ────────────────────────────────────────────────────────────────────
 *  Escribir la palabra en el terminal y pulsar Enter: el recorrido arranca en
 *  la primera letra y se repite en bucle al terminar.
 *  Girar el encoder  → cambia el tiempo (3 clics por opción) y lo anuncia
 *  Pulsar el encoder → pausa / reanuda el recorrido
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include "braille.h"
#include "clips_voz.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

/* ── Texto ── */
#define TEXT_MAX_LEN          32u   /* capacidad del buffer de la palabra   */

/* ── Base de tiempo ── */
#define TICKS_PER_SECOND       4u   /* el tick de TIM10 es de 250 ms        */

/* ── Encoder ──
 * ENC_COUNTS_PER_CLICK es propiedad del HARDWARE: el timer en cuadratura x4
 * cuenta cuatro flancos por clic mecánico. ENC_CLICKS_PER_STEP es decisión
 * de uso: con solo tres opciones, exigir tres clics por cambio da un tacto
 * mucho más controlable que uno.                                           */
#define ENC_COUNTS_PER_CLICK   4
#define ENC_CLICKS_PER_STEP    3
#define ENC_COUNTS_PER_STEP   (ENC_COUNTS_PER_CLICK * ENC_CLICKS_PER_STEP)

/* Tiempo que el mando debe quedar quieto antes de anunciar por voz. Se mide
 * con HAL_GetTick (1 ms de resolución) y no con el tick de 250 ms, que sería
 * demasiado grueso para que la respuesta se sienta natural.                */
#define VOZ_ESPERA_MS        400u

/* ── Opciones de tiempo de espera entre letras, en segundos ── */
#define OPCIONES               3u
static const uint8_t tiempos[OPCIONES] = { 1u, 2u, 3u };

/* ── Audio ──
 * Formato de 32 bits: cada trama ocupa cuatro medias palabras en memoria,
 * [L_alto][L_bajo][R_alto][R_bajo]. 256 tramas a 16 kHz son 16 ms.         */
#define BUF_FRAMES           256u
#define BUF_HALFWORDS       (BUF_FRAMES * 4u)
#define BUF_SAMPLES         (BUF_FRAMES * 2u)

/* ── Geometría de la interfaz (pantalla de 128x64) ──
 * El panel es bicolor: las 16 primeras filas son amarillas y el resto azul,
 * con una franja oscura de un par de filas en la transición. Todo el
 * contenido gráfico empieza por debajo de y = 18 para no invadir el
 * amarillo; en particular, la primera fila de la celda tiene que estar lo
 * bastante abajo como para que el círculo de radio 5 quepa entero en la
 * zona azul (24 - 5 = 19).
 *
 * Disposición horizontal:  letra → flecha → celda Braille → tiempo
 */
#define UI_SEP_Y            10     /* línea separadora, en zona amarilla    */

#define UI_LETTER_X          4     /* letra grande, pegada a la izquierda   */
#define UI_LETTER_Y         22
#define UI_LETTER_SCALE      4     /* 5x7 escalado x4 = 20x32 px            */

#define UI_ARROW_X0         30     /* flecha entre la letra y la celda      */
#define UI_ARROW_X1         50
#define UI_ARROW_Y          36

#define UI_CELL_COL_L       62     /* celda desplazada a la izquierda para  */
#define UI_CELL_COL_R       84     /* dejar sitio al tiempo                 */
#define UI_CELL_ROW_1       24
#define UI_CELL_ROW_2       36
#define UI_CELL_ROW_3       48
#define UI_DOT_R             5

#define UI_TIME_X           99     /* tiempo, a la derecha de la celda      */
#define UI_TIME_Y           30
#define UI_STATE_X           0     /* aviso de pausa, esquina inferior      */
#define UI_STATE_Y          56

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,        /* sin eventos pendientes                          */
    FSM_PROC_UART,       /* llegó un carácter por el puerto serial          */
    FSM_PROC_ENCODER,    /* el mando giró un paso completo                  */
    FSM_PROC_BOTON,      /* se pulsó el mando → pausa / reanuda             */
    FSM_NEXT_LETTER,     /* venció el tiempo → avanzar de letra             */
    FSM_ANUNCIAR,        /* el mando se detuvo → anunciar el tiempo         */
    FSM_REFRESH_OLED     /* redibujar la pantalla                           */
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */
/* Globales porque stm32f4xx_it.c los referencia con 'extern' para pasarlos
 * a los HAL_xxx_IRQHandler() dentro de cada ISR.                            */

TIM_HandleTypeDef  htim1;         /* encoder                    */
TIM_HandleTypeDef  htim10;        /* blinky y base de tiempo    */
I2S_HandleTypeDef  hi2s2;         /* audio full-duplex          */
DMA_HandleTypeDef  hdma_i2s2_tx;  /* hacia el amplificador      */
DMA_HandleTypeDef  hdma_i2s2_rx;  /* desde el micrófono         */
I2C_HandleTypeDef  hi2c1;         /* pantalla OLED              */
UART_HandleTypeDef huart2;        /* puerto serial              */

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

/* Banderas de evento: las escriben las ISR/callbacks y las consume la FSM.
 * 'volatile' es obligatorio porque cambian fuera del flujo normal y sin él
 * el compilador podría optimizar las lecturas del lazo.                    */
static volatile uint8_t flag_rx      = 0;   /* carácter recibido            */
static volatile uint8_t flag_next    = 0;   /* venció el tiempo de espera   */
static volatile uint8_t flag_refresh = 0;   /* hay que redibujar            */
static volatile uint8_t rx_byte      = 0;   /* buffer de recepción (1 byte) */

/* ── Texto en curso ── */
static char    text_buf[TEXT_MAX_LEN + 1] = {0};
static uint8_t text_len = 0;      /* letras válidas                         */
static uint8_t text_idx = 0;      /* letra que se muestra ahora             */

/* Buffer de ensamblado de la recepción: los caracteres se acumulan aquí
 * hasta el fin de línea, para poder seguir escribiendo mientras la palabra
 * anterior sigue corriendo.                                                */
static char    rx_line[TEXT_MAX_LEN + 1] = {0};
static uint8_t rx_len = 0;
static uint8_t rx_utf8_pending = 0;   /* se recibió el 0xC3 de la 'ñ'       */

/* ── Estado del secuenciador ── */
static uint8_t opcion_idx = 0;    /* índice dentro de tiempos[]             */
static uint8_t paused     = 0;    /* 1 = recorrido detenido                 */
static uint8_t running    = 0;    /* 1 = hay palabra en curso               */

/* ── Encoder y pulsador ── */
static uint16_t enc_last_cnt = 0;
static uint8_t  sw_prev      = 1;

/* ── Anuncio por voz ── */
static uint8_t  voz_pendiente   = 0;
static uint32_t t_ultimo_cambio = 0;

/* ── Audio ── */
static uint16_t tx_buf[BUF_HALFWORDS];
static uint16_t rx_buf[BUF_HALFWORDS];

/* Reproductor de clips: el callback lee de aquí y el lazo principal solo
 * arranca o detiene.                                                       */
static const int16_t * volatile clip_datos  = 0;
static volatile uint32_t        clip_largo  = 0;
static volatile uint32_t        clip_pos    = 0;
static volatile uint8_t         clip_activo = 0;

static fsm_state_t fsm_state = FSM_IDLE;

/* ═══════════════════════ Prototipos ═══════════════════════════════════════ */

static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim10_Init(void);
static void tim1_encoder_Init(void);
static void dma_Init(void);
static void i2s2_Init(void);
static void i2c1_Init(void);
static void usart2_Init(void);

static void fsm_Run(void);
static void trap_Error(void);
static void encoder_Poll(void);
static void boton_Poll(void);
static void text_Start(void);
static void clip_Reproducir(const int16_t *datos, uint32_t largo);
static void anuncio_Programar(void);
static void servos_Aplicar(uint8_t mask);
static void ui_DrawArrow(uint8_t x0, uint8_t x1, uint8_t y);
static void ui_Draw(void);
static void ui_DrawBrailleCell(uint8_t mask);
static void print(const char *s);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();             /* SysTick a 1 ms, caché de Flash, NVIC         */
    SystemClock_Config();   /* PLL a 96 MHz y PLLI2S a 76.8 MHz             */

    gpio_Init();
    usart2_Init();          /* primero el serial, para poder informar       */
    print("\r\n=== TRADUCTOR DE TEXTO A BRAILLE ===\r\n");

    tim10_Init();           print("  Base de tiempo OK\r\n");
    tim1_encoder_Init();    print("  Encoder OK\r\n");
    dma_Init();             print("  DMA OK\r\n");
    i2s2_Init();            print("  Audio OK\r\n");
    i2c1_Init();            print("  I2C OK\r\n");

    SSD1306_SetI2C(&hi2c1);
    SSD1306_Init();
    print("  Pantalla OK\r\n\r\n");

    /* Arrancar el audio: a partir de aquí el DMA mantiene el flujo en las
     * dos direcciones sin intervención del procesador.                     */
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_buf, rx_buf, BUF_SAMPLES)
        != HAL_OK)
    {
        print("  [ERROR] No arranco el audio\r\n");
        trap_Error();
    }

    /* Pantalla de bienvenida */
    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0,  "TRADUCTOR BRAILLE");
    SSD1306_DrawHLine(0, 127, UI_SEP_Y);
    SSD1306_WriteString(0, 20, "Escriba una palabra");
    SSD1306_WriteString(0, 30, "y pulse Enter.");
    SSD1306_WriteString(0, 44, "Encoder: tiempo");
    SSD1306_WriteString(0, 54, "Boton: pausa");
    SSD1306_UpdateScreen();

    {
    	const char hello[] =
    			"\r\n== Traductor de texto a Braille ==\r\n"
    			"Escriba una palabra y pulse Enter.\r\n"
    			"Encoder: tiempo entre letras (1/2/3 s). Boton: pausa.\r\n\r\n> ";
    	HAL_UART_Transmit(&huart2, (uint8_t *)hello,
    			(uint16_t)strlen(hello), 200);
    }

    /* Armar la recepción del primer carácter por interrupción */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);

    /* Lazo de aplicación: no bloquea. Los periféricos avisan por banderas;
     * aquí solo se sondean encoder y pulsador y se despachan los eventos.  */
    while (1)
    {
        encoder_Poll();
        boton_Poll();
        fsm_Run();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *                        FUNCIONES DE CONFIGURACIÓN
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════ Reloj del sistema y reloj de audio ═══════════════════ */
/*
 * El sistema necesita DOS lazos de enganche de fase: el PLL principal para
 * el reloj del procesador y el PLLI2S para el audio.
 *
 *   VCO_in = HSI / PLLM = 16 MHz / 8 = 2 MHz     (recomendado: 1-2 MHz)
 *   PLL:     VCO = 2 x 96  = 192 MHz  →  SYSCLK = 192/2 = 96 MHz
 *   PLLI2S:  VCO = 2 x 192 = 384 MHz  →  I2SCLK = 384/5 = 76.8 MHz
 *
 * La elección de 76.8 MHz no es arbitraria: es el valor que hace que los
 * divisores enteros del periférico I2S den 16 000,00 Hz exactos. Con formato
 * de 32 bits el periférico divide entre 64 x (2 x I2SDIV + ODD) = 64 x 75.
 *
 * PLLI2SM: en el STM32F411 el PLLI2S tiene su PROPIO divisor de entrada,
 * independiente del PLLM del PLL principal. Omitirlo lo deja en cero y el
 * PLL de audio nunca engancha; el fallo se manifiesta más tarde, al
 * inicializar el I2S, lo que despista bastante.
 *
 * Divisores de bus a 96 MHz (respetando los máximos del F411):
 *   AHB = 96 MHz    APB1 = 48 MHz (máx. 50)    APB2 = 96 MHz (máx. 100)
 * Por encima de 90 MHz la Flash necesita 3 estados de espera y el regulador
 * debe estar en Scale 1.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;    /* también entra al I2S   */
    RCC_OscInitStruct.PLL.PLLN            = 96;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;    /* rama USB/SDIO, sin uso */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { trap_Error(); }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        trap_Error();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInit.PLLI2S.PLLI2SM       = 8;    /* divisor propio del F411   */
    PeriphClkInit.PLLI2S.PLLI2SN       = 192;
    PeriphClkInit.PLLI2S.PLLI2SR       = 5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════ GPIO: LED, pulsador y control del audio ══════════════ */
/*
 * PH1  = LED integrado de la board. PH0/PH1 son los pines del oscilador
 *        externo; como el sistema corre con HSI + PLL y no hay cristal,
 *        quedan libres como GPIO.
 * PA10 = pulsador del encoder. Entrada con pull-up: el contacto lo lleva a
 *        GND al pulsar, así que en reposo se lee 1 y pulsado 0.
 * PA11 = pin SD del amplificador. No es una entrada lógica corriente: el
 *        chip mide el VOLTAJE y con él decide entre apagado, canal derecho,
 *        mezcla o canal izquierdo. En bajo lo apaga; en alto desde un GPIO
 *        de 3.3 V lo pone en canal izquierdo, estado determinista que no
 *        depende del divisor resistivo del módulo.
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_1;          /* PH1: LED de estado      */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;           /* PA11: SD del ampli      */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* Silenciado hasta que haya algo que reproducir: evita el siseo de
     * reposo y el chasquido del arranque.                                  */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = GPIO_PIN_10;          /* PA10: pulsador          */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ═══════════════════════ TIM10: tick de 250 ms (IRQ) ══════════════════════ */
/*
 * Cadena de reloj:  PLL (96 MHz) → APB2 (96 MHz) → reloj de TIM10 (96 MHz)
 *
 * PSC = 9599 → tick = 96 MHz / 9600 = 10 kHz  (100 us por cuenta)
 * ARR = 2499 → update = 2500 x 100 us = 250 ms
 *
 * A 96 MHz no se puede usar un prescaler que dé 1 ms por cuenta (harían
 * falta 95 999, que no cabe en los 16 bits del registro), así que se baja la
 * resolución del tick y se sube el auto-recarga.
 *
 * Este único timer cumple dos funciones: conmutar el LED de estado y contar
 * el tiempo entre letras. Así quedan libres TIM3 y TIM4 para el PWM de los
 * seis servomotores.
 *
 * TIM10 comparte el vector TIM1_UP_TIM10_IRQn con el update de TIM1; no hay
 * conflicto porque TIM1 (encoder) no tiene interrupciones habilitadas.
 */
static void tim10_Init(void)
{
    __HAL_RCC_TIM10_CLK_ENABLE();

    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 9599;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 2499;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) { trap_Error(); }

    HAL_TIM_Base_Start_IT(&htim10);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* ═══════════════════════ TIM1: encoder rotativo (sin IRQ) ═════════════════ */
/*
 * El encoder se decodifica por HARDWARE en modo TI12 (cuadratura x4): el
 * contador del timer sube o baja solo siguiendo los flancos de ambos
 * canales, con filtro digital que absorbe los rebotes mecánicos. La posición
 * se lee por sondeo del registro CNT.
 *
 * Esa es la razón de que el encoder no necesite interrupciones: no hay que
 * atrapar los flancos con el procesador porque el periférico ya los cuenta.
 * Un esquema con interrupciones externas en los dos pines gastaría tiempo de
 * CPU en cada flanco y obligaría a filtrar rebotes por software.
 */
static void tim1_encoder_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct = {0};
    TIM_Encoder_InitTypeDef sEncoder        = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA8/PA9 en función alternada AF1: dejan de ser GPIO y pasan a ser las
     * entradas de captura CH1/CH2 del timer                                */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;   /* señales en reposo altas   */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xFFFF;     /* contador libre de 16 bits */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoder.EncoderMode  = TIM_ENCODERMODE_TI12;
    sEncoder.IC1Polarity  = TIM_ICPOLARITY_FALLING; /* fija el sentido según
                                                       el cableado DT/CLK;
                                                       si queda invertido,
                                                       usar RISING          */
    sEncoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoder.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoder.IC1Filter    = 0x0F;              /* filtro antirrebote máximo */
    sEncoder.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sEncoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoder.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoder.IC2Filter    = 0x0F;
    if (HAL_TIM_Encoder_Init(&htim1, &sEncoder) != HAL_OK) { trap_Error(); }

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   /* sin _IT: sin IRQ   */
}

/* ═══════════════ DMA: un stream por dirección, ambos circulares ═══════════ */
/*
 * A 16 kHz el periférico pide una muestra cada 62 microsegundos. Atender eso
 * por interrupción consumiría el procesador que hace falta para la pantalla,
 * el encoder y, mañana, los servos. El DMA traslada las muestras por
 * hardware.
 *
 * El full-duplex necesita DOS streams, uno por bloque del periférico, y cada
 * uno tiene su correspondencia fija en el mapa del hardware:
 *
 *   Transmisión (bloque principal SPI2)   → Stream 4, canal 0
 *   Recepción   (bloque auxiliar I2S2ext) → Stream 3, canal 3
 *
 * El canal de recepción es el 3 y no el 0 porque el canal 0 del stream 3
 * corresponde a la recepción del SPI2 principal, que aquí no se usa: ese
 * bloque está dedicado a transmitir. El bloque extendido tiene su propia
 * línea de petición.
 *
 * MODO CIRCULAR: al llegar al final del buffer el DMA vuelve solo al
 * principio y avisa dos veces por vuelta, a la mitad y al final. Eso permite
 * rellenar la mitad que ya se envió mientras la otra sigue en uso, sin
 * competir nunca por la misma memoria.
 */
static void dma_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_i2s2_tx.Instance                 = DMA1_Stream4;
    hdma_i2s2_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_i2s2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2s2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2s2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2s2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2s2_tx) != HAL_OK) { trap_Error(); }
    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_i2s2_tx);

    hdma_i2s2_rx.Instance                 = DMA1_Stream3;
    hdma_i2s2_rx.Init.Channel             = DMA_CHANNEL_3;   /* I2S2ext_RX  */
    hdma_i2s2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2s2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2s2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2s2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s2_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2s2_rx) != HAL_OK) { trap_Error(); }
    __HAL_LINKDMA(&hi2s2, hdmarx, hdma_i2s2_rx);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

/* ═══════════════════════ I2S2 en full-duplex ══════════════════════════════ */
/*
 * El periférico por sí solo puede transmitir O recibir, no ambas cosas. Para
 * las dos direcciones el chip incorpora un bloque auxiliar, I2S2ext, que se
 * engancha al MISMO generador de reloj. Mode = MASTER_TX indica qué hace el
 * bloque principal; al activar FullDuplexMode el auxiliar toma la dirección
 * contraria.
 *
 * El STM32 debe ser maestro obligatoriamente: ni el amplificador ni el
 * micrófono pueden generar los relojes, ambos son siempre esclavos.
 *
 * Al compartir relojes, las dos direcciones quedan obligadas al mismo
 * formato. Como el micrófono trabaja en ranuras de 32 bits, todo el sistema
 * va a 32 bits.
 *
 * OJO CON LAS FUNCIONES ALTERNADAS: PB12, PB13 y PB15 pertenecen al SPI2
 * (AF5), pero PB14 pertenece al bloque extendido y usa AF6. Configurar los
 * cuatro con la misma AF es un error silencioso: transmitiría bien y la
 * recepción llegaría siempre en cero.
 */
static void i2s2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2S2ext;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_SPI2_CLK_ENABLE();

    hi2s2.Instance            = SPI2;
    hi2s2.Init.Mode           = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat     = I2S_DATAFORMAT_32B;
    hi2s2.Init.MCLKOutput     = I2S_MCLKOUTPUT_DISABLE;  /* no lo necesitan */
    hi2s2.Init.AudioFreq      = I2S_AUDIOFREQ_16K;
    hi2s2.Init.CPOL           = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════════ I2C1: bus de la pantalla ═════════════════════════ */
/*
 * SCL = PB8, SDA = PB9 en AF4 y en drenador abierto, como exige el bus I2C:
 * los dispositivos solo tiran la línea a nivel bajo y las resistencias de
 * pull-up del módulo la suben.
 *
 * Velocidad de 100 kHz en lugar de 400 kHz. Con todo el cableado del montaje
 * (audio, parlante, encoder) la capacidad parásita del bus crece y a 400 kHz
 * los flancos se redondean lo suficiente como para que las transferencias
 * empiecen a fallar. A 100 kHz hay cuatro veces más tiempo de subida y el
 * volcado de la pantalla completa sigue tardando solo unos 100 ms.
 *
 * Sin interrupciones: el SSD1306 se maneja por sondeo.
 */
static void i2c1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Desatascar el bus antes de tomar los pines: recupera el sistema de un
     * reset ocurrido en mitad de una transferencia, situación en la que el
     * módulo deja la línea SDA bloqueada en nivel bajo.                    */
    SSD1306_BusRecover(GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════════ USART2: puerto serial por el VCP ═════════════════ */
/*
 * PA2/PA3 (AF7) están cableados internamente al Virtual COM Port del
 * ST-LINK, así que el serial viaja por el mismo cable USB de programación,
 * sin cableado adicional.
 *
 * Transmisión por sondeo; recepción de un carácter por interrupción, que es
 * lo que estrictamente lo requiere: los caracteres llegan en cualquier
 * momento y no se puede bloquear el lazo esperándolos.
 *
 * Se usa el driver ASÍNCRONO del HAL (UART_HandleTypeDef, HAL_UART_Init,
 * HAL_UART_Receive_IT). El driver síncrono requeriría una señal de reloj
 * adicional; el nombre "USART2" aparece solo porque así se llama la
 * instancia física del periférico en el F411.
 */
static void usart2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) { trap_Error(); }

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *                    FUNCIONES DE CONTROL (APLICACIÓN / FSM)
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * trap_Error — trampa de error de configuración.
 * Si una función HAL de inicialización devuelve algo distinto de HAL_OK, el
 * programa queda atrapado aquí. Con el debugger pausado, el call stack
 * indica exactamente qué configuración falló.
 */
static void trap_Error(void)
{
    while (1) { __NOP(); }
}

static void print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 500);
}

/*
 * encoder_Poll — sondeo del contador del encoder.
 *
 * Un "paso" son ENC_COUNTS_PER_STEP cuentas, es decir tres clics mecánicos.
 * Como la división entera descarta el resto y solo se consumen los múltiplos
 * completos, los clics sobrantes quedan acumulados en el contador del timer
 * y se suman al siguiente giro: no se pierde movimiento, solo se exige más
 * recorrido por cambio.
 *
 * La resta se hace en aritmética modular de 16 bits, lo que maneja
 * correctamente el desborde del contador en ambos sentidos.
 */
static void encoder_Poll(void)
{
    uint16_t cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t  diff  = (int16_t)(cnt - enc_last_cnt);
    int16_t  pasos = (int16_t)(diff / ENC_COUNTS_PER_STEP);

    if (pasos != 0)
    {
        enc_last_cnt = (uint16_t)(enc_last_cnt +
                                  (uint16_t)(pasos * ENC_COUNTS_PER_STEP));

        /* Recorrido cíclico entre las tres opciones de tiempo */
        if (pasos > 0)
        {
            opcion_idx = (uint8_t)((opcion_idx + 1u) % OPCIONES);
        }
        else
        {
            opcion_idx = (uint8_t)((opcion_idx + OPCIONES - 1u) % OPCIONES);
        }

        if (fsm_state == FSM_IDLE) { fsm_state = FSM_PROC_ENCODER; }
    }
}

/*
 * boton_Poll — pulsador con detección de flanco.
 * Solo interesa la TRANSICIÓN de suelto a pulsado: sin esta comprobación,
 * mantener el botón generaría un evento en cada pasada del lazo.
 */
static void boton_Poll(void)
{
    uint8_t sw = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)
                 ? 0u : 1u;

    if (sw_prev == 1u && sw == 0u)
    {
        if (fsm_state == FSM_IDLE) { fsm_state = FSM_PROC_BOTON; }
    }
    sw_prev = sw;
}

/*
 * text_Start — carga la línea recibida como palabra en curso.
 * Se llama al llegar el fin de línea: copia el buffer de ensamblado al de
 * trabajo, reinicia el índice y arranca el recorrido.
 */
static void text_Start(void)
{
    if (rx_len == 0u) { return; }        /* Enter sin texto: no hacer nada  */

    memcpy(text_buf, rx_line, rx_len);
    text_buf[rx_len] = '\0';
    text_len = rx_len;
    text_idx = 0;
    running  = 1;
    paused   = 0;

    rx_len = 0;                     /* listo para la siguiente palabra      */
    flag_refresh = 1;
}

/*
 * clip_Reproducir — arranca un clip de voz, cancelando el que estuviera
 * sonando.
 *
 * El orden de las asignaciones importa: primero se baja el interruptor para
 * que el callback deje de leer, después se cambian puntero y longitud, y
 * solo al final se vuelve a subir. Así el callback nunca puede encontrarse
 * con un puntero nuevo y una longitud vieja, que le haría leer fuera del
 * arreglo.
 */
static void clip_Reproducir(const int16_t *datos, uint32_t largo)
{
    clip_activo = 0;
    clip_datos  = datos;
    clip_largo  = largo;
    clip_pos    = 0;
    clip_activo = 1;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   /* ampli ON      */
}

/*
 * anuncio_Programar — arma la espera antes de hablar.
 * Se llama en cada cambio de opción. Al reescribir la marca de tiempo, cada
 * movimiento nuevo reinicia la cuenta, de modo que el anuncio solo sale
 * cuando el mando lleva VOZ_ESPERA_MS quieto. Es lo que evita encadenar
 * "uno, dos, tres" al girar rápido de un extremo al otro.
 */
static void anuncio_Programar(void)
{
    voz_pendiente   = 1;
    t_ultimo_cambio = HAL_GetTick();
}

/*
 * servos_Aplicar — punto de enganche para los seis servomotores.
 *
 * Recibe la misma máscara de 6 bits que dibuja la celda en pantalla, de modo
 * que motores y pantalla siempre muestran la misma letra: una sola fuente de
 * verdad. Cada bit corresponde a un punto del signo generador y decidirá si
 * ese servo sube o baja.
 *
 * Pendiente de implementar en la etapa de los servos.
 */
static void servos_Aplicar(uint8_t mask)
{
    (void)mask;
}

/*
 * ui_DrawBrailleCell — dibuja la celda de 6 puntos a partir de la máscara.
 *
 * Los puntos 1-2-3 forman la columna izquierda de arriba abajo y los 4-5-6 la
 * derecha, según la numeración del signo generador. Un punto activo se dibuja
 * como círculo relleno y uno inactivo como un recuadro pequeno, para que la
 * retícula siga siendo visible: así se aprecia qué puntos existen y cuáles
 * están levantados, igual que en las tablas del alfabeto Braille.
 */
static void ui_DrawBrailleCell(uint8_t mask)
{
    /* Tabla local: cada punto con su máscara y su posición. Recorrerla en un
     * bucle evita repetir seis veces la misma comprobación.                */
    static const struct { uint8_t bit; uint8_t x; uint8_t y; } dots[6] =
    {
        { BRAILLE_DOT_1, UI_CELL_COL_L, UI_CELL_ROW_1 },
        { BRAILLE_DOT_2, UI_CELL_COL_L, UI_CELL_ROW_2 },
        { BRAILLE_DOT_3, UI_CELL_COL_L, UI_CELL_ROW_3 },
        { BRAILLE_DOT_4, UI_CELL_COL_R, UI_CELL_ROW_1 },
        { BRAILLE_DOT_5, UI_CELL_COL_R, UI_CELL_ROW_2 },
        { BRAILLE_DOT_6, UI_CELL_COL_R, UI_CELL_ROW_3 },
    };

    for (uint8_t i = 0; i < 6u; i++)
    {
        if (mask & dots[i].bit)
        {
            SSD1306_DrawFilledCircle(dots[i].x, dots[i].y, UI_DOT_R);
        }
        else
        {
        	/* Punto inactivo: recuadro pequeño centrado en la misma
        	 * posición, igual que el indicador de opción no seleccionada.
        	 * Deja ver la retícula completa de la celda y se distingue sin
        	 * ambigüedad de un punto levantado.                           */
        	SSD1306_DrawEmptyRect((uint8_t)(dots[i].x - 2u),
        			(uint8_t)(dots[i].y - 2u), 4u, 4u);
        }
    }
}

/*
 * ui_DrawArrow — flecha horizontal que va de x0 a x1 a la altura y.
 *
 * Señala visualmente la relación entre la letra y su traducción: el asta se
 * dibuja con dos filas de píxeles para que se vea bien en un panel pequeño,
 * y la punta son dos diagonales que retroceden desde el extremo.
 */
static void ui_DrawArrow(uint8_t x0, uint8_t x1, uint8_t y)
{
    /* Asta de dos píxeles de grosor */
    SSD1306_DrawHLine(x0, x1, y);
    SSD1306_DrawHLine(x0, x1, (uint8_t)(y + 1u));

    /* Punta: diagonales que se abren hacia atrás desde el extremo */
    for (uint8_t i = 1u; i <= 4u; i++)
    {
        SSD1306_DrawPixel((uint8_t)(x1 - i), (uint8_t)(y - i), 1);
        SSD1306_DrawPixel((uint8_t)(x1 - i), (uint8_t)(y + 1u + i), 1);
    }
}

/*
 * ui_Draw — redibuja la pantalla completa.
 *
 * Se recompone toda la escena en cada refresco en lugar de actualizar zonas
 * sueltas: con 1024 bytes de framebuffer el costo es irrelevante y a cambio
 * se elimina por completo la posibilidad de restos de un dibujo anterior.
 *
 * El refresco solo ocurre cuando algo cambia (letra nueva, giro del encoder,
 * pausa, palabra nueva) y no de forma periódica, porque volcar la pantalla
 * por I2C ocupa el lazo unos 100 ms y no tiene sentido repetirlo sin motivo.
 */
static void ui_Draw(void)
{
	char    linea[48];
	char    letra;
	uint8_t mask;

	SSD1306_Fill(0);

	/* ── Encabezado (zona amarilla): palabra y posición ──                 */
	if (running)
	{
		snprintf(linea, sizeof(linea), "%s (%u/%u)", text_buf,
				(unsigned)(text_idx + 1u), (unsigned)text_len);
	}
	else
	{
		snprintf(linea, sizeof(linea), "Sin palabra");
	}
	SSD1306_WriteString(0, 0, linea);
	SSD1306_DrawHLine(0, 127, UI_SEP_Y);

	/* ── Zona azul: letra, flecha y celda Braille ──                       */
	if (running)
	{
		letra = text_buf[text_idx];

		/* La 'ñ' se guarda con un código propio porque no es ASCII */
		if ((uint8_t)letra == 0xF1u)
		{
			mask = braille_GetMaskEnie();
			SSD1306_WriteCharScaled(UI_LETTER_X, UI_LETTER_Y, 'N',
					UI_LETTER_SCALE);
			SSD1306_DrawHLine((uint8_t)(UI_LETTER_X + 2),
					(uint8_t)(UI_LETTER_X + 14),
					(uint8_t)(UI_LETTER_Y - 4));
		}
		else
		{
			mask = braille_GetMask(letra);
			SSD1306_WriteCharScaled(UI_LETTER_X, UI_LETTER_Y, letra,
					UI_LETTER_SCALE);
		}

		ui_DrawArrow(UI_ARROW_X0, UI_ARROW_X1, UI_ARROW_Y);

		/* Los caracteres sin traducción se muestran con la celda vacía */
		ui_DrawBrailleCell((mask == BRAILLE_INVALID) ? 0u : mask);
	}

	/* ── Tiempo vigente, a la derecha de la celda ──                       */
	snprintf(linea, sizeof(linea), "T=%us", (unsigned)tiempos[opcion_idx]);
	SSD1306_WriteString(UI_TIME_X, UI_TIME_Y, linea);

	/* ── Aviso de pausa, en la esquina inferior ──                         */
	if (paused)
	{
		SSD1306_WriteString(UI_STATE_X, UI_STATE_Y, "PAUSA");
	}

	SSD1306_UpdateScreen();
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */
/*
 * FSM dirigida por eventos. En FSM_IDLE se revisan las banderas que dejan
 * las interrupciones y el sondeo de encoder y pulsador; cada evento provoca
 * una transición a un estado que ejecuta UNA acción y vuelve a FSM_IDLE.
 *
 * El estado FSM_ANUNCIAR es especial: no se alcanza por una bandera sino por
 * una CONDICIÓN DE TIEMPO (hay anuncio pendiente y el mando lleva quieto lo
 * suficiente). Es la forma de implementar "hablar al detenerse" sin bloquear
 * el lazo ni gastar un temporizador.
 *
 * El orden de comprobación fija la prioridad: primero la entrada del usuario,
 * después el avance del secuenciador y el anuncio, y por último el
 * redibujado, que es lo más costoso.
 */
static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (flag_rx)        { fsm_state = FSM_PROC_UART;   }
        else if (flag_next) { fsm_state = FSM_NEXT_LETTER; }
        else if (voz_pendiente &&
                 ((HAL_GetTick() - t_ultimo_cambio) >= VOZ_ESPERA_MS))
        {
            fsm_state = FSM_ANUNCIAR;
        }
        else if (flag_refresh) { fsm_state = FSM_REFRESH_OLED; }
        break;

        /* ── Carácter recibido por el puerto serial ──                         */
    case FSM_PROC_UART:
    	flag_rx = 0;
    	{
    		uint8_t b = rx_byte;

    		/* Cada carácter aceptado se devuelve por el mismo puerto (eco).
    		 * El terminal no muestra lo que uno teclea: solo enseña lo que
    		 * llega desde la board, así que sin este eco se escribiría a
    		 * ciegas y no habría forma de detectar una errata.            */

    		/* La 'ñ' llega como la secuencia UTF-8 de dos bytes 0xC3 0xB1.
    		 * Al ver el primero se marca la espera; al llegar el segundo se
    		 * guarda con el código interno 0xF1, que cabe en un char y no
    		 * colisiona con ninguna letra ASCII.                           */
    		if (b == 0xC3u)
    		{
    			rx_utf8_pending = 1;
    		}
    		else if (rx_utf8_pending)
    		{
    			rx_utf8_pending = 0;
    			if (b == 0xB1u && rx_len < TEXT_MAX_LEN)
    			{
    				rx_line[rx_len++] = (char)0xF1;
    				print("\xC3\xB1");           /* eco de la 'ñ' */
    			}
    		}
    		else if (b == '\r' || b == '\n')
    		{
    			print("\r\n");
    			text_Start();                    /* fin de línea: arrancar */

    			if (text_len > 0u)
    			{
    				char aviso[96];
    				snprintf(aviso, sizeof(aviso),
    						"Traduciendo \"%s\" (%u letras)\r\n\r\n> ",
							text_buf, (unsigned)text_len);
    				print(aviso);
    			}
    			else
    			{
    				print("> ");
    			}
    		}
    		else if (b == '\b' || b == 127u)
    		{
    			if (rx_len > 0u)
    			{
    				rx_len--;
    				/* Retroceder, tapar con un espacio y retroceder otra
    				 * vez: es la forma de borrar visualmente en un terminal
    				 * que solo entiende texto.                            */
    				print("\b \b");
    			}
    		}
    		else if (b >= 32u && b <= 126u && rx_len < TEXT_MAX_LEN)
    		{
    			rx_line[rx_len++] = (char)b;
    			HAL_UART_Transmit(&huart2, (uint8_t *)&b, 1, 100);
    		}
    		/* Cualquier otro byte se descarta en silencio, sin eco */
    	}
    	fsm_state = FSM_IDLE;
    	break;

    /* ── El mando giró un paso: ya se actualizó opcion_idx ──              */
    case FSM_PROC_ENCODER:
        anuncio_Programar();     /* la voz esperará a que el mando se pare  */
        flag_refresh = 1;        /* la pantalla sí cambia al instante       */
        fsm_state = FSM_IDLE;
        break;

    /* ── Se pulsó el mando: pausa o reanuda ──                             */
    case FSM_PROC_BOTON:
        paused = paused ? 0u : 1u;
        flag_refresh = 1;
        fsm_state = FSM_IDLE;
        break;

    /* ── Venció el tiempo: avanzar a la letra siguiente ──                 */
    case FSM_NEXT_LETTER:
        flag_next = 0;
        if (running && !paused && text_len > 0u)
        {
            text_idx++;
            if (text_idx >= text_len) { text_idx = 0; }   /* bucle          */

            /* Mover los servos a la letra nueva con la misma máscara que
             * después dibuja la celda en pantalla.                         */
            {
                char    l = text_buf[text_idx];
                uint8_t m = ((uint8_t)l == 0xF1u) ? braille_GetMaskEnie()
                                                  : braille_GetMask(l);
                servos_Aplicar((m == BRAILLE_INVALID) ? 0u : m);
            }
            flag_refresh = 1;
        }
        fsm_state = FSM_IDLE;
        break;

    /* ── El mando se detuvo: anunciar el tiempo elegido ──                 */
    case FSM_ANUNCIAR:
        voz_pendiente = 0;
        switch (opcion_idx)
        {
        case 0:  clip_Reproducir(clip_uno,  clip_uno_largo);  break;
        case 1:  clip_Reproducir(clip_dos,  clip_dos_largo);  break;
        default: clip_Reproducir(clip_tres, clip_tres_largo); break;
        }
        fsm_state = FSM_IDLE;
        break;

    case FSM_REFRESH_OLED:
        flag_refresh = 0;
        ui_Draw();
        fsm_state = FSM_IDLE;
        break;

    default:
        fsm_state = FSM_IDLE;
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *            ISR — CALLBACKS DE INTERRUPCIONES (equivalente HAL)
 *  Las ISR reales (los vectores del NVIC) están en stm32f4xx_it.c y delegan
 *  aquí a través de HAL_xxx_IRQHandler().
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * HAL_TIM_PeriodElapsedCallback
 * La llama HAL_TIM_IRQHandler() en cada update de TIM10 (250 ms). Es
 * compartida por todos los timers con interrupción, así que hay que
 * verificar htim->Instance.
 *
 * Conmuta el LED de estado y lleva la cuenta del tiempo entre letras: al
 * acumular tiempo_en_segundos x 4 ticks, pide avanzar. El contador solo
 * avanza si hay palabra en curso y no está en pausa, de modo que el tiempo
 * no corre mientras el sistema está detenido.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        static uint8_t ticks = 0;

        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);

        if (running && !paused)
        {
            ticks++;
            if (ticks >= (uint8_t)(tiempos[opcion_idx] * TICKS_PER_SECOND))
            {
                ticks = 0;
                flag_next = 1;
            }
        }
        else
        {
            ticks = 0;      /* al reanudar, el tiempo empieza de cero       */
        }
    }
}

/*
 * audio_Rellenar — vuelca la mitad del buffer de salida que el DMA acaba de
 * liberar.
 *
 * Si hay un clip en curso saca sus muestras una a una; si no, escribe
 * silencio. Al terminar el clip se apaga el amplificador por su pin SD, lo
 * que corta la etapa de salida y elimina el siseo de reposo entre anuncios.
 *
 * El recorrido va de 4 en 4 medias palabras (una trama) y la muestra se
 * escribe en los DOS canales, alineada a la izquierda dentro de la ranura de
 * 32 bits: así suena sea cual sea el canal que tenga seleccionado el
 * amplificador.
 */
static void audio_Rellenar(uint16_t offset)
{
    const uint16_t n = BUF_HALFWORDS / 2u;

    for (uint16_t i = 0; i < n; i += 4u)
    {
        int16_t salida = 0;

        if (clip_activo)
        {
            if (clip_pos < clip_largo)
            {
                salida = clip_datos[clip_pos++];
            }
            else
            {
                clip_activo = 0;                        /* fin del clip     */
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
            }
        }

        tx_buf[offset + i]      = (uint16_t)salida;   /* L alto             */
        tx_buf[offset + i + 1u] = 0;                   /* L bajo             */
        tx_buf[offset + i + 2u] = (uint16_t)salida;   /* R alto             */
        tx_buf[offset + i + 3u] = 0;                   /* R bajo             */
    }
}

/*
 * Callbacks del audio en full-duplex. El HAL los invoca cuando la
 * transferencia conjunta llega a la mitad y cuando se completa; en ese
 * instante la mitad correspondiente ya se envió y puede reescribirse sin
 * competir con el DMA. Es el mecanismo de doble buffer.
 */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) { audio_Rellenar(0); }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) { audio_Rellenar(BUF_HALFWORDS / 2u); }
}

/*
 * HAL_UART_RxCpltCallback
 * La recepción por interrupción es de un solo uso: hay que rearmarla aquí
 * para el carácter siguiente.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        flag_rx = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
    }
}
