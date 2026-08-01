/*
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Traductor de texto a Braille — OLED + encoder + UART
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
 * letra a un ritmo configurable. En cada letra, la pantalla OLED muestra
 * simultáneamente tres cosas: la letra en grande, su traducción en una celda
 * Braille de 3x2 puntos, y el tiempo de espera vigente entre letras. Un
 * encoder rotativo cambia ese tiempo entre 1, 2 y 3 segundos de forma
 * cíclica, y su pulsador pausa o reanuda el recorrido.
 *
 * Esta etapa deja preparado el terreno para los 6 servomotores: la máscara
 * de 6 bits que ya se usa para dibujar la celda es exactamente la que
 * después decidirá qué punto sube y qué punto baja.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  TIM10  → Blinky en PH1 (LED de la board) cada 250 ms, y base de
 *           tiempo del secuenciador de letras                  [CON IRQ]
 *  TIM1   → Encoder rotativo en modo encoder, PA8/PA9 (AF1)    [SIN IRQ]
 *  I2C1   → OLED SSD1306, SCL = PB8, SDA = PB9 (AF4)           [SIN IRQ]
 *  GPIO   → Pulsador del encoder, SW = PA10 (entrada pull-up)
 *  USART2 → 115200-8N1 por el VCP del ST-LINK,
 *           TX = PA2 (AF7) por sondeo,
 *           RX = PA3 (AF7) por interrupción, 1 carácter        [CON IRQ]
 *
 * ─── Pines reservados para etapas siguientes ────────────────────────────────
 *  PA6, PA7, PB0, PB1, PB6, PB7 → PWM de los 6 servos (TIM3 y TIM4)
 *  PB12, PB13, PB14, PB15       → I2S2 full-duplex (micrófono y amplificador)
 *  PA11                         → SD del amplificador MAX98357A
 *
 * ─── Uso ────────────────────────────────────────────────────────────────────
 *  Escribir la palabra en el terminal y pulsar Enter: el recorrido arranca
 *  desde la primera letra y se repite en bucle al terminar.
 *  Girar el encoder  → cambia el tiempo de espera (1 → 2 → 3 → 1 s)
 *  Pulsar el encoder → pausa / reanuda el recorrido
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include "braille.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

#define TEXT_MAX_LEN          32u   /* capacidad del buffer de la palabra   */
#define ENC_COUNTS_PER_CLICK   4    /* encoder en cuadratura x4 (modo TI12) */
#define TICK_MS              250u   /* periodo del tick de TIM10, en ms     */
#define TICKS_PER_SECOND       4u   /* 1000 ms / TICK_MS                    */

/* Opciones de tiempo de espera entre letras, en segundos */
#define WAIT_OPTIONS           3u
static const uint8_t wait_options[WAIT_OPTIONS] = { 1u, 2u, 3u };

/* ── Geometría de la interfaz en la pantalla de 128x64 ──
 * Se centraliza aquí para poder recolocar la interfaz sin buscar números
 * sueltos por el código de dibujo.                                         */
#define UI_HEADER_Y      0     /* palabra completa, fuente 5x7             */
#define UI_SEP_Y        10     /* línea que separa el encabezado           */
#define UI_LETTER_X     10     /* letra grande                             */
#define UI_LETTER_Y     18
#define UI_LETTER_SCALE  4     /* 5x7 x4 = 20x28 px                        */
#define UI_FOOTER_Y     50     /* tiempo de espera vigente                 */

#define UI_CELL_COL_L   76     /* columna izquierda de la celda (puntos 1-3)*/
#define UI_CELL_COL_R  102     /* columna derecha de la celda   (puntos 4-6)*/
#define UI_CELL_ROW_1   18     /* filas de la celda Braille                */
#define UI_CELL_ROW_2   31
#define UI_CELL_ROW_3   44
#define UI_DOT_R         5     /* radio del punto activo                    */

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,        /* sin eventos pendientes                          */
    FSM_PROC_UART,       /* llegó un carácter por el puerto serial          */
    FSM_PROC_ENCODER,    /* el encoder giró → cambiar tiempo de espera      */
    FSM_PROC_BUTTON,     /* se pulsó el botón del encoder → pausa/reanuda   */
    FSM_NEXT_LETTER,     /* venció el tiempo → avanzar a la letra siguiente */
    FSM_REFRESH_OLED     /* redibujar la pantalla                           */
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */
/* Deben ser globales: stm32f4xx_it.c los referencia con 'extern' para
 * pasarlos a los HAL_xxx_IRQHandler() dentro de cada ISR.                   */

TIM_HandleTypeDef  htim1;    /* encoder rotativo          */
TIM_HandleTypeDef  htim10;   /* blinky y base de tiempo   */
I2C_HandleTypeDef  hi2c1;    /* pantalla OLED             */
UART_HandleTypeDef huart2;   /* puerto serial             */

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

/* Banderas de evento: las escriben las ISR/callbacks, las consume la FSM.
 * 'volatile' es obligatorio porque cambian fuera del flujo normal y sin él
 * el compilador podría optimizar (eliminar) las lecturas del lazo.          */
static volatile uint8_t flag_rx      = 0;   /* carácter recibido            */
static volatile uint8_t flag_next    = 0;   /* venció el tiempo de espera   */
static volatile uint8_t flag_refresh = 0;   /* hay que redibujar            */
static volatile uint8_t rx_byte      = 0;   /* buffer de recepción (1 byte) */

/* Texto en curso. text_buf guarda la palabra recibida; text_idx apunta a la
 * letra que se está mostrando. Se usa un buffer propio y no el de recepción
 * para que se pueda seguir escribiendo mientras la palabra anterior corre.  */
static char    text_buf[TEXT_MAX_LEN + 1] = {0};
static uint8_t text_len = 0;      /* letras válidas en el buffer            */
static uint8_t text_idx = 0;      /* letra que se muestra ahora             */

/* Buffer de ensamblado de la recepción: los caracteres se acumulan aquí
 * hasta que llega el fin de línea, y entonces se copian a text_buf.         */
static char    rx_line[TEXT_MAX_LEN + 1] = {0};
static uint8_t rx_len = 0;
static uint8_t rx_utf8_pending = 0;  /* 1 = se recibió el 0xC3 de la 'ñ'    */

/* Configuración y estado del secuenciador */
static uint8_t wait_idx  = 0;     /* índice dentro de wait_options[]        */
static uint8_t paused    = 0;     /* 1 = recorrido detenido                 */
static uint8_t running   = 0;     /* 1 = hay una palabra en curso           */

/* Estado del encoder y del pulsador */
static uint16_t enc_last_cnt = 0; /* último CNT de TIM1 ya consumido        */
static uint8_t  sw_prev      = 1; /* estado anterior del botón (1 = suelto) */

static fsm_state_t fsm_state = FSM_IDLE;

/* ═══════════════════════ Prototipos ═══════════════════════════════════════ */

static void SystemClock_Config(void);
static void gpio_Init(void);          /* PH1 (LED) y PA10 (botón)           */
static void tim10_Init(void);         /* tick de 250 ms, con interrupción   */
static void tim1_encoder_Init(void);  /* encoder PA8/PA9, sin interrupción  */
static void i2c1_Init(void);          /* bus de la pantalla, PB8/PB9        */
static void usart2_Init(void);        /* PA2 TX / PA3 RX por el VCP         */

static void fsm_Run(void);
static void trap_Error(void);
static void encoder_Poll(void);
static void button_Poll(void);
static void text_Start(void);
static void ui_Draw(void);
static void ui_DrawBrailleCell(uint8_t mask);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();             /* SysTick a 1 ms, caché de Flash, NVIC         */
    SystemClock_Config();   /* HSI a 16 MHz, sin PLL                        */

    gpio_Init();            /* LED de estado y pulsador del encoder         */
    tim10_Init();           /* tick de 250 ms por interrupción              */
    tim1_encoder_Init();    /* encoder decodificado por hardware            */
    i2c1_Init();            /* bus I2C del panel                            */
    usart2_Init();          /* serial: Tx por sondeo, Rx por interrupción   */

    SSD1306_SetI2C(&hi2c1); /* el driver ya sabe por dónde hablar           */
    SSD1306_Init();         /* secuencia de arranque del panel              */

    /* Pantalla de bienvenida: sin palabra cargada todavía */
    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0,  "TRADUCTOR BRAILLE");
    SSD1306_DrawHLine(0, 127, UI_SEP_Y);
    SSD1306_WriteString(0, 20, "Escriba una palabra");
    SSD1306_WriteString(0, 30, "y pulse Enter.");
    SSD1306_WriteString(0, 44, "Encoder: tiempo");
    SSD1306_WriteString(0, 54, "Boton: pausa");
    SSD1306_UpdateScreen();

    /* Instrucciones también por el puerto serial (transmisión por sondeo) */
    {
        const char hello[] =
            "\r\n== Traductor de texto a Braille ==\r\n"
            "Escriba una palabra y pulse Enter.\r\n"
            "Encoder: tiempo entre letras (1/2/3 s). Boton: pausa.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)hello,
                          (uint16_t)strlen(hello), 200);
    }

    /* Armar la recepción: desde aquí, cada byte que llegue dispara la
     * interrupción de USART2 y el callback lo deja en rx_byte.              */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);

    /* Lazo de aplicación: no bloquea. Los periféricos avisan por banderas;
     * aquí solo se sondean encoder y botón y se despachan los eventos.      */
    while (1)
    {
        encoder_Poll();     /* lectura del contador del encoder             */
        button_Poll();      /* lectura del pulsador con detección de flanco */
        fsm_Run();          /* despacho de eventos                          */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *                        FUNCIONES DE CONFIGURACIÓN
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════════ Configuración del reloj ══════════════════════════ */
/*
 * Oscilador interno HSI a 16 MHz, sin PLL. A esta frecuencia los tres buses
 * (AHB, APB1, APB2) pueden ir con divisor 1 y la Flash no necesita estados
 * de espera. Es suficiente para todo lo de esta etapa: I2C a 400 kHz, UART
 * a 115200 y el PWM de servos a 50 Hz que vendrá después.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* El HSI ya viene encendido tras el reset: se confirma y se usa        */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { trap_Error(); }

    /* HSI como SYSCLK, todos los divisores de bus en 1                     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK = 16 MHz  */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* APB1 = 16 MHz  */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2 = 16 MHz  */

    /* Latencia 0 de Flash: válida hasta 30 MHz                             */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        trap_Error();
    }
}

/* ═══════════════════════ GPIO: LED de estado y pulsador ═══════════════════ */
/*
 * PH1 = LED integrado de la board. En el F411, PH0/PH1 son los pines del
 * oscilador externo de alta frecuencia; como el sistema corre con el HSI
 * interno y no hay cristal, quedan libres como GPIO.
 * (Si se quisiera usar el LED LD2 de la Nucleo, sería PA5 en GPIOA.)
 *
 * PA10 = pulsador SW del encoder. Entrada con pull-up interno: el módulo
 * cierra el contacto a GND al pulsar, así que en reposo se lee 1 y pulsado 0.
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Habilitar el reloj de los puertos en el bus AHB1
       Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOxEN          */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PH1 como salida push-pull para el LED de estado                      */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* PA10 como entrada con pull-up para el pulsador                        */
    GPIO_InitStruct.Pin  = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ═══════════════════════ TIM10: tick de 250 ms (IRQ) ══════════════════════ */
/*
 * Cadena de reloj:
 *   HSI (16 MHz) → APB2 (16 MHz) → reloj de TIM10 (16 MHz)
 *
 * PSC = 15999 → tick = 16 MHz / (15999+1) = 1 kHz  (1 ms por cuenta)
 * ARR = 249   → evento de update cada (249+1) x 1 ms = 250 ms
 *
 * Este único timer cumple dos funciones: conmutar el LED de estado a 250 ms
 * y servir de base de tiempo al secuenciador de letras (contando cuántos
 * ticks de 250 ms han pasado). Así no se gasta un segundo timer, que hará
 * falta para el PWM de los servos.
 *
 * TIM10 comparte el vector TIM1_UP_TIM10_IRQn con el update de TIM1; no hay
 * conflicto porque TIM1 (encoder) no tiene interrupciones habilitadas.
 */
static void tim10_Init(void)
{
    __HAL_RCC_TIM10_CLK_ENABLE();

    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 15999;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 249;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Cargar la configuración en los registros del timer (PSC, ARR, CR1)   */
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) { trap_Error(); }

    /* Arrancar el contador con la interrupción de update habilitada        */
    HAL_TIM_Base_Start_IT(&htim10);

    /* Registrar la IRQ en el NVIC y darle prioridad                        */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* ═══════════════════════ TIM1: encoder rotativo (sin IRQ) ═════════════════ */
/*
 * El encoder se decodifica por HARDWARE con el timer en modo encoder TI12
 * (cuadratura x4): el contador del timer sube o baja solo, siguiendo los
 * flancos de ambos canales, con filtro digital de entrada que absorbe los
 * rebotes mecánicos. La posición se lee por sondeo del registro CNT.
 *
 * Esta es la razón de que el encoder NO necesite interrupciones: no hay que
 * "atrapar" los flancos con el CPU porque el periférico ya los cuenta. Un
 * esquema con interrupciones externas (EXTI) en los dos pines gastaría
 * tiempo de CPU en cada flanco y obligaría a filtrar rebotes en software.
 *
 * DT  → PA8 = TIM1_CH1 (AF1)
 * CLK → PA9 = TIM1_CH2 (AF1)
 */
static void tim1_encoder_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct = {0};
    TIM_Encoder_InitTypeDef sEncoder        = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA8/PA9 en función alternada AF1: dejan de ser GPIO y pasan a ser
     * las entradas de captura CH1/CH2 del timer                            */
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

    sEncoder.EncoderMode  = TIM_ENCODERMODE_TI12;   /* cuadratura x4        */
    sEncoder.IC1Polarity  = TIM_ICPOLARITY_FALLING; /* fija el sentido de
                                                       conteo según cómo
                                                       estén cableados
                                                       DT y CLK; si el giro
                                                       resulta invertido,
                                                       usar RISING          */
    sEncoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoder.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoder.IC1Filter    = 0x0F;                   /* filtro antirrebote   */
    sEncoder.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sEncoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoder.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoder.IC2Filter    = 0x0F;

    if (HAL_TIM_Encoder_Init(&htim1, &sEncoder) != HAL_OK) { trap_Error(); }

    /* Arrancar ambos canales sin interrupción (no se usa la versión _IT)   */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

/* ═══════════════════════ I2C1: bus de la pantalla ═════════════════════════ */
/*
 * SCL = PB8, SDA = PB9, en función alternada AF4 y con la salida en drenador
 * abierto (open-drain), como exige el bus I2C: los dispositivos solo tiran
 * la línea a nivel bajo y las resistencias de pull-up del módulo la suben.
 *
 * El SSD1306 no requiere interrupciones: las transferencias se hacen por
 * sondeo con HAL_I2C_Mem_Write. A 400 kHz (Fast Mode), volcar la pantalla
 * completa (1024 bytes más comandos) toma del orden de 25 ms, holgado para
 * un refresco cada 250 ms.
 */
static void i2c1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    SSD1306_BusRecover(GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;   /* drenador abierto      */
    GPIO_InitStruct.Pull      = GPIO_NOPULL;       /* pull-ups del módulo   */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;          /* 400 kHz, Fast Mode     */
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
 * 115200-8N1. La transmisión se hace por sondeo; la recepción es de un
 * carácter a la vez por interrupción, que es lo que estrictamente lo
 * requiere: los caracteres llegan en cualquier momento y no se puede
 * bloquear el lazo esperándolos.
 *
 * PA2/PA3 (AF7) están cableados internamente al Virtual COM Port del
 * ST-LINK, así que el serial viaja por el mismo cable USB de programación
 * sin cableado adicional.
 *
 * Se usa el driver asíncrono del HAL (UART_HandleTypeDef, HAL_UART_Init,
 * HAL_UART_Receive_IT); el nombre "USART2" aparece solo porque así se llama
 * la instancia física del periférico en el F411.
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

    /* Cargar la configuración; el HAL calcula el divisor de baudios (BRR)
     * a partir del reloj de APB1                                           */
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
    while (1)
    {
        __NOP();
    }
}

/*
 * text_Start — carga la línea recibida como palabra en curso.
 * Se llama cuando llega el fin de línea. Copia el buffer de ensamblado al
 * buffer de trabajo, reinicia el índice y arranca el recorrido.
 */
static void text_Start(void)
{
    if (rx_len == 0)
    {
        return;                     /* Enter sin texto: no hacer nada       */
    }

    memcpy(text_buf, rx_line, rx_len);
    text_buf[rx_len] = '\0';
    text_len = rx_len;
    text_idx = 0;
    running  = 1;
    paused   = 0;

    rx_len = 0;                     /* buffer listo para la próxima palabra */
    flag_refresh = 1;               /* mostrar de inmediato la primera letra*/
}

/*
 * encoder_Poll — lectura del encoder por sondeo, sin interrupciones.
 * El TIM1 en modo encoder cuenta 4 flancos por cada clic mecánico. Aquí se
 * compara el contador actual con el último valor ya consumido; cuando se
 * acumula al menos un clic completo se genera el evento para la FSM y el
 * residuo queda pendiente para la siguiente vuelta.
 *
 * La resta se hace en aritmética modular de 16 bits, lo que maneja
 * correctamente el desborde del contador en ambos sentidos.
 */
static void encoder_Poll(void)
{
    uint16_t cnt    = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t  diff   = (int16_t)(cnt - enc_last_cnt);
    int16_t  clicks = (int16_t)(diff / ENC_COUNTS_PER_CLICK);

    if (clicks != 0)
    {
        enc_last_cnt = (uint16_t)(enc_last_cnt +
                                  (uint16_t)(clicks * ENC_COUNTS_PER_CLICK));

        /* Recorrido cíclico de las opciones de tiempo: girar en un sentido
         * avanza y en el otro retrocede, dando la vuelta en los extremos.  */
        if (clicks > 0)
        {
            wait_idx = (uint8_t)((wait_idx + 1u) % WAIT_OPTIONS);
        }
        else
        {
            wait_idx = (uint8_t)((wait_idx + WAIT_OPTIONS - 1u) % WAIT_OPTIONS);
        }

        if (fsm_state == FSM_IDLE) { fsm_state = FSM_PROC_ENCODER; }
    }
}

/*
 * button_Poll — lectura del pulsador con detección de flanco.
 * El pin tiene pull-up, así que reposo = 1 y pulsado = 0. Solo interesa la
 * TRANSICIÓN de suelto a pulsado: sin esta comprobación, mantener el botón
 * generaría un evento en cada pasada del lazo. Como el sondeo ocurre miles
 * de veces por segundo y el contacto rebota unos pocos milisegundos, se
 * confirma la pulsación exigiendo que el estado se mantenga; aquí basta con
 * el flanco porque la acción (pausar) es idempotente frente a un rebote
 * corto y el usuario percibiría un doble rebote como un solo toque.
 */
static void button_Poll(void)
{
    uint8_t sw_now = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)
                     ? 0u : 1u;

    if (sw_prev == 1u && sw_now == 0u)        /* flanco de bajada = pulsado */
    {
        if (fsm_state == FSM_IDLE) { fsm_state = FSM_PROC_BUTTON; }
    }
    sw_prev = sw_now;
}

/*
 * ui_DrawBrailleCell — dibuja la celda de 6 puntos a partir de la máscara.
 *
 * Los puntos 1-2-3 forman la columna izquierda de arriba abajo y los
 * puntos 4-5-6 la columna derecha, según la numeración del signo generador.
 * Un punto activo se dibuja como círculo relleno; uno inactivo, como un
 * único píxel, para que la retícula de la celda siga siendo visible (así se
 * aprecia qué puntos existen y cuáles están levantados, igual que en las
 * tablas del alfabeto Braille).
 */
static void ui_DrawBrailleCell(uint8_t mask)
{
    /* Tabla local: cada punto con su máscara y su posición en pantalla.
     * Recorrerla en un bucle evita repetir seis veces el mismo if.         */
    static const struct
    {
        uint8_t bit;
        uint8_t x;
        uint8_t y;
    } dots[6] =
    {
        { BRAILLE_DOT_1, UI_CELL_COL_L, UI_CELL_ROW_1 },
        { BRAILLE_DOT_2, UI_CELL_COL_L, UI_CELL_ROW_2 },
        { BRAILLE_DOT_3, UI_CELL_COL_L, UI_CELL_ROW_3 },
        { BRAILLE_DOT_4, UI_CELL_COL_R, UI_CELL_ROW_1 },
        { BRAILLE_DOT_5, UI_CELL_COL_R, UI_CELL_ROW_2 },
        { BRAILLE_DOT_6, UI_CELL_COL_R, UI_CELL_ROW_3 },
    };

    for (uint8_t i = 0; i < 6; i++)
    {
        if (mask & dots[i].bit)
        {
            SSD1306_DrawFilledCircle(dots[i].x, dots[i].y, UI_DOT_R);
        }
        else
        {
            SSD1306_DrawPixel(dots[i].x, dots[i].y, 1);
        }
    }
}

/*
 * ui_Draw — redibuja la pantalla completa.
 *
 * Se redibuja todo en cada refresco (borrar el framebuffer y volver a
 * componer) en lugar de actualizar zonas sueltas: con 1024 bytes de buffer
 * y un refresco cada 250 ms el costo es irrelevante, y a cambio se elimina
 * por completo la posibilidad de restos de un dibujo anterior en pantalla.
 *
 * Composición de los tres elementos que pide la interfaz:
 *   - palabra completa arriba, con la letra en curso marcada entre corchetes
 *   - letra actual en grande a la izquierda
 *   - celda Braille de 3x2 a la derecha
 *   - tiempo de espera vigente abajo
 */
static void ui_Draw(void)
{
    char    linea[48];            /* palabra (hasta 32) + sufijo " (nn/nn)" */
    char    letra;
    uint8_t mask;

    SSD1306_Fill(0);                       /* limpiar el framebuffer        */

    /* ── Encabezado: la palabra y la posición dentro de ella ──            */
    if (running)
    {
        snprintf(linea, sizeof(linea), "%s (%u/%u)",
                 text_buf, (unsigned)(text_idx + 1u), (unsigned)text_len);
    }
    else
    {
        snprintf(linea, sizeof(linea), "Sin palabra");
    }
    SSD1306_WriteString(0, UI_HEADER_Y, linea);
    SSD1306_DrawHLine(0, 127, UI_SEP_Y);

    /* ── Letra actual y su traducción ──                                   */
    if (running)
    {
        letra = text_buf[text_idx];

        /* La 'ñ' se guarda internamente con un código propio porque no es
         * ASCII; se detecta aquí para pedir su máscara a la función
         * específica y dibujarla en pantalla como "N".                     */
        if ((uint8_t)letra == 0xF1u)
        {
            mask = braille_GetMaskEnie();
            SSD1306_WriteCharScaled(UI_LETTER_X, UI_LETTER_Y, 'N',
                                    UI_LETTER_SCALE);
            /* virgulilla de la eñe, encima de la letra grande             */
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

        /* Los caracteres sin traducción (espacios, signos) se muestran con
         * la celda vacía en lugar de dejar la zona en blanco.              */
        ui_DrawBrailleCell((mask == BRAILLE_INVALID) ? 0u : mask);
    }

    /* ── Pie: tiempo de espera vigente y estado del recorrido ──           */
    snprintf(linea, sizeof(linea), "T=%us %s",
             (unsigned)wait_options[wait_idx],
             paused ? "PAUSA" : "");
    SSD1306_WriteString(0, UI_FOOTER_Y, linea);

    SSD1306_UpdateScreen();                /* volcar al panel por I2C       */
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */
/*
 * FSM dirigida por eventos. En FSM_IDLE se revisan las banderas que dejan
 * las interrupciones (recepción serial, vencimiento del tiempo, refresco) y
 * el sondeo de encoder y botón. Cada evento provoca una transición a un
 * estado de proceso que ejecuta UNA acción y vuelve a FSM_IDLE.
 *
 * El orden de comprobación establece la prioridad: primero la entrada del
 * usuario (serial, encoder, botón), después el avance del secuenciador y por
 * último el redibujado, que es lo más costoso en tiempo.
 */
static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (flag_rx)           { fsm_state = FSM_PROC_UART;    }
        else if (flag_next)    { fsm_state = FSM_NEXT_LETTER;  }
        else if (flag_refresh) { fsm_state = FSM_REFRESH_OLED; }
        break;

    /* ── Carácter recibido por el puerto serial ──                         */
    case FSM_PROC_UART:
        flag_rx = 0;                        /* consumir el evento           */
        {
            uint8_t b = rx_byte;

            /* La 'ñ' llega como la secuencia UTF-8 de dos bytes 0xC3 0xB1.
             * Al ver el primero se marca la espera; al llegar el segundo se
             * guarda con el código interno 0xF1 (la 'ñ' de Latin-1), que
             * cabe en un char y no colisiona con ninguna letra ASCII.      */
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
                }
            }
            else if (b == '\r' || b == '\n')
            {
                text_Start();               /* fin de línea: arrancar       */
            }
            else if (b == '\b' || b == 127u)
            {
                if (rx_len > 0) { rx_len--; }   /* retroceso                */
            }
            else if (b >= 32u && b <= 126u && rx_len < TEXT_MAX_LEN)
            {
                rx_line[rx_len++] = (char)b;
            }
            /* Cualquier otro byte se descarta en silencio                  */
        }
        fsm_state = FSM_IDLE;
        break;

    /* ── El encoder giró: ya se actualizó wait_idx en encoder_Poll ──      */
    case FSM_PROC_ENCODER:
        flag_refresh = 1;                   /* mostrar el nuevo tiempo      */
        fsm_state = FSM_IDLE;
        break;

    /* ── Se pulsó el botón del encoder: pausa / reanuda ──                 */
    case FSM_PROC_BUTTON:
        paused = paused ? 0u : 1u;
        flag_refresh = 1;
        fsm_state = FSM_IDLE;
        break;

    /* ── Venció el tiempo de espera: avanzar a la letra siguiente ──       */
    case FSM_NEXT_LETTER:
        flag_next = 0;                      /* consumir el evento           */
        if (running && !paused && text_len > 0u)
        {
            text_idx++;
            if (text_idx >= text_len)
            {
                text_idx = 0;               /* recorrido cíclico            */
            }
            flag_refresh = 1;
        }
        fsm_state = FSM_IDLE;
        break;

    /* ── Redibujar la pantalla ──                                          */
    case FSM_REFRESH_OLED:
        flag_refresh = 0;                   /* consumir el evento           */
        ui_Draw();
        fsm_state = FSM_IDLE;
        break;

    default:                                /* estado inválido: recuperar   */
        fsm_state = FSM_IDLE;
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *            ISR — CALLBACKS DE INTERRUPCIONES (equivalente HAL)
 *  Las ISR reales (los vectores del NVIC) están en stm32f4xx_it.c y delegan
 *  aquí a través de HAL_xxx_IRQHandler(). Se mantienen cortos: capturan el
 *  dato, levantan la bandera y salen; el trabajo lo hace la FSM del lazo.
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * HAL_TIM_PeriodElapsedCallback
 * La llama HAL_TIM_IRQHandler() en cada evento de update. Es compartida por
 * todos los timers con interrupción, así que hay que verificar htim->Instance.
 *
 * Cada 250 ms conmuta el LED de estado y suma un tick al secuenciador.
 * Cuando los ticks acumulados alcanzan el tiempo de espera configurado
 * (tiempo_en_segundos x 4 ticks), se pide avanzar de letra.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        static uint8_t ticks = 0;

        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);   /* blinky de estado        */

        /* El contador solo avanza si hay palabra en curso y no está en
         * pausa; así el tiempo no "corre" mientras el sistema está detenido */
        if (running && !paused)
        {
            ticks++;
            if (ticks >= (uint8_t)(wait_options[wait_idx] * TICKS_PER_SECOND))
            {
                ticks = 0;
                flag_next = 1;
            }
        }
        else
        {
            ticks = 0;      /* al reanudar, el tiempo empieza de cero      */
        }
    }
}

/*
 * HAL_UART_RxCpltCallback
 * La llama HAL_UART_IRQHandler() al completarse la recepción del byte
 * solicitado. La recepción por interrupción es de un solo uso, así que hay
 * que rearmarla aquí para el carácter siguiente.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        flag_rx = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
    }
}
