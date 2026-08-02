/*
 ******************************************************************************
 * @file           : main.c   (PRUEBA 3 — I2S full-duplex)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Captura y reproducción simultáneas sobre un mismo bus I2S
 ******************************************************************************
 *
 * OBJETIVO DE ESTA PRUEBA
 * Poner a funcionar las dos direcciones de audio a la vez sobre un único
 * juego de relojes, que es la configuración definitiva del proyecto. Se
 * verifica con dos modos seleccionables desde el puerto serial:
 *
 *   - MODO ECO: lo que capta el micrófono sale por el parlante en tiempo
 *     real. Es la prueba más contundente, porque solo funciona si ambas
 *     direcciones y el DMA están bien.
 *   - MODO TONO: senoidal generada internamente, para comprobar la salida
 *     de forma independiente del micrófono.
 *
 * ─── CÓMO FUNCIONA EL FULL-DUPLEX EN EL F411 ────────────────────────────────
 * El periférico I2S2 por sí solo puede transmitir O recibir, no ambas cosas.
 * Para lograr las dos direcciones el chip incorpora un bloque auxiliar
 * llamado I2S2ext, que se engancha al MISMO generador de reloj:
 *
 *   I2S2 (bloque principal) → TRANSMITE por PB15 hacia el amplificador
 *   I2S2ext (bloque aux.)   → RECIBE   por PB14 desde el micrófono
 *   CK (PB13) y WS (PB12)   → compartidos por ambos y por los dos módulos
 *
 * La consecuencia de compartir relojes es que las dos direcciones quedan
 * obligadas a la MISMA frecuencia de muestreo y el MISMO formato de datos.
 * Como el INMP441 trabaja en ranuras de 32 bits, todo el sistema va a 32
 * bits, incluida la salida hacia el amplificador.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  TIM10  → Blinky en PH1 cada 250 ms                            [CON IRQ]
 *  I2S2   → Maestro transmisor + full-duplex, 16 kHz, 32 bit     [SIN IRQ]
 *  DMA1_S4→ Transmisión hacia el amplificador (SPI2_TX, canal 0) [CON IRQ]
 *  DMA1_S3→ Recepción desde el micrófono (I2S2ext_RX, canal 3)   [CON IRQ]
 *  GPIO   → PA11 → SD del amplificador (habilitación / silencio)
 *  I2C1   → OLED SSD1306, SCL = PB8, SDA = PB9 (AF4), 100 kHz    [SIN IRQ]
 *  USART2 → 115200-8N1 por el VCP del ST-LINK                    [CON IRQ]
 *
 * ─── Comandos por puerto serial ─────────────────────────────────────────────
 *   'e' → modo eco (micrófono al parlante)
 *   't' → modo tono (senoidal de 440 Hz)
 *   's' → silencio
 *   '+' / '-' → subir o bajar la ganancia del eco
 *
 * ─── ADVERTENCIA ────────────────────────────────────────────────────────────
 * En modo eco, acercar el micrófono al parlante produce realimentación
 * acústica (un pitido creciente). Sepáralos y empieza con ganancia baja.
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

#define AUDIO_FS        16000u   /* frecuencia de muestreo, Hz              */
#define TONE_HZ           500u   /* nota del modo tono (divisor de 16000)   */

/*
 * Buffers de audio.
 *
 * FORMATO: el periférico trabaja en 32 bits, así que cada muestra ocupa DOS
 * medias palabras en memoria (primero los 16 bits altos, luego los bajos), y
 * cada trama estéreo son dos muestras. Es decir, por trama:
 *
 *   [L_alto][L_bajo][R_alto][R_bajo]   →   4 medias palabras
 *
 * El audio útil se maneja con 16 bits de resolución, que van en la parte
 * ALTA de cada ranura de 32; la parte baja queda en cero. Ese es justamente
 * el formato en que el INMP441 entrega sus muestras (alineadas a la
 * izquierda), así que capturar y reproducir usan el mismo esquema y el eco
 * se reduce a copiar medias palabras.
 *
 * 256 tramas a 16 kHz son 16 ms; el DMA avisa cada 8 ms.
 */
#define BUF_FRAMES        256u
#define BUF_HALFWORDS    (BUF_FRAMES * 4u)   /* tamaño real de los arreglos */
#define BUF_SAMPLES      (BUF_FRAMES * 2u)   /* "datos" de 32 bits para HAL */

/* Modos de funcionamiento */
typedef enum { MODO_SILENCIO = 0, MODO_TONO, MODO_ECO } modo_t;

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,
    FSM_PROC_UART,       /* llegó un comando por el puerto serial           */
    FSM_PROC_AUDIO,      /* media transferencia lista → medir nivel         */
    FSM_REFRESH_OLED
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */

TIM_HandleTypeDef  htim10;
I2S_HandleTypeDef  hi2s2;
DMA_HandleTypeDef  hdma_i2s2_tx;   /* SPI2_TX      → amplificador           */
DMA_HandleTypeDef  hdma_i2s2_rx;   /* I2S2ext_RX   → micrófono              */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

static volatile uint8_t flag_rx      = 0;
static volatile uint8_t flag_audio   = 0;
static volatile uint8_t flag_refresh = 0;
static volatile uint8_t rx_byte      = 0;
static volatile uint8_t half_listo   = 0;   /* 0 = primera mitad, 1 = segunda*/

/* Buffers que el DMA recorre directamente, en modo circular */
static uint16_t tx_buf[BUF_HALFWORDS];
static uint16_t rx_buf[BUF_HALFWORDS];

/* Tabla del tono, precalculada una vez para no hacer trigonometría dentro
 * de los callbacks. Solo guarda el canal izquierdo de cada trama; el
 * derecho se replica al copiar.                                            */
static int16_t tono_tabla[BUF_FRAMES];

static volatile modo_t  modo       = MODO_TONO;
static volatile uint8_t eco_gan    = 4;     /* ganancia del eco, 1..16      */
static volatile uint16_t nivel_pico = 0;

static fsm_state_t fsm_state = FSM_IDLE;

/* ═══════════════════════ Prototipos ═══════════════════════════════════════ */

static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim10_Init(void);
static void dma_Init(void);
static void i2s2_Init(void);
static void i2c1_Init(void);
static void usart2_Init(void);

static void fsm_Run(void);
static void trap_Error(void);
static void tono_Preparar(void);
static void audio_ProcesarMitad(uint16_t offset);
static void modo_Cambiar(modo_t nuevo);
static void ui_Draw(void);
static void print(const char *s);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    gpio_Init();
    usart2_Init();
    print("\r\n=== PRUEBA 3: I2S FULL-DUPLEX ===\r\n");

    tim10_Init();   print("  TIM10 OK\r\n");
    dma_Init();     print("  DMA (Tx stream 4 / Rx stream 3) OK\r\n");
    i2s2_Init();    print("  I2S full-duplex OK\r\n");
    i2c1_Init();    print("  I2C OK\r\n");

    SSD1306_SetI2C(&hi2c1);
    SSD1306_Init();
    print("  OLED OK\r\n\r\n");

    tono_Preparar();

    /* Arrancar las DOS direcciones con una sola llamada: el HAL pone en
     * marcha los dos streams de DMA y los dos bloques del periférico.      */
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_buf, rx_buf, BUF_SAMPLES)
        != HAL_OK)
    {
        print("  [ERROR] No arranco el full-duplex\r\n");
        trap_Error();
    }

    modo_Cambiar(MODO_TONO);   /* arranca con el tono, verificable de oido  */

    print("Comandos:  e=eco  t=tono  s=silencio  +/-=ganancia\r\n\r\n");
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);

    flag_refresh = 1;

    while (1)
    {
        fsm_Run();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *                        FUNCIONES DE CONFIGURACIÓN
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════ Reloj del sistema y reloj de audio ═══════════════════ */
/*
 *   VCO_in = HSI / PLLM = 16 MHz / 8 = 2 MHz
 *   PLL:     VCO = 2 x 96  = 192 MHz  →  SYSCLK = 96 MHz
 *   PLLI2S:  VCO = 2 x 192 = 384 MHz  →  I2SCLK = 384/5 = 76.8 MHz
 *
 * PLLI2SM: en el STM32F411 el PLLI2S tiene su propio divisor de entrada,
 * independiente del PLLM del PLL principal. Omitirlo deja el divisor en
 * cero y el PLL de audio nunca engancha.
 *
 * Con 76.8 MHz y formato de 32 bits, el periférico divide entre
 * 64 x (2 x I2SDIV + ODD) = 64 x 75 = 4800 → 16 000,00 Hz exactos, y el
 * reloj de bit resultante es 76,8 MHz / 75 = 1,024 MHz.
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
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 96;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
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
    PeriphClkInit.PLLI2S.PLLI2SM       = 8;
    PeriphClkInit.PLLI2S.PLLI2SN       = 192;
    PeriphClkInit.PLLI2S.PLLI2SR       = 5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { trap_Error(); }
}

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

    /* Silenciado hasta que el bus lleve datos válidos, para evitar el
     * chasquido del arranque.                                              */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

/* APB2 = 96 MHz;  PSC = 9599 → 10 kHz;  ARR = 2499 → 250 ms */
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

/* ═══════════════ DMA: un stream por dirección, ambos circulares ═══════════ */
/*
 * El full-duplex necesita DOS streams, uno por cada bloque del periférico, y
 * cada uno tiene su correspondencia fija en el mapa del hardware:
 *
 *   Transmisión (bloque principal SPI2) → Stream 4, canal 0
 *   Recepción   (bloque auxiliar I2S2ext) → Stream 3, canal 3
 *
 * Nótese que el canal de recepción es el 3 y no el 0: el canal 0 del stream
 * 3 corresponde a la recepción del SPI2 principal, que aquí no se usa
 * porque ese bloque está dedicado a transmitir. El bloque extendido tiene
 * su propia línea de petición.
 *
 * Ambos en modo circular: los buffers se recorren indefinidamente y el DMA
 * avisa a la mitad y al final de cada vuelta, que es lo que permite
 * procesar una mitad mientras la otra está en uso.
 */
static void dma_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ── Transmisión: memoria → periférico ──                              */
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

    /* ── Recepción: periférico → memoria ──                                */
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
 * Mode = MASTER_TX indica qué hace el bloque PRINCIPAL; al activar
 * FullDuplexMode, el bloque auxiliar toma automáticamente la dirección
 * contraria. Es decir: principal transmite (PB15) y auxiliar recibe (PB14).
 *
 * Los cuatro pines van en función alternada, pero con AF distinta: PB12,
 * PB13 y PB15 pertenecen al SPI2 (AF5), mientras que PB14 pertenece al
 * bloque extendido (AF6). Configurarlos todos con la misma AF es un error
 * silencioso: el bus funcionaría en transmisión y la recepción llegaría
 * siempre en cero.
 */
static void i2s2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB12 (WS), PB13 (CK) y PB15 (SD salida) → SPI2, AF5                  */
    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB14 (SD entrada) → I2S2ext, AF6                                     */
    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2S2ext;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_SPI2_CLK_ENABLE();

    hi2s2.Instance            = SPI2;
    hi2s2.Init.Mode           = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat     = I2S_DATAFORMAT_32B;
    hi2s2.Init.MCLKOutput     = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq      = I2S_AUDIOFREQ_16K;
    hi2s2.Init.CPOL           = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK) { trap_Error(); }
}

static void i2c1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
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

static void trap_Error(void)
{
    while (1) { __NOP(); }
}

static void print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 500);
}

/*
 * tono_Preparar — precalcula una vuelta completa de la senoidal.
 *
 * El buffer de 256 tramas a 16 kHz dura 16 ms. Para que el bucle circular
 * del DMA no produzca un chasquido en el empalme, el buffer debe contener
 * un número ENTERO de periodos: con 500 Hz caben exactamente 8.
 *
 * La trigonometría se hace aquí, una sola vez, y no dentro de los callbacks:
 * el resultado queda en una tabla que después solo se copia.
 */
static void tono_Preparar(void)
{
    for (uint16_t i = 0; i < BUF_FRAMES; i++)
    {
        float fase = 2.0f * 3.14159265f * (float)TONE_HZ *
                     (float)i / (float)AUDIO_FS;
        tono_tabla[i] = (int16_t)(6000.0f * sinf(fase));
    }
}

/*
 * audio_ProcesarMitad — rellena la mitad del buffer de salida que el DMA
 * acaba de dejar libre, y de paso mide el nivel de entrada.
 *
 * Se llama desde los callbacks con el desplazamiento de la mitad
 * correspondiente. El recorrido va de 4 en 4 medias palabras porque esa es
 * la longitud de una trama:  [L_alto][L_bajo][R_alto][R_bajo].
 *
 * El dato del micrófono está en la posición del canal izquierdo (el pin L/R
 * del módulo está a GND), y la salida se escribe en AMBOS canales para que
 * suene sea cual sea el canal que tenga seleccionado el amplificador.
 *
 * Nota sobre el eco: la ganancia se aplica como un desplazamiento de bits
 * en lugar de una multiplicación con coma flotante, porque esta función se
 * ejecuta cada 8 ms y conviene que sea barata.
 */
static void audio_ProcesarMitad(uint16_t offset)
{
    const uint16_t n = BUF_HALFWORDS / 2u;
    uint32_t pico = 0;
    modo_t   m    = modo;

    for (uint16_t i = 0; i < n; i += 4u)
    {
        /* Muestra capturada: 16 bits altos de la ranura izquierda */
        int16_t entrada = (int16_t)rx_buf[offset + i];

        uint32_t mag = (entrada < 0) ? (uint32_t)(-(int32_t)entrada)
                                     : (uint32_t)entrada;
        if (mag > pico) { pico = mag; }

        /* Elegir qué se envía a la salida según el modo */
        int32_t salida;
        switch (m)
        {
        case MODO_ECO:
            salida = ((int32_t)entrada * (int32_t)eco_gan) / 4;
            /* Recorte para evitar que un desbordamiento se convierta en
             * un chasquido: al pasarse, se satura en el máximo.            */
            if (salida >  32767) { salida =  32767; }
            if (salida < -32768) { salida = -32768; }
            break;

        case MODO_TONO:
            salida = tono_tabla[(offset + i) / 4u];
            break;

        default:
            salida = 0;
            break;
        }

        /* Escribir en las dos ranuras, alineado a la izquierda: el audio va
         * en la media palabra alta y la baja queda en cero.                */
        tx_buf[offset + i]      = (uint16_t)(int16_t)salida;   /* L alto    */
        tx_buf[offset + i + 1u] = 0;                            /* L bajo    */
        tx_buf[offset + i + 2u] = (uint16_t)(int16_t)salida;   /* R alto    */
        tx_buf[offset + i + 3u] = 0;                            /* R bajo    */
    }

    nivel_pico = (uint16_t)pico;
}

/*
 * modo_Cambiar — conmuta entre silencio, tono y eco.
 * El amplificador se habilita solo cuando hay algo que reproducir; en
 * silencio se apaga por su pin SD, que es un corte real de la etapa de
 * salida y no solo el envío de ceros.
 */
static void modo_Cambiar(modo_t nuevo)
{
    modo = nuevo;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,
                      (nuevo == MODO_SILENCIO) ? GPIO_PIN_RESET
                                               : GPIO_PIN_SET);
    flag_refresh = 1;
}

static void ui_Draw(void)
{
    char linea[24];
    const char *nombre = (modo == MODO_ECO)  ? "ECO" :
                         (modo == MODO_TONO) ? "TONO" : "SILENCIO";
    uint8_t barra = (uint8_t)(nivel_pico / 256u);

    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0, "I2S FULL-DUPLEX");
    SSD1306_DrawHLine(0, 127, 10);

    snprintf(linea, sizeof(linea), "Modo: %s", nombre);
    SSD1306_WriteString(0, 16, linea);

    snprintf(linea, sizeof(linea), "Ganancia eco: %u", (unsigned)eco_gan);
    SSD1306_WriteString(0, 26, linea);

    SSD1306_WriteString(0, 40, "Entrada:");
    SSD1306_DrawEmptyRect(0, 50, 126, 10);
    for (uint8_t x = 0; x < barra && x < 126u; x++)
    {
        for (uint8_t y = 52; y < 58u; y++)
        {
            SSD1306_DrawPixel((uint8_t)(x + 1u), y, 1);
        }
    }

    SSD1306_UpdateScreen();
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */

static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (flag_rx)           { fsm_state = FSM_PROC_UART;    }
        else if (flag_audio)   { fsm_state = FSM_PROC_AUDIO;   }
        else if (flag_refresh) { fsm_state = FSM_REFRESH_OLED; }
        break;

    case FSM_PROC_UART:
        flag_rx = 0;
        switch (rx_byte)
        {
        case 'e': modo_Cambiar(MODO_ECO);      break;
        case 't': modo_Cambiar(MODO_TONO);     break;
        case 's': modo_Cambiar(MODO_SILENCIO); break;
        case '+':
            if (eco_gan < 16u) { eco_gan++; }
            flag_refresh = 1;
            break;
        case '-':
            if (eco_gan > 1u) { eco_gan--; }
            flag_refresh = 1;
            break;
        default: break;
        }
        fsm_state = FSM_IDLE;
        break;

    case FSM_PROC_AUDIO:
        flag_audio = 0;
        /* El procesamiento real ya se hizo en el callback; aquí solo se
         * consume el evento para que la pantalla se entere de que hay
         * nivel nuevo.                                                     */
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
 * ═══════════════════════════════════════════════════════════════════════════ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        static uint8_t ticks = 0;

        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);

        if (++ticks >= 2u)
        {
            ticks = 0;
            flag_refresh = 1;
        }
    }
}

/*
 * Callbacks del full-duplex.
 *
 * A diferencia de los modos simplex, aquí hay UN par de callbacks para las
 * dos direcciones: el HAL avisa cuando la transferencia conjunta llega a la
 * mitad y cuando se completa. En ese instante, la mitad correspondiente del
 * buffer de recepción ya está llena y la del buffer de transmisión ya se
 * envió, así que ambas pueden tocarse sin competir con el DMA.
 *
 * El procesamiento se hace aquí y no en la FSM porque hay un plazo estricto:
 * los datos deben estar listos antes de que el DMA complete la otra mitad,
 * es decir, en menos de 8 ms. Aun así la función es corta y de coste
 * acotado: un recorrido lineal de 256 muestras sin divisiones ni coma
 * flotante.
 */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)
    {
        audio_ProcesarMitad(0);                  /* primera mitad           */
        half_listo = 0;
        flag_audio = 1;
    }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)
    {
        audio_ProcesarMitad(BUF_HALFWORDS / 2u); /* segunda mitad           */
        half_listo = 1;
        flag_audio = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        flag_rx = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
    }
}
