/*
 ******************************************************************************
 * @file           : main.c   (PRUEBA 4 — grabadora de palabras)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Captura, verificación y volcado de clips de voz
 ******************************************************************************
 *
 * OBJETIVO
 * Capturar las tres palabras que después confirmarán por voz el tiempo de
 * espera entre letras ("uno", "dos", "tres"). El flujo de trabajo es:
 *
 *   1. Grabar la palabra (tecla 'r').
 *   2. Escucharla por el parlante para comprobar que quedó bien ('p').
 *   3. Volcarla por el puerto serial como arreglo de C ('d').
 *   4. Pegar ese arreglo en el proyecto definitivo.
 *
 * Se graba una sola palabra a la vez y se repite el proceso tres veces. El
 * audio queda incrustado en Flash como dato constante, así que en el
 * programa final la reproducción es simplemente leer de un arreglo: no hay
 * escritura de Flash en tiempo de ejecución ni riesgo asociado.
 *
 * RECORTE DE SILENCIO: al volcar, solo se emite el tramo donde realmente
 * hay voz. La detección es por umbral de amplitud, con un pequeño margen a
 * cada lado para no cortar el ataque de la palabra ni su cola. Esto reduce
 * bastante el tamaño del arreglo, porque en un segundo de grabación la
 * palabra suele ocupar la mitad.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  Idéntico a la prueba de full-duplex: I2S2 principal transmite por PB15
 *  hacia el amplificador y el bloque I2S2ext recibe por PB14 desde el
 *  micrófono, con CK en PB13 y WS en PB12 compartidos.
 *
 * ─── Comandos por puerto serial ─────────────────────────────────────────────
 *   'r' → grabar (1 segundo)
 *   'p' → reproducir lo grabado
 *   'd' → volcar como arreglo de C
 *   's' → silencio
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

#define AUDIO_FS        16000u   /* frecuencia de muestreo, Hz              */

/* Duración de la grabación. Un segundo es holgado para "uno", "dos" o
 * "tres"; el recorte de silencio se encarga de que el arreglo final solo
 * contenga la parte útil.                                                  */
#define REC_SEGUNDOS        1u
#define REC_MUESTRAS    (AUDIO_FS * REC_SEGUNDOS)

/* Umbral de voz para el recorte, sobre el fondo de escala de 16 bits.
 * Por debajo se considera silencio. Si el volcado sale cortado o con
 * demasiado silencio, este es el número que hay que ajustar.               */
#define UMBRAL_VOZ        800

/* Margen que se conserva antes y después de la voz detectada, en muestras
 * (1600 = 100 ms), para no comerse el ataque ni la cola de la palabra.     */
#define MARGEN_MUESTRAS  1600

/* Buffers de intercambio con el DMA (formato de 32 bits, ver prueba 3):
 * cada trama son 4 medias palabras [L_alto][L_bajo][R_alto][R_bajo].       */
#define BUF_FRAMES        256u
#define BUF_HALFWORDS    (BUF_FRAMES * 4u)
#define BUF_SAMPLES      (BUF_FRAMES * 2u)

/* Estado del grabador */
typedef enum { EST_LISTO = 0, EST_GRABANDO, EST_REPRODUCIENDO } estado_t;

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,
    FSM_PROC_UART,
    FSM_REFRESH_OLED
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */

TIM_HandleTypeDef  htim10;
I2S_HandleTypeDef  hi2s2;
DMA_HandleTypeDef  hdma_i2s2_tx;
DMA_HandleTypeDef  hdma_i2s2_rx;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

static volatile uint8_t flag_rx      = 0;
static volatile uint8_t flag_refresh = 0;
static volatile uint8_t rx_byte      = 0;

static uint16_t tx_buf[BUF_HALFWORDS];
static uint16_t rx_buf[BUF_HALFWORDS];

/*
 * Almacén de la grabación: 16 000 muestras de 16 bits = 32 KB de RAM.
 * Es la estructura más grande del programa con diferencia, y por eso este
 * grabador es un programa aparte: en el proyecto final este espacio no hace
 * falta, porque el audio vivirá en Flash como dato constante.
 */
static int16_t rec_buf[REC_MUESTRAS];

static volatile estado_t estado   = EST_LISTO;
static volatile uint32_t rec_pos  = 0;      /* posición dentro de rec_buf   */
static volatile uint16_t nivel    = 0;      /* pico instantáneo             */
static uint32_t          rec_largo = 0;     /* muestras válidas grabadas    */

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
static void audio_ProcesarMitad(uint16_t offset);
static void rec_Iniciar(void);
static void rep_Iniciar(void);
static void dump_Emitir(void);
static void ui_Draw(void);
static void print(const char *s);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    gpio_Init();
    usart2_Init();
    print("\r\n=== PRUEBA 4: GRABADORA DE PALABRAS ===\r\n");

    tim10_Init();   print("  TIM10 OK\r\n");
    dma_Init();     print("  DMA OK\r\n");
    i2s2_Init();    print("  I2S full-duplex OK\r\n");
    i2c1_Init();    print("  I2C OK\r\n");

    SSD1306_SetI2C(&hi2c1);
    SSD1306_Init();
    print("  OLED OK\r\n\r\n");

    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_buf, rx_buf, BUF_SAMPLES)
        != HAL_OK)
    {
        print("  [ERROR] No arranco el audio\r\n");
        trap_Error();
    }

    print("Comandos:  r=grabar  p=reproducir  d=volcar  s=silencio\r\n");
    print("Diga la palabra justo despues de pulsar 'r'.\r\n\r\n");

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

/*
 * Reloj: HSI → PLL a 96 MHz y PLLI2S a 76.8 MHz, que da 16 kHz exactos.
 * PLLI2SM es el divisor propio del PLLI2S en el F411; sin él el PLL de
 * audio no engancha.
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

    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;           /* SD del amplificador     */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

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

/* Transmisión: Stream 4 canal 0 (SPI2_TX). Recepción: Stream 3 canal 3
 * (I2S2ext_RX). Ambos circulares.                                          */
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
    hdma_i2s2_rx.Init.Channel             = DMA_CHANNEL_3;
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

/* PB12/PB13/PB15 en AF5 (SPI2) y PB14 en AF6 (bloque extendido) */
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
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 1000);
}

/*
 * audio_ProcesarMitad — corazón del grabador, se ejecuta en los callbacks.
 *
 * Según el estado, hace una de tres cosas con la mitad del buffer que el
 * DMA acaba de liberar:
 *
 *   GRABANDO      : copia las muestras entrantes al almacén y avanza.
 *   REPRODUCIENDO : saca del almacén hacia la salida y avanza.
 *   LISTO         : escribe ceros en la salida (silencio).
 *
 * El recorrido va de 4 en 4 medias palabras porque cada trama ocupa cuatro:
 * [L_alto][L_bajo][R_alto][R_bajo]. El dato útil está en la ranura
 * izquierda, y la salida se escribe en las dos para que suene sea cual sea
 * el canal seleccionado en el amplificador.
 */
static void audio_ProcesarMitad(uint16_t offset)
{
    const uint16_t n = BUF_HALFWORDS / 2u;
    uint32_t pico = 0;
    estado_t est  = estado;

    for (uint16_t i = 0; i < n; i += 4u)
    {
        int16_t entrada = (int16_t)rx_buf[offset + i];
        int16_t salida  = 0;

        uint32_t mag = (entrada < 0) ? (uint32_t)(-(int32_t)entrada)
                                     : (uint32_t)entrada;
        if (mag > pico) { pico = mag; }

        if (est == EST_GRABANDO)
        {
            if (rec_pos < REC_MUESTRAS)
            {
                rec_buf[rec_pos++] = entrada;
            }
            else
            {
                estado = EST_LISTO;      /* se llenó el almacén: terminar   */
                est    = EST_LISTO;
            }
        }
        else if (est == EST_REPRODUCIENDO)
        {
            if (rec_pos < rec_largo)
            {
                salida = rec_buf[rec_pos++];
            }
            else
            {
                estado = EST_LISTO;      /* fin del clip                    */
                est    = EST_LISTO;
            }
        }

        tx_buf[offset + i]      = (uint16_t)salida;
        tx_buf[offset + i + 1u] = 0;
        tx_buf[offset + i + 2u] = (uint16_t)salida;
        tx_buf[offset + i + 3u] = 0;
    }

    nivel = (uint16_t)pico;
}

/*
 * rec_Iniciar — arranca la captura desde el principio del almacén.
 * El amplificador se silencia durante la grabación: si estuviera activo,
 * cualquier sonido residual se realimentaría al micrófono.
 */
static void rec_Iniciar(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    rec_pos   = 0;
    rec_largo = REC_MUESTRAS;
    estado    = EST_GRABANDO;
    print("Grabando 1 segundo... hable ahora.\r\n");
}

/*
 * rep_Iniciar — reproduce lo último grabado, para verificarlo de oído
 * antes de volcarlo.
 */
static void rep_Iniciar(void)
{
    if (rec_largo == 0u)
    {
        print("No hay nada grabado todavia.\r\n");
        return;
    }
    rec_pos = 0;
    estado  = EST_REPRODUCIENDO;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    print("Reproduciendo...\r\n");
}

/*
 * dump_Emitir — vuelca la grabación por el puerto serial como arreglo de C.
 *
 * Antes de emitir, busca dónde empieza y dónde termina la voz comparando
 * cada muestra contra un umbral, y añade un margen a cada lado. Así el
 * arreglo resultante contiene la palabra y no el segundo entero, lo que
 * reduce mucho el espacio que ocupará en Flash.
 *
 * El volcado es texto plano: basta con copiarlo del terminal y pegarlo en
 * un archivo del proyecto.
 */
static void dump_Emitir(void)
{
    char     linea[96];
    uint32_t ini = 0;
    uint32_t fin = 0;
    uint32_t i;

    if (rec_largo == 0u)
    {
        print("No hay nada grabado todavia.\r\n");
        return;
    }

    /* Primera muestra que supera el umbral */
    for (i = 0; i < rec_largo; i++)
    {
        int32_t v = rec_buf[i];
        if (v < 0) { v = -v; }
        if (v > UMBRAL_VOZ) { ini = i; break; }
    }
    /* Última muestra que supera el umbral */
    for (i = rec_largo; i > 0u; i--)
    {
        int32_t v = rec_buf[i - 1u];
        if (v < 0) { v = -v; }
        if (v > UMBRAL_VOZ) { fin = i; break; }
    }

    if (fin <= ini)
    {
        print("No se detecto voz. Baje UMBRAL_VOZ o hable mas fuerte.\r\n");
        return;
    }

    /* Aplicar el margen sin salirse del almacén */
    ini = (ini > MARGEN_MUESTRAS) ? (ini - MARGEN_MUESTRAS) : 0u;
    fin = (fin + MARGEN_MUESTRAS < rec_largo) ? (fin + MARGEN_MUESTRAS)
                                              : rec_largo;

    snprintf(linea, sizeof(linea),
             "\r\n/* %lu muestras, %lu ms, %lu bytes en Flash */\r\n",
             (unsigned long)(fin - ini),
             (unsigned long)((fin - ini) * 1000u / AUDIO_FS),
             (unsigned long)((fin - ini) * 2u));
    print(linea);
    print("const int16_t clip_XXX[] = {\r\n");

    for (i = ini; i < fin; i++)
    {
        char num[10];
        snprintf(num, sizeof(num), "%6d,", (int)rec_buf[i]);
        print(num);

        if (((i - ini + 1u) % 12u) == 0u)
        {
            print("\r\n");
        }
    }

    snprintf(linea, sizeof(linea),
             "\r\n};\r\nconst uint32_t clip_XXX_largo = %lu;\r\n\r\n",
             (unsigned long)(fin - ini));
    print(linea);
    print("Renombre clip_XXX por clip_uno, clip_dos o clip_tres.\r\n\r\n");
}

static void ui_Draw(void)
{
    char linea[24];
    const char *nombre = (estado == EST_GRABANDO)      ? "GRABANDO" :
                         (estado == EST_REPRODUCIENDO) ? "PLAY"     : "LISTO";
    uint8_t barra = (uint8_t)(nivel / 256u);

    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0, "GRABADORA");
    SSD1306_DrawHLine(0, 127, 10);

    snprintf(linea, sizeof(linea), "Estado: %s", nombre);
    SSD1306_WriteString(0, 16, linea);

    snprintf(linea, sizeof(linea), "Muestras: %lu",
             (unsigned long)rec_pos);
    SSD1306_WriteString(0, 26, linea);

    SSD1306_DrawEmptyRect(0, 40, 126, 10);
    for (uint8_t x = 0; x < barra && x < 126u; x++)
    {
        for (uint8_t y = 42; y < 48u; y++)
        {
            SSD1306_DrawPixel((uint8_t)(x + 1u), y, 1);
        }
    }

    SSD1306_WriteString(0, 54, "r=grab p=play d=dump");
    SSD1306_UpdateScreen();
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */

static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (flag_rx)           { fsm_state = FSM_PROC_UART;    }
        else if (flag_refresh) { fsm_state = FSM_REFRESH_OLED; }
        break;

    case FSM_PROC_UART:
        flag_rx = 0;
        switch (rx_byte)
        {
        case 'r': rec_Iniciar(); break;
        case 'p': rep_Iniciar(); break;
        case 'd': dump_Emitir(); break;   /* tarda unos segundos            */
        case 's':
            estado = EST_LISTO;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
            break;
        default: break;
        }
        flag_refresh = 1;
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
 *            ISR — CALLBACKS DE INTERRUPCIONES
 * ═══════════════════════════════════════════════════════════════════════════ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
        flag_refresh = 1;
    }
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) { audio_ProcesarMitad(0); }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) { audio_ProcesarMitad(BUF_HALFWORDS / 2u); }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        flag_rx = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
    }
}
