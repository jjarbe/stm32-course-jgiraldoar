/*
 ******************************************************************************
 * @file           : main.c   (PRUEBA 5 — encoder con confirmación por voz)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Selección del tiempo de espera con anuncio hablado
 ******************************************************************************
 *
 * OBJETIVO
 * Unir el encoder rotativo con la reproducción de audio: al girar el mando
 * se elige el tiempo de espera entre letras (1, 2 o 3 segundos) y el sistema
 * lo confirma en voz alta con la palabra correspondiente.
 *
 * ─── DOS DETALLES DE COMPORTAMIENTO ─────────────────────────────────────────
 *
 * 1. TRES CLICS POR CAMBIO. Con solo tres opciones, un clic por opción hace
 *    que el valor salte demasiado rápido y sea difícil detenerse donde uno
 *    quiere. Aquí hacen falta tres clics mecánicos para pasar de una opción
 *    a la siguiente, lo que da un tacto mucho más controlable. El sondeo del
 *    encoder ya consume solo múltiplos completos y guarda el residuo, así
 *    que basta con definir el "paso" como tres clics.
 *
 * 2. LA VOZ SUENA AL DETENERSE, NO EN CADA CAMBIO. Si se gira rápido de 1 a
 *    3, encadenar "uno, dos, tres" sería confuso y lento. En su lugar, cada
 *    cambio arma una espera de 400 ms que se reinicia con cada movimiento
 *    nuevo; solo cuando el mando lleva ese tiempo quieto se reproduce el
 *    valor final. Si llega un cambio mientras un clip está sonando, el clip
 *    se corta y se anuncia el nuevo valor.
 *
 *    La espera se mide con HAL_GetTick(), que tiene resolución de 1 ms, y no
 *    con el tick de 250 ms del parpadeo: un cuarto de segundo es demasiado
 *    grueso para que la respuesta se sienta natural.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  TIM10  → Blinky en PH1 cada 250 ms                            [CON IRQ]
 *  TIM1   → Encoder rotativo, PA8 (DT) / PA9 (CLK), AF1          [SIN IRQ]
 *  GPIO   → PA10 pulsador del encoder, PA11 SD del amplificador
 *  I2S2   → Full-duplex, 16 kHz, 32 bit (solo se usa la salida)  [SIN IRQ]
 *  DMA1_S4→ Transmisión hacia el amplificador                    [CON IRQ]
 *  DMA1_S3→ Recepción desde el micrófono                         [CON IRQ]
 *  I2C1   → OLED SSD1306, PB8/PB9, 100 kHz                       [SIN IRQ]
 *  USART2 → 115200-8N1 por el VCP                                [CON IRQ]
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include "clips_voz.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

/* ── Encoder ──
 * ENC_COUNTS_PER_CLICK es una propiedad del HARDWARE: el timer en modo
 * cuadratura x4 cuenta cuatro flancos por cada clic mecánico del mando.
 * ENC_CLICKS_PER_STEP es una decisión de USO: cuántos clics se exigen para
 * cambiar de opción. Separarlos deja claro qué es físico y qué es ajustable.
 */
#define ENC_COUNTS_PER_CLICK   4
#define ENC_CLICKS_PER_STEP    3     /* decisión de uso: cuántos clics por opción */
#define ENC_COUNTS_PER_STEP   (ENC_COUNTS_PER_CLICK * ENC_CLICKS_PER_STEP)

/* Tiempo que el mando debe permanecer quieto antes de anunciar el valor */
#define VOZ_ESPERA_MS        400u

/* Opciones de tiempo de espera entre letras, en segundos */
#define OPCIONES               3u
static const uint8_t tiempos[OPCIONES] = { 1u, 2u, 3u };

/* Buffers de intercambio con el DMA. Formato de 32 bits: cada trama son 4
 * medias palabras [L_alto][L_bajo][R_alto][R_bajo].                        */
#define BUF_FRAMES           256u
#define BUF_HALFWORDS       (BUF_FRAMES * 4u)
#define BUF_SAMPLES         (BUF_FRAMES * 2u)

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,
    FSM_PROC_ENCODER,    /* el mando giró lo suficiente → cambiar opción    */
    FSM_PROC_BOTON,      /* se pulsó el mando → repetir el anuncio          */
    FSM_ANUNCIAR,        /* venció la espera → reproducir el clip           */
    FSM_REFRESH_OLED
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */

TIM_HandleTypeDef  htim1;         /* encoder                   */
TIM_HandleTypeDef  htim10;        /* blinky                    */
I2S_HandleTypeDef  hi2s2;
DMA_HandleTypeDef  hdma_i2s2_tx;
DMA_HandleTypeDef  hdma_i2s2_rx;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

static volatile uint8_t flag_refresh = 0;

/* Buffers del DMA */
static uint16_t tx_buf[BUF_HALFWORDS];
static uint16_t rx_buf[BUF_HALFWORDS];

/* Reproductor de clips. El callback lee de aquí; el lazo principal solo
 * arranca y detiene. 'clip_activo' actúa de interruptor y se manipula en un
 * orden concreto para que el callback nunca vea un estado a medio cambiar. */
static const int16_t * volatile clip_datos = 0;
static volatile uint32_t        clip_largo = 0;
static volatile uint32_t        clip_pos   = 0;
static volatile uint8_t         clip_activo = 0;

/* Selección y anuncio */
static uint8_t  opcion_idx   = 0;   /* índice dentro de tiempos[]           */
static uint8_t  voz_pendiente = 0;  /* hay un anuncio esperando a sonar     */
static uint32_t t_ultimo_cambio = 0;/* marca de tiempo del último giro      */

/* Estado del encoder y del pulsador */
static uint16_t enc_last_cnt = 0;
static uint8_t  sw_prev      = 1;

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
static void clip_Reproducir(const int16_t *datos, uint32_t largo);
static void anuncio_Programar(void);
static void ui_Draw(void);
static void print(const char *s);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    gpio_Init();
    usart2_Init();
    print("\r\n=== PRUEBA 5: ENCODER CON CONFIRMACION POR VOZ ===\r\n");

    tim10_Init();        print("  TIM10 OK\r\n");
    tim1_encoder_Init(); print("  Encoder OK\r\n");
    dma_Init();          print("  DMA OK\r\n");
    i2s2_Init();         print("  I2S OK\r\n");
    i2c1_Init();         print("  I2C OK\r\n");

    SSD1306_SetI2C(&hi2c1);
    SSD1306_Init();
    print("  OLED OK\r\n\r\n");

    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_buf, rx_buf, BUF_SAMPLES)
        != HAL_OK)
    {
        print("  [ERROR] No arranco el audio\r\n");
        trap_Error();
    }

    print("Gire el encoder (3 clics por cambio).\r\n");
    print("Pulse el mando para repetir el anuncio.\r\n\r\n");

    flag_refresh = 1;

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

/*
 * Reloj: HSI → PLL a 96 MHz y PLLI2S a 76.8 MHz (16 kHz exactos).
 * PLLI2SM es el divisor propio del PLLI2S en el F411.
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

/* PH1 = LED de estado, PA10 = pulsador del encoder, PA11 = SD del ampli */
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

    GPIO_InitStruct.Pin  = GPIO_PIN_10;          /* pulsador del encoder    */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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

/*
 * Encoder decodificado por hardware, sin interrupciones: el timer sigue los
 * flancos de ambos canales y la posición se lee por sondeo del contador.
 * El filtro de entrada al máximo absorbe los rebotes mecánicos.
 */
static void tim1_encoder_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct = {0};
    TIM_Encoder_InitTypeDef sEncoder        = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xFFFF;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoder.EncoderMode  = TIM_ENCODERMODE_TI12;
    sEncoder.IC1Polarity  = TIM_ICPOLARITY_FALLING;  /* sentido según el
                                                        cableado DT/CLK     */
    sEncoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoder.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoder.IC1Filter    = 0x0F;
    sEncoder.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sEncoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoder.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoder.IC2Filter    = 0x0F;
    if (HAL_TIM_Encoder_Init(&htim1, &sEncoder) != HAL_OK) { trap_Error(); }

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

/* Transmisión: Stream 4 canal 0. Recepción: Stream 3 canal 3 (I2S2ext). */
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
 * encoder_Poll — sondeo del contador del encoder.
 *
 * La diferencia con las versiones anteriores está en el divisor: ahora un
 * "paso" son ENC_COUNTS_PER_STEP cuentas (tres clics mecánicos) en lugar de
 * uno. Como la división entera descarta el resto y solo se consumen los
 * múltiplos completos, los clics sobrantes quedan acumulados en el contador
 * del timer y se suman al siguiente giro: no se pierde movimiento, solo se
 * exige más recorrido por cambio.
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

        /* Recorrido cíclico entre las tres opciones */
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
 * Pulsarlo repite el anuncio del valor actual, útil para comprobar el audio
 * sin tener que girar el mando.
 */
static void boton_Poll(void)
{
    uint8_t sw = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)
                 ? 0u : 1u;

    if (sw_prev == 1u && sw == 0u)      /* flanco de bajada = pulsado       */
    {
        if (fsm_state == FSM_IDLE) { fsm_state = FSM_PROC_BOTON; }
    }
    sw_prev = sw;
}

/*
 * clip_Reproducir — arranca un clip, cancelando el que estuviera sonando.
 *
 * El orden de las asignaciones importa: primero se baja el interruptor
 * 'clip_activo' para que el callback deje de leer, después se cambian el
 * puntero y la longitud, y solo al final se vuelve a subir. Así el callback
 * nunca puede encontrarse con un puntero nuevo y una longitud vieja, que le
 * haría leer fuera del arreglo.
 */
static void clip_Reproducir(const int16_t *datos, uint32_t largo)
{
    clip_activo = 0;                 /* detener lo que hubiera              */
    clip_datos  = datos;
    clip_largo  = largo;
    clip_pos    = 0;
    clip_activo = 1;                 /* arrancar                            */

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   /* ampli ON      */
}

/*
 * anuncio_Programar — arma la espera antes de hablar.
 * Se llama en cada cambio de opción. Al reescribir la marca de tiempo, cada
 * movimiento nuevo reinicia la cuenta, de modo que el anuncio solo sale
 * cuando el mando lleva VOZ_ESPERA_MS quieto.
 */
static void anuncio_Programar(void)
{
    voz_pendiente   = 1;
    t_ultimo_cambio = HAL_GetTick();
}

static void ui_Draw(void)
{
    char linea[24];

    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0, "TIEMPO ENTRE LETRAS");
    SSD1306_DrawHLine(0, 127, 10);

    /* El valor grande, que es lo que interesa leer de un vistazo */
    snprintf(linea, sizeof(linea), "%u", (unsigned)tiempos[opcion_idx]);
    SSD1306_WriteCharScaled(20, 20, linea[0], 4);
    SSD1306_WriteString(48, 34, "segundos");

    /* Indicador de las tres opciones, con la activa marcada */
    for (uint8_t i = 0; i < OPCIONES; i++)
    {
        uint8_t x = (uint8_t)(30 + i * 24);
        if (i == opcion_idx)
        {
            SSD1306_DrawFilledCircle(x, 56, 4);
        }
        else
        {
            SSD1306_DrawEmptyRect((uint8_t)(x - 2), 54, 4, 4);
        }
    }

    SSD1306_UpdateScreen();
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */
/*
 * El estado FSM_ANUNCIAR no se alcanza por una bandera sino por una
 * CONDICIÓN DE TIEMPO: hay un anuncio pendiente y han pasado VOZ_ESPERA_MS
 * desde el último movimiento. Es la forma de implementar "reproducir al
 * detenerse" sin bloquear el lazo ni gastar un temporizador.
 */
static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (voz_pendiente &&
            ((HAL_GetTick() - t_ultimo_cambio) >= VOZ_ESPERA_MS))
        {
            fsm_state = FSM_ANUNCIAR;
        }
        else if (flag_refresh)
        {
            fsm_state = FSM_REFRESH_OLED;
        }
        break;

    case FSM_PROC_ENCODER:
        anuncio_Programar();     /* la voz esperará a que el mando se pare  */
        flag_refresh = 1;        /* la pantalla sí se actualiza al instante */
        fsm_state = FSM_IDLE;
        break;

    case FSM_PROC_BOTON:
        anuncio_Programar();     /* repetir el anuncio del valor actual     */
        fsm_state = FSM_IDLE;
        break;

    case FSM_ANUNCIAR:
        voz_pendiente = 0;
        switch (opcion_idx)
        {
        case 0: clip_Reproducir(clip_uno,  clip_uno_largo);  break;
        case 1: clip_Reproducir(clip_dos,  clip_dos_largo);  break;
        default: clip_Reproducir(clip_tres, clip_tres_largo); break;
        }
        {
            char buf[40];
            snprintf(buf, sizeof(buf), "Anuncio: %u segundos\r\n",
                     (unsigned)tiempos[opcion_idx]);
            print(buf);
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
 *            ISR — CALLBACKS DE INTERRUPCIONES
 * ═══════════════════════════════════════════════════════════════════════════ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
    }
}

/*
 * audio_Rellenar — vuelca la mitad correspondiente del buffer de salida.
 *
 * Si hay un clip en curso, saca sus muestras una a una; si no, escribe
 * silencio. Al terminar el clip se apaga el amplificador por su pin SD, lo
 * que corta la etapa de salida y elimina el siseo de reposo entre anuncios.
 *
 * El recorrido va de 4 en 4 medias palabras (una trama) y la muestra se
 * escribe en los dos canales, alineada a la izquierda dentro de la ranura
 * de 32 bits.
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
                clip_activo = 0;                          /* fin del clip   */
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
            }
        }

        tx_buf[offset + i]      = (uint16_t)salida;
        tx_buf[offset + i + 1u] = 0;
        tx_buf[offset + i + 2u] = (uint16_t)salida;
        tx_buf[offset + i + 3u] = 0;
    }
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) { audio_Rellenar(0); }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) { audio_Rellenar(BUF_HALFWORDS / 2u); }
}
