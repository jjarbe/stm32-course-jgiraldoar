/*
 ******************************************************************************
 * @file           : main.c   (PRUEBA 2 — micrófono I2S INMP441)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Captura de audio por I2S con DMA y medidor de nivel
 ******************************************************************************
 *
 * OBJETIVO DE ESTA PRUEBA
 * Validar de forma aislada la cadena de audio de ENTRADA: el micrófono
 * INMP441, el periférico I2S en modo maestro receptor y la captura continua
 * por DMA circular. El nivel de sonido captado se muestra como una barra en
 * la pantalla OLED y se reporta por el puerto serial, de modo que la prueba
 * se verifica hablando o aplaudiendo frente al micrófono, sin osciloscopio.
 *
 * El amplificador NO interviene: se mantiene silenciado por su pin SD. La
 * transmisión y la recepción se combinarán en la prueba siguiente
 * (full-duplex).
 *
 * ─── CABLEADO PARA ESTA PRUEBA ──────────────────────────────────────────────
 * En modo maestro receptor SIMPLEX los datos entran por el pin principal de
 * datos del periférico, que es PB15. El pin PB14 (I2S2ext) solo entra en
 * juego en full-duplex. Por tanto, temporalmente:
 *
 *   INMP441 SCK  → PB13        INMP441 VDD  → 3V3  (NUNCA 5 V)
 *   INMP441 WS   → PB12        INMP441 GND  → GND
 *   INMP441 SD   → PB15        INMP441 L/R  → GND (canal izquierdo)
 *
 * En la prueba de full-duplex, el SD del micrófono se moverá a PB14 y ese
 * será ya el cableado definitivo.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  TIM10  → Blinky en PH1 cada 250 ms                            [CON IRQ]
 *  I2S2   → Maestro receptor, 16 kHz, 32 bit, estéreo            [SIN IRQ]
 *  DMA1_S3→ Recibe del I2S2 en modo circular                     [CON IRQ]
 *  GPIO   → PA11 → SD del amplificador, en bajo (silenciado)
 *  I2C1   → OLED SSD1306, SCL = PB8, SDA = PB9 (AF4), 100 kHz    [SIN IRQ]
 *  USART2 → 115200-8N1 por el VCP del ST-LINK                    [CON IRQ]
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

#define AUDIO_FS        16000u   /* frecuencia de muestreo, Hz              */

/*
 * Buffer de captura.
 *
 * FORMATO DE LOS DATOS: el periférico está en formato de 32 bits, así que
 * cada muestra ocupa DOS medias palabras de 16 bits en memoria: primero los
 * 16 bits altos y después los bajos. Y como la trama es estéreo, cada
 * "trama" son dos muestras (izquierda y derecha). En total:
 *
 *   una trama = 4 medias palabras:  [L_alto][L_bajo][R_alto][R_bajo]
 *
 * El INMP441 con su pin L/R a GND transmite en la ranura IZQUIERDA, así que
 * el dato útil está en la primera media palabra de cada trama.
 *
 * 256 tramas a 16 kHz son 16 ms de audio, y el DMA avisa cada 8 ms (en la
 * mitad y al final del buffer).
 */
#define CAP_FRAMES        256u
#define CAP_HALFWORDS    (CAP_FRAMES * 4u)   /* 4 medias palabras por trama */
#define CAP_SAMPLES      (CAP_FRAMES * 2u)   /* "datos" de 32 bits para HAL */

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,        /* sin eventos pendientes                          */
    FSM_PROC_AUDIO,      /* media captura lista → calcular nivel            */
    FSM_REFRESH_OLED     /* redibujar el medidor                            */
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */

TIM_HandleTypeDef  htim10;
I2S_HandleTypeDef  hi2s2;
DMA_HandleTypeDef  hdma_i2s2_rx;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

/* Banderas de evento: las escriben las ISR/callbacks, las consume la FSM */
static volatile uint8_t flag_audio   = 0;   /* hay media captura lista      */
static volatile uint8_t flag_refresh = 0;
static volatile uint8_t half_listo   = 0;   /* 0 = primera mitad, 1 = segunda*/

/* Buffer que el DMA rellena directamente */
static uint16_t cap_buf[CAP_HALFWORDS];

/* Resultados de la medición */
static volatile uint16_t nivel_pico = 0;    /* pico de la última media      */
static uint16_t nivel_max_visto = 0;        /* máximo histórico, se retiene */

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
static uint16_t nivel_Calcular(const uint16_t *p, uint16_t n_halfwords);
static void ui_Draw(void);
static void print(const char *s);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    gpio_Init();
    usart2_Init();          /* primero el serial, para poder informar       */
    print("\r\n=== PRUEBA 2: MICROFONO I2S (INMP441) ===\r\n");

    tim10_Init();           print("  TIM10 OK\r\n");
    dma_Init();             print("  DMA OK\r\n");
    i2s2_Init();            print("  I2S (maestro receptor) OK\r\n");
    i2c1_Init();            print("  I2C OK\r\n");

    SSD1306_SetI2C(&hi2c1);
    SSD1306_Init();
    print("  OLED OK\r\n\r\n");

    /* Arrancar la captura continua. A partir de aquí el DMA rellena el
     * buffer sin intervención del procesador y avisa dos veces por vuelta. */
    if (HAL_I2S_Receive_DMA(&hi2s2, cap_buf, CAP_SAMPLES) != HAL_OK)
    {
        print("  [ERROR] No arranco la captura\r\n");
        trap_Error();
    }
    print("Capturando. Hable o aplauda frente al microfono.\r\n\r\n");

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
 * Idéntico al de la prueba del amplificador, porque la cadena de audio de
 * entrada y la de salida comparten el mismo reloj:
 *
 *   VCO_in = HSI / PLLM = 16 MHz / 8 = 2 MHz
 *   PLL:     VCO = 2 x 96  = 192 MHz  →  SYSCLK = 96 MHz
 *   PLLI2S:  VCO = 2 x 192 = 384 MHz  →  I2SCLK = 384/5 = 76.8 MHz
 *
 * IMPORTANTE — PLLI2SM: en el STM32F411 el PLLI2S tiene su PROPIO divisor
 * de entrada, independiente del PLLM del PLL principal. Dejarlo sin asignar
 * (queda en 0 por la inicialización de la estructura) impide que el PLL de
 * audio enganche, y el fallo se manifiesta después, al inicializar el I2S.
 *
 * Con 76.8 MHz y formato de 32 bits, el periférico divide entre
 * 64 x (2 x I2SDIV + ODD) = 64 x 75 = 4800, dando 16 000,00 Hz exactos.
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
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK = 96 MHz  */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* APB1 = 48 MHz  */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2 = 96 MHz  */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        trap_Error();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInit.PLLI2S.PLLI2SM       = 8;     /* divisor propio del F411  */
    PeriphClkInit.PLLI2S.PLLI2SN       = 192;
    PeriphClkInit.PLLI2S.PLLI2SR       = 5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════════ GPIO: LED y silenciado del amplificador ══════════ */
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

    GPIO_InitStruct.Pin   = GPIO_PIN_11;         /* PA11: SD del ampli      */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Amplificador silenciado durante toda esta prueba: su entrada de datos
     * comparte nodo con la línea del micrófono y no debe reproducir nada.  */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

/* ═══════════════════════ TIM10: blinky (250 ms, IRQ) ══════════════════════ */
/* APB2 = 96 MHz;  PSC = 9599 → 10 kHz;  ARR = 2499 → 250 ms                */
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

/* ═══════════════════════ DMA1 Stream 3: recepción del I2S2 ════════════════ */
/*
 * A 16 kHz llega una muestra cada 62 microsegundos por canal. Atender eso
 * por interrupción consumiría el procesador; el DMA traslada las muestras
 * del periférico a la RAM por hardware.
 *
 * MODO CIRCULAR con DOBLE BUFFER: al llegar al final del buffer el DMA
 * vuelve solo al principio, y avisa DOS veces por vuelta — a la mitad
 * (HalfCplt) y al final (Cplt). Eso permite procesar la mitad que el DMA ya
 * dejó atrás mientras sigue llenando la otra, sin competir nunca por la
 * misma memoria. Sin ese mecanismo se leerían muestras a medio escribir.
 *
 * Correspondencia fija del hardware: la petición de RECEPCIÓN de SPI2/I2S2
 * está cableada al Stream 3, Canal 0 del DMA1 (la de transmisión, que usó
 * la prueba anterior, es el Stream 4). No es elegible.
 */
static void dma_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_i2s2_rx.Instance                 = DMA1_Stream3;
    hdma_i2s2_rx.Init.Channel             = DMA_CHANNEL_0;
    hdma_i2s2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2s2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2s2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2s2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s2_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2s2_rx) != HAL_OK) { trap_Error(); }

    /* Enlazar el stream con el handle del I2S, campo de RECEPCIÓN */
    __HAL_LINKDMA(&hi2s2, hdmarx, hdma_i2s2_rx);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

/* ═══════════════════════ I2S2: maestro receptor ═══════════════════════════ */
/*
 * El STM32 es MAESTRO también en recepción: el INMP441 es siempre esclavo y
 * no puede generar los relojes. El micrófono se limita a poner sus datos en
 * la línea al ritmo del BCLK y el WS que recibe.
 *
 * FORMATO DE 32 BITS: el INMP441 entrega muestras de 24 bits alineadas a la
 * izquierda dentro de ranuras de 32 bits. Configurando el periférico a 32
 * bits, cada ranura se recibe completa y los 16 bits altos de cada muestra
 * (que es lo que se usa para medir el nivel) quedan directamente en la
 * primera media palabra, sin necesidad de recomponer nada.
 *
 * BCLK = 16 000 x 64 = 1,024 MHz, y WS = 16 kHz.
 *
 * MCLK deshabilitado: el INMP441 deriva todo su temporizado del BCLK.
 *
 * Pines en esta prueba (modo simplex): CK = PB13, WS = PB12, SD = PB15.
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

    __HAL_RCC_SPI2_CLK_ENABLE();

    hi2s2.Instance            = SPI2;
    hi2s2.Init.Mode           = I2S_MODE_MASTER_RX;
    hi2s2.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat     = I2S_DATAFORMAT_32B;
    hi2s2.Init.MCLKOutput     = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq      = I2S_AUDIOFREQ_16K;
    hi2s2.Init.CPOL           = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════════ I2C1: bus de la pantalla ═════════════════════════ */
static void i2c1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Desatascar el bus antes de tomar los pines: recupera el sistema de un
     * reset ocurrido en mitad de una transferencia.                        */
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

/* ═══════════════════════ USART2 por el VCP ════════════════════════════════ */
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
    while (1)
    {
        __NOP();
    }
}

static void print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 500);
}

/*
 * nivel_Calcular — pico absoluto de una mitad del buffer.
 *
 * Recorrido del buffer: cada trama ocupa 4 medias palabras
 * [L_alto][L_bajo][R_alto][R_bajo], y el dato útil está en la primera
 * (canal izquierdo, 16 bits altos de la muestra de 24 bits del micrófono).
 * Por eso el bucle avanza de 4 en 4.
 *
 * Se mide el PICO y no el valor medio porque interesa detectar sonidos
 * breves (una palmada, el inicio de una palabra), que un promedio diluiría.
 * El valor absoluto se toma con una comparación en lugar de abs() para no
 * arrastrar dependencias en una función que se ejecuta muy a menudo.
 */
static uint16_t nivel_Calcular(const uint16_t *p, uint16_t n_halfwords)
{
    uint32_t pico = 0;

    for (uint16_t i = 0; i < n_halfwords; i += 4u)
    {
        int16_t  s   = (int16_t)p[i];      /* muestra del canal izquierdo   */
        uint32_t mag = (s < 0) ? (uint32_t)(-(int32_t)s) : (uint32_t)s;

        if (mag > pico) { pico = mag; }
    }
    return (uint16_t)pico;
}

/*
 * ui_Draw — medidor de nivel en la pantalla.
 * La barra se escala dividiendo el pico (0..32767) entre 256, lo que da un
 * recorrido de 0..127 píxeles que coincide con el ancho del panel.
 */
static void ui_Draw(void)
{
    char     linea[24];
    uint16_t pico  = nivel_pico;
    uint8_t  barra = (uint8_t)(pico / 256u);        /* 0..127 píxeles       */
    uint8_t  barra_max = (uint8_t)(nivel_max_visto / 256u);

    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0, "MICROFONO I2S");
    SSD1306_DrawHLine(0, 127, 10);

    /* Valor numérico del pico actual */
    snprintf(linea, sizeof(linea), "Pico: %5u", (unsigned)pico);
    SSD1306_WriteString(0, 16, linea);

    /* Barra de nivel dentro de un marco fijo */
    SSD1306_DrawEmptyRect(0, 28, 126, 12);
    for (uint8_t x = 0; x < barra && x < 126u; x++)
    {
        for (uint8_t y = 30; y < 38u; y++)
        {
            SSD1306_DrawPixel((uint8_t)(x + 1u), y, 1);
        }
    }

    /* Marca del máximo histórico: referencia para comparar de un vistazo */
    if (barra_max > 0u && barra_max < 126u)
    {
        for (uint8_t y = 28; y <= 40u; y++)
        {
            SSD1306_DrawPixel((uint8_t)(barra_max + 1u), y, 1);
        }
    }

    snprintf(linea, sizeof(linea), "Max: %5u", (unsigned)nivel_max_visto);
    SSD1306_WriteString(0, 46, linea);
    SSD1306_WriteString(0, 56, "Hable o aplauda");

    SSD1306_UpdateScreen();
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */
/*
 * FSM dirigida por eventos. La captura la sostiene el DMA por hardware; la
 * FSM solo procesa las mitades del buffer que quedan disponibles y refresca
 * la pantalla, que es la operación costosa y por eso va a menor ritmo.
 */
static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (flag_audio)        { fsm_state = FSM_PROC_AUDIO;   }
        else if (flag_refresh) { fsm_state = FSM_REFRESH_OLED; }
        break;

    case FSM_PROC_AUDIO:
        flag_audio = 0;                     /* consumir el evento           */
        {
            /* Procesar la mitad que el DMA acaba de dejar libre: si avisó
             * de media transferencia, la primera; si de transferencia
             * completa, la segunda.                                        */
            const uint16_t *mitad = (half_listo == 0u)
                                    ? &cap_buf[0]
                                    : &cap_buf[CAP_HALFWORDS / 2u];

            nivel_pico = nivel_Calcular(mitad, CAP_HALFWORDS / 2u);

            if (nivel_pico > nivel_max_visto)
            {
                nivel_max_visto = nivel_pico;
            }
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
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * HAL_TIM_PeriodElapsedCallback
 * Cada 250 ms conmuta el LED y, cada dos ticks (500 ms), pide refrescar la
 * pantalla y reporta el nivel por el puerto serial. El refresco va a ese
 * ritmo y no al del audio porque volcar la pantalla por I2C tarda unos 100 ms
 * y no tiene sentido hacerlo 125 veces por segundo.
 */
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
 * Callbacks de la captura por DMA.
 *
 * Este es el mecanismo de doble buffer en funcionamiento: el DMA avisa a la
 * mitad del recorrido y al final, de modo que siempre hay una mitad estable
 * para procesar mientras la otra se está llenando. Los callbacks solo
 * anotan cuál mitad quedó lista y levantan la bandera; el cálculo del nivel
 * lo hace la FSM en el lazo principal, para no alargar la interrupción.
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)
    {
        half_listo = 0;      /* la PRIMERA mitad ya está completa           */
        flag_audio = 1;
    }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2)
    {
        half_listo = 1;      /* la SEGUNDA mitad ya está completa           */
        flag_audio = 1;
    }
}
