/*
 ******************************************************************************
 * @file           : main.c  (PRUEBA 1 — amplificador I2S MAX98357A)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Generación de tono por I2S con DMA circular
 ******************************************************************************
 *
 * OBJETIVO DE ESTA PRUEBA
 * Validar de forma aislada la cadena de audio de salida: árbol de relojes de
 * audio (PLLI2S), periférico I2S en modo maestro transmisor, transferencia
 * por DMA circular y el amplificador MAX98357A con su parlante. Se reproduce
 * un tono senoidal continuo cuya frecuencia se cambia desde el puerto serial.
 *
 * El micrófono NO interviene todavía: el I2S se configura en modo SOLO
 * TRANSMISIÓN (simplex). Cuando se añada la captura se pasará a full-duplex,
 * que comparte los mismos relojes y añade la línea de datos de entrada.
 *
 * ─── Mapa de periféricos ────────────────────────────────────────────────────
 *  TIM10  → Blinky en PH1 (LED de la board) cada 250 ms          [CON IRQ]
 *  I2S2   → Maestro transmisor, 16 kHz, 16 bit, estéreo:
 *             CK  = PB13 (AF5) → BCLK del amplificador
 *             WS  = PB12 (AF5) → LRC  del amplificador
 *             SD  = PB15 (AF5) → DIN  del amplificador
 *  DMA1_S4→ Alimenta al I2S2 en modo circular                    [CON IRQ]
 *  GPIO   → PA11 → pin SD del amplificador (habilitación / mute)
 *  I2C1   → OLED SSD1306, SCL = PB8, SDA = PB9 (AF4)             [SIN IRQ]
 *  USART2 → 115200-8N1 por el VCP del ST-LINK                    [CON IRQ]
 *
 * ─── Comandos por puerto serial ─────────────────────────────────────────────
 *   '1' → 400 Hz      '3' →  800 Hz     'p' → reproducir
 *   '2' → 600 Hz      '4' → 1000 Hz     's' → silenciar
 *
 * ─── ADVERTENCIA DE HARDWARE ────────────────────────────────────────────────
 * La salida del MAX98357A es diferencial (BTL): el parlante va EXCLUSIVAMENTE
 * entre los terminales + y -. Ninguno de los dos puede ir a GND; hacerlo daña
 * la etapa de salida.
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════ Definiciones de la aplicación ═══════════════════ */

#define AUDIO_FS        16000u   /* frecuencia de muestreo, Hz              */

/*
 * Longitud del buffer de tono, en TRAMAS (una trama = una muestra izquierda
 * más una derecha). 320 tramas a 16 kHz son 20 ms de audio.
 *
 * El DMA recorre este buffer en modo circular, así que al llegar al final
 * vuelve al principio: para que el empalme no produzca un chasquido, el
 * buffer debe contener un número ENTERO de periodos de la senoidal. Con 20 ms
 * de buffer, eso se cumple para toda frecuencia múltiplo de 50 Hz, que es el
 * criterio con el que se eligieron las cuatro notas de prueba.
 */
#define TONE_FRAMES       320u
#define TONE_SAMPLES     (TONE_FRAMES * 2u)   /* dos canales por trama      */

/* Amplitud de la senoidal sobre el fondo de escala de 16 bits (32767).
 * Se deja en torno al 20 % para que la primera prueba no salga a todo
 * volumen: es más fácil detectar distorsión subiendo que bajando.          */
#define TONE_AMPLITUDE   6000

/* Notas disponibles, todas múltiplos de 50 Hz (ver nota sobre TONE_FRAMES) */
#define NOTE_COUNT          4u
static const uint16_t note_hz[NOTE_COUNT] = { 400u, 600u, 800u, 1000u };

/* Estados de la máquina de estados finitos */
typedef enum
{
    FSM_IDLE = 0,        /* sin eventos pendientes                          */
    FSM_PROC_UART,       /* llegó un carácter por el puerto serial          */
    FSM_REFRESH_OLED     /* redibujar la pantalla de estado                 */
} fsm_state_t;

/* ═══════════════════════ Handles de periféricos (globales) ════════════════ */
/* Globales porque stm32f4xx_it.c los referencia con 'extern' para pasarlos
 * a los HAL_xxx_IRQHandler() dentro de cada ISR.                            */

TIM_HandleTypeDef  htim10;        /* blinky de estado          */
I2S_HandleTypeDef  hi2s2;         /* audio de salida           */
DMA_HandleTypeDef  hdma_i2s2_tx;  /* DMA que alimenta al I2S2  */
I2C_HandleTypeDef  hi2c1;         /* pantalla OLED             */
UART_HandleTypeDef huart2;        /* puerto serial             */

/* ═══════════════════════ Variables de la aplicación ═══════════════════════ */

/* Banderas de evento: las escriben las ISR/callbacks, las consume la FSM.
 * 'volatile' es obligatorio porque cambian fuera del flujo normal.          */
static volatile uint8_t flag_rx      = 0;
static volatile uint8_t flag_refresh = 0;
static volatile uint8_t rx_byte      = 0;

/*
 * Buffer del tono. Es la memoria que el DMA lee directamente, así que su
 * contenido debe estar completo ANTES de arrancar la transferencia.
 * Formato: muestras de 16 bits con signo, intercaladas izquierda-derecha.
 */
static int16_t tone_buf[TONE_SAMPLES];

static uint8_t note_idx = 0;      /* nota seleccionada                      */
static uint8_t playing  = 0;      /* 1 = DMA activo y amplificador habilitado*/

static fsm_state_t fsm_state = FSM_IDLE;

/* ═══════════════════════ Prototipos ═══════════════════════════════════════ */

static void SystemClock_Config(void);
static void gpio_Init(void);        /* PH1 (LED) y PA11 (SD del amplificador)*/
static void tim10_Init(void);       /* blinky 250 ms, con interrupción       */
static void dma_Init(void);         /* DMA1 Stream 4 para el I2S2            */
static void i2s2_Init(void);        /* maestro transmisor, 16 kHz            */
static void i2c1_Init(void);        /* bus de la pantalla                    */
static void usart2_Init(void);      /* puerto serial por el VCP              */

static void fsm_Run(void);
static void trap_Error(void);
static void tone_Generate(uint16_t freq_hz);
static void audio_Play(void);
static void audio_Stop(void);
static void ui_Draw(void);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();             /* SysTick a 1 ms, caché de Flash, NVIC         */
    SystemClock_Config();   /* PLL a 96 MHz + PLLI2S a 76.8 MHz             */

    gpio_Init();
    tim10_Init();
    dma_Init();             /* el DMA se prepara ANTES que el I2S           */
    i2s2_Init();
    i2c1_Init();
    usart2_Init();

    SSD1306_SetI2C(&hi2c1);
    SSD1306_Init();

    /* Preparar el primer tono y arrancar la reproducción */
    tone_Generate(note_hz[note_idx]);
    audio_Play();

    flag_refresh = 1;       /* pintar la pantalla de estado inicial         */

    {
        const char hello[] =
            "\r\n== Prueba de audio I2S (MAX98357A) ==\r\n"
            "1:400Hz  2:600Hz  3:800Hz  4:1000Hz   p:play  s:stop\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)hello,
                          (uint16_t)strlen(hello), 200);
    }

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);

    /* El lazo no bloquea: el tono lo mantiene el DMA por hardware, sin
     * intervención del CPU. Aquí solo se despachan los eventos.            */
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
 * Este proyecto necesita DOS lazos de enganche de fase:
 *
 *   PLL principal → SYSCLK del sistema
 *   PLLI2S        → reloj exclusivo del periférico de audio
 *
 * Ambos comparten la fuente (HSI) y el divisor de entrada PLLM, que es un
 * campo único del registro RCC_PLLCFGR. Por eso PLLM debe elegirse pensando
 * en los dos a la vez, y por eso el PLL principal tiene que estar
 * configurado aunque solo interesara el audio.
 *
 *   VCO_in = HSI / PLLM = 16 MHz / 8 = 2 MHz      (recomendado: 1-2 MHz)
 *
 *   PLL:     VCO = 2 MHz x 96  = 192 MHz  →  SYSCLK = 192/2 = 96 MHz
 *   PLLI2S:  VCO = 2 MHz x 192 = 384 MHz  →  I2SCLK = 384/5 = 76.8 MHz
 *
 * La elección de 76.8 MHz no es arbitraria: es el valor que permite obtener
 * 16 kHz EXACTOS con los divisores enteros del periférico I2S. Para datos de
 * 16 bits, el periférico divide entre 32 x (2 x I2SDIV + ODD):
 *
 *   76 800 000 / (32 x 150) = 16 000,00 Hz   →  error 0,0000 %
 *
 * Un reloj de audio inexacto no impide que suene, pero desplaza el tono y,
 * más adelante con el micrófono, desalinea la frecuencia de muestreo real
 * respecto a la nominal.
 *
 * Divisores de bus a 96 MHz (respetando los máximos del F411):
 *   AHB  = 96 MHz     APB1 = 48 MHz (máx. 50)     APB2 = 96 MHz (máx. 100)
 * Por encima de 90 MHz la Flash necesita 3 estados de espera y el regulador
 * debe estar en Scale 1.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    /* Regulador en Scale 1: obligatorio para superar los 84 MHz            */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSI como fuente y PLL principal a 96 MHz                             */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;    /* compartido con PLLI2S  */
    RCC_OscInitStruct.PLL.PLLN            = 96;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;    /* rama USB/SDIO, sin uso */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { trap_Error(); }

    /* El PLL pasa a ser la fuente del sistema                              */
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

    /* PLLI2S: reloj dedicado del periférico de audio                       */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInit.PLLI2S.PLLI2SM       = 8;
    PeriphClkInit.PLLI2S.PLLI2SN       = 192;   /* VCO  = 2 MHz x 192       */
    PeriphClkInit.PLLI2S.PLLI2SR       = 5;     /* I2SCLK = 384 MHz / 5     */
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════ GPIO: LED de estado y control del amplificador ═══════ */
/*
 * PH1  = LED integrado de la board. PH0/PH1 son los pines del oscilador
 *        externo de alta frecuencia; como el sistema corre con HSI + PLL y
 *        no hay cristal, quedan libres como GPIO.
 *
 * PA11 = pin SD del MAX98357A. Ese pin no es una entrada lógica normal: el
 *        amplificador mide el VOLTAJE aplicado y con él decide cuatro
 *        estados distintos (apagado, canal derecho, mezcla, canal
 *        izquierdo). Llevarlo a nivel bajo lo apaga; llevarlo a nivel alto
 *        desde un GPIO de 3.3 V lo pone en canal izquierdo, que es un
 *        estado determinista y no depende del divisor resistivo que traiga
 *        el módulo de fábrica. Sirve además como silenciador por software.
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

    GPIO_InitStruct.Pin   = GPIO_PIN_11;         /* PA11: SD del ampli      */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Arrancar con el amplificador apagado: así no suena nada hasta que el
     * I2S esté configurado y emitiendo datos válidos, evitando el chasquido
     * típico del arranque.                                                 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

/* ═══════════════════════ TIM10: blinky (250 ms, IRQ) ══════════════════════ */
/*
 * Cadena de reloj:  PLL (96 MHz) → APB2 (96 MHz) → reloj de TIM10 (96 MHz)
 *
 * PSC = 9599 → tick = 96 MHz / (9599+1) = 10 kHz  (100 us por cuenta)
 * ARR = 2499 → update = (2499+1) x 100 us = 250 ms
 *
 * A 96 MHz no se puede usar el prescaler de 15999 de la versión anterior
 * (que daba 1 ms por cuenta a 16 MHz): haría falta PSC = 95999, valor que
 * no cabe en los 16 bits del registro. Por eso se baja la resolución del
 * tick a 100 us y se sube el ARR.
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

/* ═══════════════════════ DMA1 Stream 4: alimentación del I2S2 ═════════════ */
/*
 * A 16 kHz el periférico pide una muestra cada 62 microsegundos. Atender eso
 * por interrupción significaría entrar y salir de una ISR 32 000 veces por
 * segundo (dos canales), consumiendo el CPU que después harán falta para los
 * servos, la pantalla y el resto del sistema. El DMA mueve las muestras de
 * la RAM al periférico POR HARDWARE, sin intervención del procesador.
 *
 * MODO CIRCULAR: al terminar de recorrer el buffer, el DMA vuelve solo al
 * principio y sigue. Sin esto habría que rearmar la transferencia cada vez
 * y en cada hueco se oiría un chasquido. Para un tono continuo es la única
 * opción sensata.
 *
 * Correspondencia fija del hardware: la petición de transmisión de SPI2/I2S2
 * está cableada al Stream 4, Canal 0 del DMA1. No es elegible: cada
 * periférico tiene sus streams asignados en el mapa del DMA.
 *
 * Anchura de datos media palabra (16 bits) en ambos extremos, porque el
 * formato de audio configurado es de 16 bits y así cada transferencia mueve
 * exactamente una muestra.
 */
static void dma_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_i2s2_tx.Instance                 = DMA1_Stream4;
    hdma_i2s2_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_i2s2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2s2_tx.Init.PeriphInc           = DMA_PINC_DISABLE; /* registro fijo */
    hdma_i2s2_tx.Init.MemInc              = DMA_MINC_ENABLE;  /* recorre buffer*/
    hdma_i2s2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2s2_tx) != HAL_OK) { trap_Error(); }

    /* Enlazar el DMA con el handle del I2S: a partir de aquí, las funciones
     * HAL_I2S_..._DMA() saben qué stream deben usar.                       */
    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_i2s2_tx);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

/* ═══════════════════════ I2S2: maestro transmisor ═════════════════════════ */
/*
 * El STM32 debe ser MAESTRO obligatoriamente: ni el MAX98357A ni el INMP441
 * pueden generar los relojes del bus, ambos son siempre esclavos. El maestro
 * genera BCLK (reloj de bit) y WS (word select, cuya frecuencia es la
 * frecuencia de muestreo).
 *
 * Estándar Philips: el dato va desplazado un ciclo de BCLK respecto al
 * flanco de WS, MSB primero. Es el que espera el MAX98357A.
 *
 * MCLK deshabilitado: este amplificador reconstruye su temporizado a partir
 * de BCLK mediante un PLL interno y no necesita reloj maestro. Eso ahorra un
 * pin y simplifica el cableado frente a los códecs de audio tradicionales.
 *
 * Pines (todos en AF5, la función alternada de SPI2/I2S2):
 *   PB13 = CK  → BCLK      PB12 = WS → LRC      PB15 = SD → DIN
 */
static void i2s2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  /* flancos limpios */
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_SPI2_CLK_ENABLE();

    hi2s2.Instance          = SPI2;
    hi2s2.Init.Mode         = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard     = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat   = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq    = I2S_AUDIOFREQ_16K;
    hi2s2.Init.CPOL         = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource  = I2S_CLOCK_PLL;   /* toma el reloj del PLLI2S  */
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;  /* solo Tx     */

    /* HAL_I2S_Init calcula I2SDIV y ODD a partir de AudioFreq y del reloj
     * real del PLLI2S; con 76.8 MHz el resultado es exacto.                */
    if (HAL_I2S_Init(&hi2s2) != HAL_OK) { trap_Error(); }
}

/* ═══════════════════════ I2C1: bus de la pantalla ═════════════════════════ */
/*
 * SCL = PB8, SDA = PB9 en AF4 y en drenador abierto, como exige el bus I2C.
 * Sin interrupciones: las transferencias se hacen por sondeo.
 */
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

/* ═══════════════════════ USART2: puerto serial por el VCP ═════════════════ */
/*
 * PA2/PA3 (AF7) están cableados al Virtual COM Port del ST-LINK, así que el
 * serial viaja por el mismo cable USB de programación. USART2 cuelga de APB1
 * (48 MHz con esta configuración); el HAL calcula el divisor de baudios con
 * ese reloj, así que el cambio de frecuencia del sistema no obliga a tocar
 * nada aquí.
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

static void trap_Error(void)
{
    while (1)
    {
        __NOP();
    }
}

/*
 * tone_Generate — rellena el buffer con una senoidal de la frecuencia pedida.
 *
 * La fase de la muestra i es  2*pi*f*i/Fs : avanza una vuelta completa cada
 * Fs/f muestras. El valor se escala a la amplitud elegida y se convierte a
 * entero de 16 bits con signo, que es el formato que espera el periférico.
 *
 * La misma muestra se escribe en los DOS canales. Es deliberado: así el tono
 * se oye independientemente de qué canal haya seleccionado el pin SD del
 * amplificador, lo que elimina una variable en esta primera prueba.
 *
 * Se usa coma flotante solo aquí, una vez por cambio de nota y fuera de
 * cualquier ruta crítica; el DMA después reproduce el buffer indefinidamente
 * sin volver a calcular nada.
 */
static void tone_Generate(uint16_t freq_hz)
{
    for (uint16_t i = 0; i < TONE_FRAMES; i++)
    {
        float   phase  = 2.0f * 3.14159265f * (float)freq_hz *
                         (float)i / (float)AUDIO_FS;
        int16_t sample = (int16_t)((float)TONE_AMPLITUDE * sinf(phase));

        tone_buf[2u * i]      = sample;   /* canal izquierdo                */
        tone_buf[2u * i + 1u] = sample;   /* canal derecho (mismo dato)     */
    }
}

/*
 * audio_Play — arranca la reproducción.
 * Orden importante: primero se lanza el DMA para que el bus ya lleve datos
 * válidos, y solo después se habilita el amplificador. Al revés se oiría un
 * chasquido mientras la línea de datos aún está indefinida.
 *
 * El tamaño que recibe HAL_I2S_Transmit_DMA se cuenta en DATOS de 16 bits,
 * no en tramas: por eso se pasan TONE_SAMPLES (tramas x 2 canales).
 */
static void audio_Play(void)
{
    if (HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)tone_buf, TONE_SAMPLES)
        != HAL_OK)
    {
        trap_Error();
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   /* ampli ON      */
    playing = 1;
}

/*
 * audio_Stop — detiene la reproducción.
 * Orden inverso al de arranque: primero se silencia el amplificador y
 * después se detiene el DMA, para no dejar la línea de datos congelada en
 * un valor arbitrario mientras el amplificador aún está activo.
 */
static void audio_Stop(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); /* ampli OFF     */
    HAL_I2S_DMAStop(&hi2s2);
    playing = 0;
}

/*
 * ui_Draw — pantalla de estado de la prueba.
 */
static void ui_Draw(void)
{
    char linea[24];

    SSD1306_Fill(0);
    SSD1306_WriteString(0, 0, "PRUEBA AUDIO I2S");
    SSD1306_DrawHLine(0, 127, 10);

    snprintf(linea, sizeof(linea), "Nota: %u Hz",
             (unsigned)note_hz[note_idx]);
    SSD1306_WriteString(0, 18, linea);

    snprintf(linea, sizeof(linea), "Estado: %s",
             playing ? "SONANDO" : "SILENCIO");
    SSD1306_WriteString(0, 30, linea);

    SSD1306_WriteString(0, 44, "Fs = 16000 Hz");
    SSD1306_WriteString(0, 54, "1-4 nota  p/s play");

    SSD1306_UpdateScreen();
}

/* ═══════════════════════ Máquina de estados (FSM) ═════════════════════════ */
/*
 * FSM dirigida por eventos. El audio en sí no pasa por aquí: una vez
 * arrancado, el DMA lo sostiene por hardware. La FSM solo atiende los
 * comandos del usuario y el refresco de la pantalla.
 */
static void fsm_Run(void)
{
    switch (fsm_state)
    {
    case FSM_IDLE:
        if (flag_rx)           { fsm_state = FSM_PROC_UART;    }
        else if (flag_refresh) { fsm_state = FSM_REFRESH_OLED; }
        break;

    case FSM_PROC_UART:
        flag_rx = 0;                        /* consumir el evento           */
        switch (rx_byte)
        {
        case '1': case '2': case '3': case '4':
            note_idx = (uint8_t)(rx_byte - '1');
            /* Para cambiar de nota se detiene el DMA, se regenera el buffer
             * y se vuelve a arrancar: reescribir el buffer mientras el DMA
             * lo está leyendo produciría un salto audible a mitad de onda. */
            {
                uint8_t was_playing = playing;
                audio_Stop();
                tone_Generate(note_hz[note_idx]);
                if (was_playing) { audio_Play(); }
            }
            flag_refresh = 1;
            break;

        case 'p':                           /* reproducir                   */
            if (!playing) { audio_Play(); flag_refresh = 1; }
            break;

        case 's':                           /* silenciar                    */
            if (playing) { audio_Stop(); flag_refresh = 1; }
            break;

        default:                            /* carácter no reconocido       */
            break;
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
 * La llama HAL_TIM_IRQHandler() en cada update de TIM10 (250 ms). El LED
 * parpadeando es la señal de que el sistema sigue vivo: si el audio se
 * cuelga pero el LED sigue, el problema está en la cadena de audio y no en
 * el firmware completo.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
    }
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

/*
 * Callbacks del I2S en modo DMA.
 *
 * En modo circular el DMA avisa dos veces por vuelta: al llegar a la mitad
 * del buffer (HalfCplt) y al llegar al final (Cplt). Ese es el mecanismo de
 * DOBLE BUFFER: permite reescribir la mitad que el DMA ya dejó atrás
 * mientras sigue leyendo la otra, sin competir por la misma memoria.
 *
 * En esta prueba el tono es fijo y el buffer no se reescribe, así que los
 * callbacks quedan vacíos. Se dejan declarados porque son el punto de
 * enganche donde irá la reproducción de audio real (los mensajes de voz
 * pregrabados) más adelante.
 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;   /* la primera mitad del buffer quedó libre para reescribir */
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    (void)hi2s;   /* la segunda mitad quedó libre; el DMA vuelve al inicio   */
}
