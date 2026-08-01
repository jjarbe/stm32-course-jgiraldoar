/*
 ******************************************************************************
 * @file           : main.c   (DIAGNÓSTICO 2 — encendido del panel OLED)
 * @author         : Juan Jose Giraldo Arbelaez
 * @brief          : Inicialización del SSD1306 con verificación por comando
 ******************************************************************************
 *
 * OBJETIVO
 * El barrido anterior confirmó que el controlador responde en 0x78, así que
 * el bus, el cableado y la dirección están descartados. Esta prueba avanza
 * al siguiente eslabón: comprobar que la SECUENCIA DE INICIALIZACIÓN llega
 * y que el panel es capaz de emitir luz.
 *
 * Se diferencia del driver normal en tres cosas:
 *   1. Verifica el valor de retorno de CADA escritura I2C y lo reporta.
 *   2. Enciende TODOS los píxeles (patrón 0xFF). Es la prueba visual más
 *      inequívoca: si una sola zona del panel ilumina, el panel vive.
 *   3. Usa el comando 0xA5 (Entire Display ON), que enciende el panel
 *      ignorando por completo el contenido de la memoria de vídeo. Así se
 *      separa el problema "no llega la imagen" del problema "el panel no
 *      ilumina".
 *
 * Reloj a 16 MHz e I2C a 100 kHz: la misma configuración con la que el
 * barrido funcionó, para no introducir variables nuevas.
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define OLED_ADDR   0x78

TIM_HandleTypeDef  htim10;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

static uint16_t errores = 0;   /* comandos que no llegaron al controlador  */

static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim10_Init(void);
static void i2c1_Init(void);
static void usart2_Init(void);
static void print(const char *s);
static uint8_t oled_Cmd(uint8_t cmd, const char *nombre);
static void oled_InitVerificado(void);
static void oled_LlenarPantalla(uint8_t patron);

/* ═══════════════════════ Programa principal ═══════════════════════════════ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    gpio_Init();
    tim10_Init();
    usart2_Init();
    i2c1_Init();

    print("\r\n=== DIAGNOSTICO 2: ENCENDIDO DEL PANEL ===\r\n\r\n");

    /* Paso 1: confirmar que el controlador sigue respondiendo */
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR, 3, 20) == HAL_OK)
    {
        print("[OK]    El controlador responde en 0x78\r\n\r\n");
    }
    else
    {
        print("[ERROR] El controlador NO responde. Revisar cableado.\r\n");
        while (1) { }
    }

    /* Paso 2: secuencia de inicialización, comando por comando */
    print("Enviando secuencia de inicializacion:\r\n");
    oled_InitVerificado();

    /* Paso 3: prueba con la memoria de vídeo en blanco.
     * El comando 0xA5 fuerza TODOS los píxeles encendidos sin importar lo
     * que haya en memoria: aísla la capacidad del panel de iluminar.       */
    print("\r\n--- PRUEBA A: forzar todos los pixeles (comando 0xA5) ---\r\n");
    print("La pantalla deberia ponerse COMPLETAMENTE BLANCA ahora.\r\n");
    oled_Cmd(0xA5, "Entire Display ON");
    HAL_Delay(4000);

    /* Paso 4: volver al modo normal y escribir datos reales */
    print("\r\n--- PRUEBA B: escribir memoria de video ---\r\n");
    oled_Cmd(0xA4, "Volver a modo normal");
    print("Llenando la memoria con 0xFF (todos los pixeles)...\r\n");
    oled_LlenarPantalla(0xFF);
    print("La pantalla deberia estar BLANCA otra vez.\r\n");
    HAL_Delay(4000);

    print("\r\n--- PRUEBA C: patron de rayas ---\r\n");
    oled_LlenarPantalla(0x55);   /* rayas horizontales alternas             */
    print("La pantalla deberia mostrar RAYAS.\r\n");

    /* Resumen */
    {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "\r\n=== RESUMEN: %u errores de comunicacion ===\r\n",
                 (unsigned)errores);
        print(buf);
    }

    if (errores == 0u)
    {
        print("Todos los comandos llegaron al controlador.\r\n");
        print("Si la pantalla sigue apagada, el problema esta en el\r\n");
        print("panel o en su alimentacion, no en la comunicacion.\r\n");
    }

    while (1) { }
}

/* ═══════════════════════ Funciones del panel ══════════════════════════════ */

/*
 * oled_Cmd — envía un comando y reporta si el controlador lo aceptó.
 * Escribir en el registro 0x00 pone el bit D/C# a 0, indicando comando.
 * A diferencia del driver normal, aquí SÍ se comprueba el retorno: un
 * comando perdido es exactamente lo que dejaría el panel apagado sin dar
 * ninguna otra señal de error.
 */
static uint8_t oled_Cmd(uint8_t cmd, const char *nombre)
{
    char buf[64];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x00, 1,
                                             &cmd, 1, 100);
    if (st != HAL_OK)
    {
        errores++;
        snprintf(buf, sizeof(buf), "  [FALLO] 0x%02X %s\r\n",
                 (unsigned)cmd, nombre);
        print(buf);
        return 0;
    }
    return 1;
}

/*
 * oled_InitVerificado — la misma secuencia del driver, con reporte.
 * El comando crítico es el par 0x8D/0x14: activa la BOMBA DE CARGA interna,
 * que es la que genera los ~7 V que necesita el panel OLED a partir del
 * riel lógico de 3.3 V. Sin ella el controlador funciona y responde por
 * I2C con normalidad, pero el panel no puede iluminar ni un píxel.
 */
static void oled_InitVerificado(void)
{
    HAL_Delay(100);                       /* estabilización tras encendido  */

    oled_Cmd(0xAE, "panel OFF");
    oled_Cmd(0x20, "modo direccionamiento");
    oled_Cmd(0x10, "  -> por paginas");
    oled_Cmd(0xB0, "pagina inicial 0");
    oled_Cmd(0xC8, "escaneo COM invertido");
    oled_Cmd(0x00, "columna baja 0");
    oled_Cmd(0x10, "columna alta 0");
    oled_Cmd(0x40, "linea de inicio 0");
    oled_Cmd(0x81, "contraste");
    oled_Cmd(0xFF, "  -> maximo");
    oled_Cmd(0xA1, "remapeo de segmentos");
    oled_Cmd(0xA6, "display normal");
    oled_Cmd(0xA8, "multiplexado");
    oled_Cmd(0x3F, "  -> 1:64");
    oled_Cmd(0xA4, "mostrar memoria");
    oled_Cmd(0xD3, "offset vertical");
    oled_Cmd(0x00, "  -> cero");
    oled_Cmd(0xD5, "reloj de display");
    oled_Cmd(0xF0, "  -> frecuencia alta");
    oled_Cmd(0xD9, "precarga");
    oled_Cmd(0x22, "  -> fase 2/2");
    oled_Cmd(0xDA, "config pines COM");
    oled_Cmd(0x12, "  -> alternativa");
    oled_Cmd(0xDB, "nivel VCOMH");
    oled_Cmd(0x20, "  -> 0.77 x VCC");
    oled_Cmd(0x8D, "BOMBA DE CARGA");        /* <- el comando crítico       */
    oled_Cmd(0x14, "  -> HABILITADA");
    oled_Cmd(0xAF, "panel ON");

    if (errores == 0u)
    {
        print("  Los 28 comandos fueron aceptados.\r\n");
    }
}

/*
 * oled_LlenarPantalla — escribe el mismo byte en toda la memoria de vídeo.
 * Recorre las 8 páginas; en cada una reposiciona el puntero y envía 128
 * bytes. El registro 0x40 pone D/C# a 1, indicando datos de píxel.
 */
static void oled_LlenarPantalla(uint8_t patron)
{
    uint8_t linea[128];
    char    buf[64];

    memset(linea, patron, sizeof(linea));

    for (uint8_t pagina = 0; pagina < 8u; pagina++)
    {
        oled_Cmd((uint8_t)(0xB0 + pagina), "pagina");
        oled_Cmd(0x00, "col baja");
        oled_Cmd(0x10, "col alta");

        if (HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x40, 1,
                              linea, 128, 200) != HAL_OK)
        {
            errores++;
            snprintf(buf, sizeof(buf),
                     "  [FALLO] datos de la pagina %u\r\n", (unsigned)pagina);
            print(buf);
        }
    }
}

/* ═══════════════════════ Configuración ════════════════════════════════════ */

/* HSI a 16 MHz, sin PLL: la configuración con la que el barrido funcionó */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

static void tim10_Init(void)
{
    __HAL_RCC_TIM10_CLK_ENABLE();

    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 15999;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 249;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim10);
    HAL_TIM_Base_Start_IT(&htim10);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

static void i2c1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

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
    HAL_I2C_Init(&hi2c1);
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
    HAL_UART_Init(&huart2);
}

static void print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 500);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
    }
}
