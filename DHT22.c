/**
 * @file    DHT22.c
 * @brief   Single-wire DHT22 (AM2302) driver for STM32F103C8T6 (HAL).
 *
 * Protocol recap (all times approximate):
 *   MCU : pull low >= 1 ms, release
 *   DHT : low 80 us, high 80 us            <- response
 *   DHT : 40 x { low 50 us, high 26-28 us = '0' | high ~70 us = '1' }
 *   Data: [RH_hi][RH_lo][T_hi][T_lo][checksum]
 *         RH   = 16-bit / 10
 *         T    = 15-bit / 10, MSB = sign
 *         csum = low 8 bits of sum of first 4 bytes
 */

#include "DHT22.h"

/* ---------------- Tunables ---------------- */
#define DHT22_START_LOW_US      1500U   /* host start pulse (datasheet: 1-10 ms)  */
#define DHT22_RESP_TIMEOUT_US   120U    /* max wait for any single response edge  */
#define DHT22_BIT_TIMEOUT_US    120U    /* max wait for any single bit edge       */
#define DHT22_BIT_THRESHOLD_US  40U     /* high longer than this => logic '1'     */

/* ---------------- Microsecond timing (DWT) ---------------- */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* enable trace block */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            /* start cycle counter */
}

static inline uint32_t cycles_per_us(void)
{
    return SystemCoreClock / 1000000U;
}

static void delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * cycles_per_us();
    while ((DWT->CYCCNT - start) < cycles) { /* spin */ }
}

/* ---------------- Pin helpers ---------------- */

static void gpio_clk_enable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
#ifdef GPIOE
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
}

/* Open-drain: writing 0 pulls the line low, writing 1 releases it
 * (external pull-up takes it high). The IDR can still be read in this
 * mode, so no mode switching is needed between "send" and "receive". */
static inline void line_low(const DHT22_t *d)
{
    d->port->BSRR = (uint32_t)d->pin << 16U;
}

static inline void line_release(const DHT22_t *d)
{
    d->port->BSRR = d->pin;
}

static inline uint8_t line_read(const DHT22_t *d)
{
    return (d->port->IDR & d->pin) ? 1U : 0U;
}

/**
 * Wait until the line equals `level`.
 * @return elapsed microseconds waited, or 0xFFFFFFFF on timeout.
 */
static uint32_t wait_for_level(const DHT22_t *d, uint8_t level, uint32_t timeout_us)
{
    uint32_t start   = DWT->CYCCNT;
    uint32_t timeout = timeout_us * cycles_per_us();
    uint32_t elapsed;

    while (line_read(d) != level) {
        elapsed = DWT->CYCCNT - start;
        if (elapsed >= timeout) {
            return 0xFFFFFFFFU;
        }
    }
    return (DWT->CYCCNT - start) / cycles_per_us();
}

/* ---------------- Public API ---------------- */

void DHT22_Init(DHT22_t *dev, GPIO_TypeDef *port, uint16_t pin)
{
    if (dev == NULL || port == NULL) {
        return;
    }

    dev->port = port;
    dev->pin  = pin;

    gpio_clk_enable(port);

    GPIO_InitTypeDef gi = {0};
    gi.Pin   = pin;
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_NOPULL;            /* use external pull-up (4.7k-10k) */
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gi);

    line_release(dev);                 /* idle high */
    dwt_init();
}

DHT22_Status_t DHT22_Measure(DHT22_t *dev, float *temperature, float *humidity)
{
    uint8_t  data[5] = {0};
    uint32_t t;
    uint32_t primask;

    if (dev == NULL || dev->port == NULL || temperature == NULL || humidity == NULL) {
        return DHT22_ERR_PARAM;
    }

    /* ---- 1. Start signal (timing here is loose, IRQs may stay on) ---- */
    line_release(dev);
    delay_us(50);
    line_low(dev);
    delay_us(DHT22_START_LOW_US);

    /* ---- 2. Time-critical section: no interrupts ---- */
    primask = __get_PRIMASK();
    __disable_irq();

    line_release(dev);

    /* Line must first rise (pull-up), then sensor pulls it low (~80us),
     * releases it high (~80us), then starts the first bit with a low. */
    if (wait_for_level(dev, 1, DHT22_RESP_TIMEOUT_US) == 0xFFFFFFFFU ||
        wait_for_level(dev, 0, DHT22_RESP_TIMEOUT_US) == 0xFFFFFFFFU ||
        wait_for_level(dev, 1, DHT22_RESP_TIMEOUT_US) == 0xFFFFFFFFU ||
        wait_for_level(dev, 0, DHT22_RESP_TIMEOUT_US) == 0xFFFFFFFFU) {
        __set_PRIMASK(primask);
        return DHT22_ERR_NO_RESPONSE;
    }

    /* ---- 3. Read 40 bits ----
     * We're now at the start of bit 0's 50us low period.
     * For each bit: wait for rising edge, then time the high pulse. */
    for (uint8_t i = 0; i < 40; i++) {
        if (wait_for_level(dev, 1, DHT22_BIT_TIMEOUT_US) == 0xFFFFFFFFU) {
            __set_PRIMASK(primask);
            return DHT22_ERR_TIMEOUT;
        }

        t = wait_for_level(dev, 0, DHT22_BIT_TIMEOUT_US);   /* high pulse width */
        if (t == 0xFFFFFFFFU) {
            /* The last bit's falling edge may be held high... the sensor
             * releases the bus after the final bit, so treat a timeout on
             * the last bit as the line returning to idle (high). */
            if (i == 39) {
                t = DHT22_BIT_THRESHOLD_US + 1U;  /* long high => '1' */
            } else {
                __set_PRIMASK(primask);
                return DHT22_ERR_TIMEOUT;
            }
        }

        data[i / 8] <<= 1;
        if (t > DHT22_BIT_THRESHOLD_US) {
            data[i / 8] |= 1U;
        }
    }

    __set_PRIMASK(primask);   /* restore interrupts */

    /* ---- 4. Validate checksum ---- */
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return DHT22_ERR_CHECKSUM;
    }

    /* ---- 5. Convert ---- */
    uint16_t raw_h = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_t = ((uint16_t)data[2] << 8) | data[3];

    float t_c = (float)(raw_t & 0x7FFFU) / 10.0f;
    if (raw_t & 0x8000U) {
        t_c = -t_c;                       /* sign-magnitude, not two's complement */
    }

    *humidity    = (float)raw_h / 10.0f;
    *temperature = t_c;

    return DHT22_OK;
}
