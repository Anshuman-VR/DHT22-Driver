/**
 * @file    DHT22.h
 * @brief   Single-wire DHT22 (AM2302) driver for STM32F103C8T6 (HAL).
 *
 * - Works on ANY GPIO pin (pass the port + pin to DHT22_Init).
 * - Pin is configured as open-drain output; an external pull-up
 *   (4.7k - 10k to 3.3V) is REQUIRED on the data line.
 * - Target device: STM32F103C8T6
 *   Written by: Anshuman VR
 */

#ifndef DHT22_H
#define DHT22_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/** Return codes */
typedef enum {
    DHT22_OK = 0,
    DHT22_ERR_NO_RESPONSE,   /**< Sensor never answered the start signal   */
    DHT22_ERR_TIMEOUT,       /**< Timed out while reading a bit            */
    DHT22_ERR_CHECKSUM,      /**< Data received but checksum mismatch      */
    DHT22_ERR_PARAM          /**< NULL pointer / bad handle                */
} DHT22_Status_t;

/** Device handle: one per sensor, lets you use several sensors on different pins */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} DHT22_t;

/**
 * @brief  Initialise the driver for a sensor on the given pin.
 *         Enables the GPIO clock, sets the pin to open-drain (released/high),
 *         and enables the DWT cycle counter used for us delays.
 *
 * @note   Wait ~1 s after sensor power-up before the first measurement.
 */
void DHT22_Init(DHT22_t *dev, GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief  Perform one measurement.
 * @param  dev          Sensor handle
 * @param  temperature  Out: temperature in degrees C  (may not be NULL)
 * @param  humidity     Out: relative humidity in %RH   (may not be NULL)
 * @retval DHT22_OK on success, otherwise an error code (outputs untouched).
 *
 * @note   Sensor needs >= 2 s between measurements.
 * @note   Interrupts are disabled for ~4 ms during the bit-reading phase.
 */
DHT22_Status_t DHT22_Measure(DHT22_t *dev, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif /* DHT22_H */
