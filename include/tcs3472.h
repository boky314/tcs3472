#ifndef TCS3472_H_
#define TCS3472_H_

#include "esp_err.h"
#include "hal/gpio_types.h"

typedef enum {
  TCS34725_GAIN_1X = 0x00,  /**  No gain  */
  TCS34725_GAIN_4X = 0x01,  /**  x4 gain  */
  TCS34725_GAIN_16X = 0x02, /**  x16 gain */
  TCS34725_GAIN_60X = 0x03  /**  x60 gain */
} tcs3472_gain_t;



/** Integration time settings for TCS34725 */
/*
 * 60-Hz period: 16.67ms, 50-Hz period: 20ms
 * 100ms is evenly divisible by 50Hz periods and by 60Hz periods
 */


#define TCS34725_INTEGRATIONTIME_2_4MS	(0xFF)	/** < 2.4ms - 1 cycle - Max Count: 1024 */
#define TCS34725_INTEGRATIONTIME_24MS	(0xF6)	/**< 24.0ms - 10 cycles - Max Count: 10240 */
#define TCS34725_INTEGRATIONTIME_50MS	(0xEB)	/**< 50.4ms - 21 cycles - Max Count: 21504 */

#define TCS34725_INTEGRATIONTIME_60MS	(0xE7) /**< 60.0ms - 25 cycles - Max Count: 25700 */
#define TCS34725_INTEGRATIONTIME_101MS	(0xD6) /**< 100.8ms - 42 cycles - Max Count: 43008 */
#define TCS34725_INTEGRATIONTIME_120MS	(0xCE) /**< 120.0ms - 50 cycles - Max Count: 51200 */
#define TCS34725_INTEGRATIONTIME_154MS	(0xC0) /**< 153.6ms - 64 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_180MS	(0xB5) /**< 180.0ms - 75 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_199MS	(0xAD) /**< 199.2ms - 83 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_240MS	(0x9C) /**< 240.0ms - 100 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_300MS	(0x83) /**< 300.0ms - 125 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_360MS	(0x6A) /**< 360.0ms - 150 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_401MS	(0x59) /**< 400.8ms - 167 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_420MS	(0x51) /**< 420.0ms - 175 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_480MS	(0x38) /**< 480.0ms - 200 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_499MS	(0x30) /**< 499.2ms - 208 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_540MS	(0x1F) /**< 540.0ms - 225 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_600MS	(0x06) /**< 600.0ms - 250 cycles - Max Count: 65535 */
#define TCS34725_INTEGRATIONTIME_614MS	(0x00) /**< 614.4ms - 256 cycles - Max Count: 65535 */



/**
 * Initializing TCS3472 driver, include i2c.
 * Convenient if only TCS3472 exists on I2C bus.
 */
esp_err_t tcs3472_init( const int i2c_port, const gpio_num_t gpio_sda, const gpio_num_t gpio_scl );

/**
 * Initializing TCS3472 driver with already initialized i2c device.
 */
esp_err_t tcs3472_init_driver( const int i2c_port );

esp_err_t tcs3472_enable();
esp_err_t tcs3472_disable();

/**
 * Enable interrupts and set the limits.
 */
esp_err_t tcs3472_enable_interrupts();
esp_err_t tcs3472_set_interrupt_limits( const uint16_t low_limit, const uint16_t high_limit );
esp_err_t tcs3472_disable_interrupts();
esp_err_t tcs3472_clear_interrupt();

esp_err_t tcs3472_set_integration_time( const uint8_t integration_time );
esp_err_t tcs3472_set_gain( const tcs3472_gain_t gain );
esp_err_t tcs3472_read_raw( uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear );
esp_err_t tcs3472_read_rgb( uint8_t *red, uint8_t *green, uint8_t *blue );

#endif
