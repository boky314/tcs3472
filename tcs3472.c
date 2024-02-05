#include <stdio.h>
#include <unistd.h>
// TODO: stdio, unistd nem kell majd!
#include "tcs3472.h"

#include "driver/i2c.h"

#define I2C_MASTER_FREQ_HZ          400000
#define TIMEOUT (1000)

/****    REGISTER ADDRESSES    ****/

#define TCS34725_REG_ENABLE 	(0x00)
#define TCS34725_REG_ATIME 		(0x01)
#define TCS34725_REG_WTIME		(0x03)
#define TCS34725_REG_AILTL		(0x04)
#define TCS34725_REG_AILTH		(0x05)
#define TCS34725_REG_AIHTL		(0x06)
#define TCS34725_REG_AIHTH		(0x07)
#define TCS34725_REG_PERS 		(0x0C)
#define TCS34725_REG_CONFIG 	(0x0D)
#define TCS34725_REG_CONTROL	(0x0F)

#define TCS34725_REG_ID			(0x12)
#define TCS34725_REG_STATUS		(0x13)
#define TCS34725_REG_CDATAL		(0x14)
#define TCS34725_REG_CDATAH		(0x15)
#define TCS34725_REG_RDATAL		(0x16)
#define TCS34725_REG_RDATAH 	(0x17)
#define TCS34725_REG_GDATAL 	(0x18)
#define TCS34725_REG_GDATAH 	(0x19)
#define TCS34725_REG_BDATAL 	(0x1A)
#define TCS34725_REG_BDATAH 	(0x1B)

/****     ****/

#define TCS34725_ENABLE_BIT_AIEN	(0x10)
#define TCS34725_ENABLE_BIT_WEN		(0x08)
#define TCS34725_ENABLE_BIT_AEN		(0x02)
#define TCS34725_ENABLE_BIT_PON		(0x01)


#define COMMAND_BIT (0x80)

#define BUFFER_LENGTH 8
uint8_t tcs3472_buffer[BUFFER_LENGTH];

int tcs4372_i2c_port;
uint8_t device_address;
uint8_t tcs4372_integration_time;
tcs3472_gain_t tcs4372_gain;

esp_err_t tcs4372_register_read(uint8_t reg_addr, size_t len)
{
	uint8_t cmd = reg_addr | COMMAND_BIT;
	return i2c_master_write_read_device( tcs4372_i2c_port, device_address, &cmd, 1, tcs3472_buffer, len, TIMEOUT / portTICK_PERIOD_MS );
}

uint16_t tcs4372_read_uint16(uint8_t reg_addr)
{
  uint8_t buffer[2] = {(uint8_t)(COMMAND_BIT | reg_addr), 0};
  i2c_master_write_read_device( tcs4372_i2c_port, device_address, buffer, 1, buffer, 2, TIMEOUT / portTICK_PERIOD_MS );
  return ((uint16_t)(buffer[1]) << 8) | ((uint16_t)(buffer[0]) & 0xFF);
}

esp_err_t tcs4372_register_write_byte(uint8_t reg_addr, uint8_t one_byte)
{
    uint8_t write_buf[2] = {reg_addr | COMMAND_BIT, one_byte};
    return i2c_master_write_to_device(tcs4372_i2c_port, device_address, write_buf, 2, TIMEOUT / portTICK_PERIOD_MS );
}

/**
 * Initializing TCS3472 driver with already initialized i2c device.
 */
esp_err_t tcs3472_init_driver( const int i2c_port )
{
	uint8_t possible_device_addresses[2] = {0x29, 0x39};

	for( int i=0; i<sizeof(possible_device_addresses); ++i ) {
		device_address = possible_device_addresses[i];
		if( tcs4372_register_read( TCS34725_REG_ID, 1) == ESP_OK &&
			(tcs3472_buffer[0] == 0x44 || tcs3472_buffer[0] == 0x4D) )
		{
			printf( "TCS3472 found at address 0x%02X\n", device_address );
			return ESP_OK;
		}
		else
		{
			printf( "NO TCS3472 at address 0x%02X\n", device_address );
		}
	}

	return ESP_ERR_NOT_FOUND;
}

esp_err_t tcs3472_init( const int i2c_port, const gpio_num_t gpio_sda, const gpio_num_t gpio_scl )
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = gpio_sda,
		.scl_io_num = gpio_scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};

	i2c_param_config(i2c_port , &conf);
	i2c_driver_install(i2c_port , conf.mode, 0, 0, 0);
	return tcs3472_init_driver(i2c_port);
}

esp_err_t tcs3472_enable()
{
	tcs4372_register_write_byte(TCS34725_REG_ENABLE, TCS34725_ENABLE_BIT_PON);
	sleep(3);
	tcs4372_register_write_byte(TCS34725_REG_ENABLE, TCS34725_ENABLE_BIT_PON | TCS34725_ENABLE_BIT_AEN);

	return ESP_OK;
}

esp_err_t tcs3472_disable()
{
	// TODO
	return ESP_ERR_NOT_SUPPORTED;
}

/**
 * Enable interrupts.
 */
esp_err_t tcs3472_enable_interrupts()
{
	// TODO
	return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t tcs3472_set_interrupt_limits( const uint16_t low_limit, const uint16_t high_limit )
{
	// TODO
	return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t tcs3472_disable_interrupts()
{
	// TODO
	return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t tcs3472_clear_interrupt()
{
	// TODO
	return ESP_ERR_NOT_SUPPORTED;
}



esp_err_t tcs3472_set_integration_time( const uint8_t integration_time )
{
	esp_err_t res = tcs4372_register_write_byte(TCS34725_REG_ATIME, integration_time);
	if( res == ESP_OK ) {
		tcs4372_integration_time = integration_time;
	}

	return res;
}


esp_err_t tcs3472_set_gain( const tcs3472_gain_t gain )
{
	esp_err_t res = tcs4372_register_write_byte(TCS34725_REG_CONTROL, gain);
	if( res == ESP_OK ) {
		tcs4372_gain = gain;
	}

	return res;
}


esp_err_t tcs3472_read_raw( uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear )
{
	*clear = tcs4372_read_uint16(TCS34725_REG_CDATAL);
	*red = tcs4372_read_uint16(TCS34725_REG_RDATAL);
	*green = tcs4372_read_uint16(TCS34725_REG_GDATAL);
	*blue = tcs4372_read_uint16(TCS34725_REG_BDATAL);

	return ESP_OK;
}


esp_err_t tcs3472_read_rgb( uint8_t *red, uint8_t *green, uint8_t *blue )
{
	uint16_t r=0, g=0, b=0, c=0;
	tcs3472_read_raw(&r, &g, &b, &c);

	if (c == 0) {
		*red = *green = *blue = 0;
		return ESP_OK;
	}

	// TODO ez elég pontos?
	*red 	= ((float)r) / c * 255;
	*green	= ((float)g) / c * 255;
	*blue	= ((float)b) / c * 255;

	return ESP_OK;
}


