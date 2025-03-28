/* hacktv - Analogue video transmitter for the HackRF                    */
/*=======================================================================*/
/* Copyright 2017 Philip Heron <phil@sanslogic.co.uk>                    */
/* Author: Matthew Millman <inaxeon@hotmail.com>                         */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "hackrf_core.h"
#include "hackdac.h"
#include "cpld_jtag.h"
#include "cpld_xc2c.h"
#include "i2s.h"
#include "firmware_info.h"

#define MCP47FEBXX_A1_SLAVE_ADDR            0x61
#define MCP47FEBXX_CMD_READ                 0x06
#define MCP47FEBXX_NONVOLATILE_DAC0         0x10
#define MCP47FEBXX_NONVOLATILE_DAC1         0x11


static struct gpio_t hackdac_pwr_en = GPIO(1, 13);
static struct gpio_t video_out_led = GPIO(1, 9);
static struct gpio_t rf_out_led = GPIO(1, 2);
static struct gpio_t vdac_sw_clock = GPIO(5, 13);
static struct gpio_t tcxo_clock_enable = GPIO(1, 6);
static struct gpio_t board_id_0 = GPIO(3, 8);
static struct gpio_t board_id_1 = GPIO(3, 9);
static struct gpio_t board_id_2 = GPIO(3, 10);
static struct gpio_t board_id_3 = GPIO(3, 11);

static uint8_t _audio_mode;
static uint8_t _board_id;
static bool _baseband_enabled;
static bool _rffc5071_hijacked;

static bool cpld_jtag_sram_load_hackrf(jtag_t* const jtag)
{
	cpld_jtag_take(jtag);
	cpld_xc2c64a_jtag_sram_write(jtag, &cpld_hackrf_program_sram);
	const bool success = cpld_xc2c64a_jtag_sram_verify(
		jtag,
		&cpld_hackrf_program_sram,
		&cpld_hackrf_verify);
	cpld_jtag_release(jtag);
	return success;
}

static bool cpld_jtag_sram_load_hackdac(jtag_t* const jtag)
{
	cpld_jtag_take(jtag);
	cpld_xc2c64a_jtag_sram_write(jtag, &cpld_hackdac_program_sram);
	const bool success = cpld_xc2c64a_jtag_sram_verify(
		jtag,
		&cpld_hackdac_program_sram,
		&cpld_hackdac_verify);
	cpld_jtag_release(jtag);
	return success;
}

static void hackdac_zero_video_output()
{
	// Latch the zero-volt word into the DAC.
	// This ensures the output is zero'd when running in RF mode
	// Only works when the sgpio_if CPLD code is loaded
	gpio_set(&vdac_sw_clock);
	delay(100);
	gpio_clear(&vdac_sw_clock);
}

void hackdac_init()
{
	_baseband_enabled = false;
	_audio_mode = HACKDAC_NO_AUDIO;
	_rffc5071_hijacked = false;

	scu_pinmux(SCU_PINMUX_GPIO3_8, SCU_CONF_FUNCTION0 | SCU_GPIO_PUP);
	scu_pinmux(SCU_PINMUX_GPIO3_9, SCU_CONF_FUNCTION0 | SCU_GPIO_PUP);
	scu_pinmux(SCU_PINMUX_GPIO3_10, SCU_CONF_FUNCTION0 | SCU_GPIO_PUP);
	scu_pinmux(SCU_PINMUX_GPIO3_11, SCU_CONF_FUNCTION0 | SCU_GPIO_PUP);

	gpio_input(&board_id_0);
	gpio_input(&board_id_1);
	gpio_input(&board_id_2);
	gpio_input(&board_id_3);

	_board_id = (!gpio_read(&board_id_0) << 0) |
				(!gpio_read(&board_id_1) << 1) |
				(!gpio_read(&board_id_2) << 2) |
				(!gpio_read(&board_id_3) << 3);
	
	_board_id &= 0x0F;

	gpio_output(&hackdac_pwr_en);
	gpio_set(&hackdac_pwr_en);

	gpio_output(&tcxo_clock_enable);
	gpio_set(&tcxo_clock_enable);

	gpio_output(&video_out_led);
	gpio_set(&video_out_led);

	gpio_output(&rf_out_led);
	gpio_set(&rf_out_led);

	gpio_output(&vdac_sw_clock);
	delay(100);
	hackdac_zero_video_output();

	i2s_init();
}

void hackdac_error()
{
	// System error: Blink the video out LED.
	while (1) {
		video_led_on();
		rf_led_on();
		delay(6000000);
		video_led_off();
		rf_led_off();
		delay(6000000);
	}
}

bool hackdac_baseband_enabled()
{
	return _baseband_enabled;
}

bool hackdac_set_mode(uint8_t mode)
{
	uint8_t audio_mode = (mode & HACKDAC_AUDIO_MODE_MASK) >> HACKDAC_AUDIO_MODE_SHIFT;
	bool rffc5071_hijacked = (mode & HACKDAC_RFFC5071_HIJACK) == HACKDAC_RFFC5071_HIJACK;
	bool baseband_enabled = (mode & HACKDAC_MODE_BASEBAND) == HACKDAC_MODE_BASEBAND;

	// Step 1: Check if requested mode can be set
	if (baseband_enabled) {
		// Presently no validation
	} else {
		if (audio_mode == HACKDAC_SYNC_AUDIO) {
			return false; 	// Not possible on anything other than HackRF-R9 becuase of the limitations of PLL 7.
							// No point in screwing around enabling it for a small clutch of R9 users. Just block it.
		}
	}

	// Step 2: Set requested mode. Nothing which can fail allowed after here.
	_baseband_enabled = baseband_enabled;
	_rffc5071_hijacked = rffc5071_hijacked;
	_audio_mode = audio_mode;

	if (_baseband_enabled)
	{
		if (!cpld_jtag_sram_load_hackdac(&jtag_cpld)) {
			halt_and_flash(6000000);
		}

		// Soft-start HackDAC
		for (int i = 0; i <= 5000; i++)
		{
			gpio_clear(&hackdac_pwr_en); // Power up
			gpio_set(&hackdac_pwr_en); // Power down
			delay_us_at_mhz(10 - (i / 500), 96);
		}
		
		gpio_clear(&hackdac_pwr_en); // Power up and stay on
		
		gpio_clear(&tcxo_clock_enable); // Activate 27 MHz clock
		si5351c_mcu_clk_enable(&clock_gen, true);
	}
	else
	{
		if (!cpld_jtag_sram_load_hackrf(&jtag_cpld)) {
			halt_and_flash(6000000);
		}
		si5351c_mcu_clk_enable(&clock_gen, false);
		gpio_set(&hackdac_pwr_en); // Power down HackDAC
		gpio_set(&tcxo_clock_enable); // Shut off 27 MHz clock
		hackdac_zero_video_output();
	}

	activate_best_clock_source();
	return true;
}

audio_mode_t hackdac_get_audio_mode()
{
	return _audio_mode;
}

bool hackdac_i2c_reg_read(uint8_t reg, uint16_t *value)
{
	const uint8_t data_tx[] = {(reg << 3) | MCP47FEBXX_CMD_READ};

	i2c_bus_transfer(&i2c0, MCP47FEBXX_A1_SLAVE_ADDR, data_tx, 1, (uint8_t *)value, 2);

	return true;
}

bool hackdac_i2c_reg_write(uint8_t reg, uint16_t value)
{
	uint8_t data_tx[3];

	data_tx[0] = (reg << 3);
	data_tx[2] = value & 0xFF;
	data_tx[1] = value >> 8;

	i2c_bus_transfer(&i2c0, MCP47FEBXX_A1_SLAVE_ADDR, data_tx, 3, NULL, 0);

	if (reg == MCP47FEBXX_NONVOLATILE_DAC0 || reg == MCP47FEBXX_NONVOLATILE_DAC1)
		delay(500000); // NV registers take a little bit of time to update

	return true;
}

bool hackdac_rffc5071_api_is_hijacked()
{
	return _rffc5071_hijacked;
}

void video_led_on()
{
	gpio_clear(&video_out_led);
}

void video_led_off()
{
	gpio_set(&video_out_led);
}

void rf_led_on()
{
	gpio_clear(&rf_out_led);
}

void rf_led_off()
{
	gpio_set(&rf_out_led);
}

void hackdac_get_version(char *buffer, int len)
{
	const char *hackdac_token;
	strncpy(buffer, firmware_info.version_string, len - 1);

	hackdac_token = strstr(buffer, "hackdac");

	if (!hackdac_token)
		hackdac_error();

	*((char *)(hackdac_token + 8)) = 'a' + _board_id;
}