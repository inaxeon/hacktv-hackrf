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

#define MCP47FEBXX_A1_SLAVE_ADDR            0x61
#define MCP47FEBXX_CMD_READ                 0x06
#define MCP47FEBXX_NONVOLATILE_DAC0         0x10
#define MCP47FEBXX_NONVOLATILE_DAC1         0x11

struct gpio_t video_out_led = GPIO(1, 9);
struct gpio_t vdac_sw_clock = GPIO(5, 13);

static uint8_t _audio_mode;
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
	gpio_set(&vdac_sw_clock);
	delay(100);
	gpio_clear(&vdac_sw_clock);
}

void hackdac_init()
{
	_baseband_enabled = false;
	_audio_mode = HACKDAC_NO_AUDIO;
	_rffc5071_hijacked = false;

	gpio_output(&video_out_led);
	gpio_set(&video_out_led);

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
		delay(6000000);
		video_led_off();
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

	if (baseband_enabled) {
		if (!cpld_jtag_sram_load_hackdac(&jtag_cpld)) {
			halt_and_flash(6000000);
		}
		si5351c_mcu_clk_enable(&clock_gen, true);
	} else {
		if (audio_mode == HACKDAC_SYNC_AUDIO) {
			return false; 	// Not possible on anything other than HackRF-R9 becuase of the limitations of PLL 7.
							// No point in screwing around enabling it for a small clutch of R9 users. Just block it.
		}
		if (!cpld_jtag_sram_load_hackrf(&jtag_cpld)) {
			halt_and_flash(6000000);
		}
		si5351c_mcu_clk_enable(&clock_gen, false);
	}

	_baseband_enabled = baseband_enabled;
	_rffc5071_hijacked = rffc5071_hijacked;
	_audio_mode = audio_mode;
	hackdac_zero_video_output();
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