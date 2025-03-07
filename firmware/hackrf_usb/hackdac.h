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

#ifndef __HACKDAC_H__
#define __HACKDAC_H__

#define HACKDAC_SYNC_XFER_SIZE			512
#define HACKDAC_SYNC_MAGIC_1			0x87654321
#define HACKDAC_SYNC_MAGIC_2			0x12345678
#define HACKDAC_SYNC_AWAITING			0xFFFFFFFF

#define HACKDAC_MODE_BASEBAND			(1 << 7)
#define HACKDAC_RFFC5071_HIJACK			(1 << 6)

#define HACKDAC_AUDIO_MODE_SHIFT		(1)
#define HACKDAC_AUDIO_MODE_MASK			(0x3 << HACKDAC_AUDIO_MODE_SHIFT)
#define HACKDAC_AUDIO_MODE(x) 			((x) << HACKDAC_AUDIO_MODE_SHIFT)

#define HACKDAC_REG_START				0x20
#define HACKDAC_NUM_REGS				0x20

typedef enum {
	HACKDAC_NO_AUDIO = 0,
	HACKDAC_SYNC_AUDIO = 1,
	HACKDAC_ASYNC_AUDIO = 2
} audio_mode_t;

void hackdac_init();
bool hackdac_set_mode(uint8_t mode);
audio_mode_t hackdac_get_audio_mode();
bool hackdac_baseband_enabled();
bool hackdac_i2c_reg_read(uint8_t idx, uint16_t *value);
bool hackdac_i2c_reg_write(uint8_t idx, uint16_t value);
bool hackdac_rffc5071_api_is_hijacked();
void hackdac_get_version(char *buffer, int len);

#endif /*__HACKDAC_H__*/
