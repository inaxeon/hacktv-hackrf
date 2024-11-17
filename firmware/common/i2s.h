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

#ifndef __I2S_H__
#define __I2S_H__

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/i2s.h>
#include <libopencm3/lpc43xx/creg.h>

#define I2S_DMA_CHANNEL         1
#define I2S_NUM_BUFFERS			4
#define I2S_BUFFER_DEPTH		128 // 32-bit words (each one 16-bit L+R sample)
#define I2S_USB_TRANSFER_SIZE	(I2S_BUFFER_DEPTH * sizeof(uint32_t))
#define I2S_BUFFER_MASK			((I2S_USB_TRANSFER_SIZE * I2S_NUM_BUFFERS) - 1)
#define I2S_BUFFER_SIZE			(I2S_USB_TRANSFER_SIZE * I2S_NUM_BUFFERS)

void i2s_init();
void i2s_startup();
void i2s_shutdown();
void i2s_streaming_enable();
void i2s_mute(bool mute);
void i2s_gpdma_isr();
void i2s_generate_test_tone();
void i2s_resume();
bool i2s_is_paused();
uint32_t i2s_bytes_transferred();

extern uint8_t i2s_audio_buffer[];
extern uint32_t i2s_usb_bytes_transferred;

#endif /*__I2S_H__*/
