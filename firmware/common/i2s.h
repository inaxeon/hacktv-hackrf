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

#define I2S_DMA_CHANNEL         	1
#define I2S_NUM_BUFFERS				4

// For sync mode audio buffers are transferred and consumed in lockstep with video data,
// possible because the I2S master is clocked from the same source thus entirely synchronous.
// 128 samples is chosen because it results in a 512 byte buffer which is efficent to transfer
// over USB with the video buffers (which are 16KB each). Resulting audio sample rates
// are fs / 64. A bit overkill but the alternative is throwing away data on the wire.

#define I2S_BUFFER_DEPTH_SYNC		128 // 32-bit words (each one 16-bit L+R sample)

// For async mode larger buffers are used. 512 samples results in 2K buffers which
// from experimentation (thus far) appears to be the sweet spot in terms of minimal
// CPU time to de-queue them -without- clogging up the wire potentially causing video underruns.

#define I2S_BUFFER_DEPTH_ASYNC		512 // 32-bit words (each one 16-bit L+R sample)

struct i2s_state
{
	uint32_t usb_count;
	uint32_t i2s_count;
	uint32_t num_shortfalls;
	uint32_t usb_transfer_size;
	uint32_t buffer_depth;
	uint32_t buffer_size;
	uint32_t buffer_mask;
};

void i2s_init();
void i2s_startup(bool ext_clock);
void i2s_shutdown();
void i2s_streaming_enable();
void i2s_mute(bool mute);
void i2s_gpdma_isr();
void i2s_generate_test_tone();
void i2s_resume();
bool i2s_is_paused();

extern uint8_t i2s_audio_buffer[];
extern volatile struct i2s_state i2s_state;

#endif /*__I2S_H__*/
