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

#include <stdio.h>
#include <math.h>

#include "i2s.h"
#include "gpdma.h"

#define CLK_APB1_I2S 			204000000 // 204 MHz
#define I2S_DMA_FIFO_LEVEL		4
#define I2S_SAMPLE_RATE			48000

dma_lli i2s_dma_lli[I2S_NUM_BUFFERS];
uint8_t i2s_audio_buffer[I2S_USB_TRANSFER_SIZE * I2S_NUM_BUFFERS];
int32_t i2s_usb_bytes_transferred;
int32_t _bytes_transferred;

static void i2s_init_dma()
{
	// Set level of fifo at which to initiate a new transfer
	I2S0_DMA1 &= ~I2S0_DMA1_TX_DEPTH_DMA1_MASK;
	I2S0_DMA1 |= I2S0_DMA1_TX_DEPTH_DMA1(I2S_DMA_FIFO_LEVEL) | I2S0_DMA1_TX_DMA1_ENABLE_MASK;

	// Route I2S DMA1 to DMA req channel 9
	CREG_DMAMUX &= ~CREG_DMAMUX_DMAMUXPER9_MASK;
	CREG_DMAMUX |= CREG_DMAMUX_DMAMUXPER9(1);
}

static void i2s_init_lli()
{
	uint32_t cw = GPDMA_CCONTROL_TRANSFERSIZE(I2S_BUFFER_DEPTH) |
		GPDMA_CCONTROL_SBSIZE(0) // 1
		| GPDMA_CCONTROL_DBSIZE(0) // 1
		| GPDMA_CCONTROL_SWIDTH(2) // 32-bit word
		| GPDMA_CCONTROL_DWIDTH(2) // 32-bit word
		| GPDMA_CCONTROL_S(0)	  // AHB Master 0
		| GPDMA_CCONTROL_D(1)	  // AHB Master 1
		| GPDMA_CCONTROL_SI(1)	 // increment source
		| GPDMA_CCONTROL_DI(0)	 // do not increment destination
		| GPDMA_CCONTROL_PROT1(0)  // user mode
		| GPDMA_CCONTROL_PROT2(0)  // not bufferable
		| GPDMA_CCONTROL_PROT3(0)  // not cacheable
		| GPDMA_CCONTROL_I(1);	 // interrupt enabled

	i2s_dma_lli[0].src = (uint32_t) & (i2s_audio_buffer[0]); // Buffer 0
	i2s_dma_lli[0].dest = (uint32_t) & (I2S0_TXFIFO);
	i2s_dma_lli[0].next_lli = (uint32_t) & (i2s_dma_lli[1]);
	i2s_dma_lli[0].control = cw;

	i2s_dma_lli[1].src = (uint32_t) & (i2s_audio_buffer[I2S_USB_TRANSFER_SIZE]); // Buffer 1
	i2s_dma_lli[1].dest = (uint32_t) & (I2S0_TXFIFO);
	i2s_dma_lli[1].next_lli = (uint32_t) & (i2s_dma_lli[0]);
	i2s_dma_lli[1].control = cw;

	GPDMA_CSRCADDR(I2S_DMA_CHANNEL) = i2s_dma_lli[0].src;
	GPDMA_CDESTADDR(I2S_DMA_CHANNEL) = i2s_dma_lli[0].dest;
	GPDMA_CLLI(I2S_DMA_CHANNEL) = i2s_dma_lli[0].next_lli;
	GPDMA_CCONTROL(I2S_DMA_CHANNEL) = i2s_dma_lli[0].control;
	GPDMA_CCONFIG(I2S_DMA_CHANNEL) = GPDMA_CCONFIG_DESTPERIPHERAL(0x9) // DMA req channel 9
		| GPDMA_CCONFIG_FLOWCNTRL(1)				// memory-to-peripheral
		| GPDMA_CCONFIG_H(0)						// do not halt
		| GPDMA_CCONFIG_ITC(1)						// terminal count interrupt
		| GPDMA_CCONFIG_IE(1);						// error interrupt
}

static bool i2s_get_clock_divider(int sample_rate, int word_width, uint16_t *px_div, uint16_t *py_div, uint32_t *pclk_n)
{
	uint32_t x, y;
	uint64_t divider;
	uint16_t dif;
	uint16_t x_div = 0;
	uint16_t y_div = 0;
	uint32_t clk_n;
	uint16_t err;
	uint16_t error_optimal = 0xFFFF;

	/* divider is a fixed point number with 16 fractional bits */
	divider = (((uint64_t) sample_rate * 2 * word_width * 2) << 16) / CLK_APB1_I2S;
	
	/* find clk_n that makes x/y <= 1 -> divider <= 2^16 */
	for (clk_n = 64; clk_n > 0; clk_n--) {
		if ((divider * clk_n) < (1 << 16)) {
			break;
		}
	}
	
	if (clk_n == 0) {
		return false;
	}
	
	divider *= clk_n;

	for (y = 255; y > 0; y--) {
		x = y * divider;

		if (x & (0xFF000000)) {
			continue;
		}

		dif = x & 0xFFFF;

		if (dif > 0x8000) {
			err = 0x10000 - dif;
		}

		else {
			err = dif;
		}

		if (err == 0) {
			y_div = y;
			break;
		}

		else if (err < error_optimal) {
			error_optimal = err;
			y_div = y;
		}
	}

	x_div = ((uint64_t) y_div * sample_rate * 2 * word_width * clk_n * 2) / CLK_APB1_I2S;

	if (x_div >= 256) {
		x_div = 0xFF;
	}
	
	if (x_div == 0) {
		x_div = 1;
	}

	*px_div = x_div;
	*py_div = y_div;
	*pclk_n = clk_n;

	return true;
}

void i2s_init()
{
	int word_width = 16;
	uint16_t x_div = 0;
	uint16_t y_div = 0;
	uint32_t clk_n = 0;

	CCU1_CLK_APB1_I2S_CFG = 1; // Enable I2S branch clock

	i2s_get_clock_divider(I2S_SAMPLE_RATE, word_width, &x_div, &y_div, &clk_n);

	I2S0_DAO = I2S0_DAO_WORDWIDTH(1) | // 16-bit
		I2S0_DAO_MONO(0) | // Stereo
		I2S0_DAO_WS_SEL(0) | // Master
		I2S0_DAO_WS_HALFPERIOD(word_width - 1) |
		I2S0_DAO_STOP(1) | I2S0_DAO_RESET(1); // WS half-cycle: 16-bits

	I2S0_TXMODE = I2S0_TXMODE_TXCLKSEL(0);

	I2S0_TXBITRATE = (clk_n - 1);
	I2S0_TXRATE = y_div | (x_div << 8);

	i2s_init_dma();

	gpdma_interrupt_enable();
	gpdma_controller_enable();
}

void i2s_startup()
{
	i2s_init_lli();
	i2s_usb_bytes_transferred = 0;
	_bytes_transferred = 0;
}

void i2s_shutdown()
{
	// TODO: This is likely not an elegant shut down
	I2S0_DAO &= ~I2S0_DAO_MUTE_MASK;
	I2S0_DAO |= (I2S0_DAO_STOP_MASK | I2S0_DAO_RESET_MASK);
	gpdma_channel_disable(I2S_DMA_CHANNEL);
}

void i2s_streaming_enable()
{
	gpdma_channel_enable(I2S_DMA_CHANNEL);
	I2S0_DAO &= ~(I2S0_DAO_RESET_MASK | I2S0_DAO_STOP_MASK | I2S0_DAO_MUTE_MASK);
}

void i2s_mute(bool mute)
{
	if (mute)
		I2S0_DAO |= I2S0_DAO_MUTE_MASK;
	else
		I2S0_DAO &= ~I2S0_DAO_MUTE_MASK;
}

int32_t i2s_bytes_transferred()
{
	return _bytes_transferred;
}

void i2s_gpdma_isr()
{
	if (GPDMA_INTTCSTAT & (1 << I2S_DMA_CHANNEL))
	{
		GPDMA_INTTCCLEAR = (1 << I2S_DMA_CHANNEL);
		_bytes_transferred += I2S_USB_TRANSFER_SIZE;
	}

	if (GPDMA_INTTCSTAT & (1 << I2S_DMA_CHANNEL))
	{
		GPDMA_INTERRCLR = (1 << I2S_DMA_CHANNEL);
	}
}

#if 0
static void i2s_init_lli_test(int num_samples)
{
	uint32_t cw = GPDMA_CCONTROL_TRANSFERSIZE(num_samples) |
		GPDMA_CCONTROL_SBSIZE(0) // 1
		| GPDMA_CCONTROL_DBSIZE(0) // 1
		| GPDMA_CCONTROL_SWIDTH(2) // 32-bit word
		| GPDMA_CCONTROL_DWIDTH(2) // 32-bit word
		| GPDMA_CCONTROL_S(0)	  // AHB Master 0
		| GPDMA_CCONTROL_D(1)	  // AHB Master 1
		| GPDMA_CCONTROL_SI(1)	 // increment source
		| GPDMA_CCONTROL_DI(0)	 // do not increment destination
		| GPDMA_CCONTROL_PROT1(0)  // user mode
		| GPDMA_CCONTROL_PROT2(0)  // not bufferable
		| GPDMA_CCONTROL_PROT3(0)  // not cacheable
		| GPDMA_CCONTROL_I(1);	 // interrupt enabled

	i2s_dma_lli[0].src = (uint32_t) & (i2s_audio_buffer[0]); // Buffer 0
	i2s_dma_lli[0].dest = (uint32_t) & (I2S0_TXFIFO);
	i2s_dma_lli[0].next_lli = (uint32_t) & (i2s_dma_lli[0]);
	i2s_dma_lli[0].control = cw;

	GPDMA_CSRCADDR(I2S_DMA_CHANNEL) = i2s_dma_lli[0].src;
	GPDMA_CDESTADDR(I2S_DMA_CHANNEL) = i2s_dma_lli[0].dest;
	GPDMA_CLLI(I2S_DMA_CHANNEL) = i2s_dma_lli[0].next_lli;
	GPDMA_CCONTROL(I2S_DMA_CHANNEL) = i2s_dma_lli[0].control;
	GPDMA_CCONFIG(I2S_DMA_CHANNEL) = GPDMA_CCONFIG_DESTPERIPHERAL(0x9) // DMA req channel 9
		| GPDMA_CCONFIG_FLOWCNTRL(1)				// memory-to-peripheral
		| GPDMA_CCONFIG_H(0)						// do not halt
		| GPDMA_CCONFIG_ITC(1)						// terminal count interrupt
		| GPDMA_CCONFIG_IE(1);						// error interrupt
}

static uint8_t i2s_get_tx_level()
{
	return (I2S0_STATE >> 16) & 0x0F;
}

static void i2s_tx(uint32_t data)
{
	I2S0_TXFIFO = data;
}

void i2s_generate_test_tone()
{
	int x, y;
	double d;
	int16_t l;
	int audio_samples;

	d = 1000.0 * 2 * M_PI / I2S_SAMPLE_RATE;
	y = I2S_SAMPLE_RATE / 1000; /* 1ms */
	audio_samples = y;

	for (x = 0; x < audio_samples; x++)
	{
		l = sin(x * d) * INT16_MAX * 0.1;
		*(((uint32_t *)i2s_audio_buffer) + x) = ((uint16_t)l | (uint32_t)(l << 16));
	}

	i2s_init_lli_test(audio_samples);
	i2s_streaming_enable();
}
#endif