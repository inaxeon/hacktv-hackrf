#include "i2s.h"

#include <stdio.h>
#include <math.h>

#define CLK_APB1_I2S 204000000 // 204 MHz


static bool i2s_get_clock_divider(int sample_rate, int word_width, uint16_t *px_div, uint16_t *py_div, uint32_t *pN);

char debug_buffer[512];

void i2s_init()
{
	int word_width = 16;
	int sample_rate = 48000;
	uint16_t x_div = 0;
	uint16_t y_div = 0;
	uint32_t clk_n = 0;

	CCU1_CLK_APB1_I2S_CFG = 1;

	i2s_get_clock_divider(sample_rate, word_width, &x_div, &y_div, &clk_n);

	I2S0_DAO = I2S0_DAO_WORDWIDTH(1) | // 16-bit
		I2S0_DAO_MONO(0) | // Stereo
		I2S0_DAO_WS_SEL(0) | // Master
		I2S0_DAO_WS_HALFPERIOD(word_width - 1); // WS half-cycle: 16-bits

	I2S0_TXMODE = I2S0_TXMODE_TXCLKSEL(0);

	I2S0_TXBITRATE = (clk_n - 1);
	I2S0_TXRATE = y_div | (x_div << 8);

	sprintf(debug_buffer, "I2S setup complete\n");
}

void i2s_start()
{
	I2S0_DAO &= ~(I2S0_DAO_RESET_MASK | I2S0_DAO_STOP_MASK | I2S0_DAO_MUTE_MASK);
}

void i2s_stop()
{
	I2S0_DAO &= ~I2S0_DAO_MUTE_MASK;
	I2S0_DAO |= (I2S0_DAO_STOP_MASK | I2S0_DAO_RESET_MASK);
}

void i2s_mute(bool mute)
{
	if (mute)
		I2S0_DAO |= I2S0_DAO_MUTE_MASK;
	else
		I2S0_DAO &= ~I2S0_DAO_MUTE_MASK;
}

uint8_t i2s_get_tx_level()
{
	return (I2S0_STATE >> 16) & 0x0F;
}

void i2s_tx(uint32_t data)
{
	I2S0_TXFIFO = data;
}

#define NUM_SAMPLES 48

void i2s_test_tone()
{
	int x, y;
	double d;
	int16_t l;
	int sample_rate = 48000;
	int audio_samples;
	uint32_t audio[NUM_SAMPLES];
	int tone_duration = 3000;

	d = 1000.0 * 2 * M_PI / sample_rate;
	y = sample_rate / 1000; /* 1ms */
	audio_samples = y;

	if (audio_samples != NUM_SAMPLES)
		return;

	for (x = 0; x < audio_samples; x++)
	{
		l = sin(x * d) * INT16_MAX * 0.1;
		audio[x] = ((uint16_t)l | (uint32_t)(l << 16));
	}

	for (int x = 0; x < tone_duration; x++)
	{
		for (int y = 0; y < NUM_SAMPLES; y++)
		{
			while (i2s_get_tx_level() >= 4);

			i2s_tx(audio[y]);
		}
	}
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