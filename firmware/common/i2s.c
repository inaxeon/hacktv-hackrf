#include "i2s.h"
#include "gpdma.h"

#include <stdio.h>
#include <math.h>

#define CLK_APB1_I2S 204000000 // 204 MHz
#define I2S_DMA_FIFO_LEVEL	4

#define GPDMA_NUMBER_CHANNELS 8
#define DMA_CHANNEL 0

#define NUM_SAMPLES 48

static bool i2s_get_clock_divider(int sample_rate, int word_width, uint16_t *px_div, uint16_t *py_div, uint32_t *pN);
static void i2s_init_dma(int depth);

/* DMA linked list item */
typedef struct {
	uint32_t src;
	uint32_t dest;
	uint32_t next_lli;
	uint32_t control;
} dma_lli;


dma_lli i2s_dma_lli[3];
uint32_t audio[NUM_SAMPLES];
uint32_t transfers_completed;
uint32_t transfers_errored;


char debug_buffer[256];


void i2s_init()
{
	int word_width = 16;
	int sample_rate = 48000;
	uint16_t x_div = 0;
	uint16_t y_div = 0;
	uint32_t clk_n = 0;
	transfers_completed = 0;
	transfers_errored = 0;

	CCU1_CLK_APB1_I2S_CFG = 1;
	CCU1_CLK_M4_DMA_CFG = 3; // RUN+AUTO
	CREG_DMAMUX &= ~CREG_DMAMUX_DMAMUXPER9_MASK;
	CREG_DMAMUX |= CREG_DMAMUX_DMAMUXPER9(1);

	i2s_get_clock_divider(sample_rate, word_width, &x_div, &y_div, &clk_n);

	I2S0_DAO = I2S0_DAO_WORDWIDTH(1) | // 16-bit
		I2S0_DAO_MONO(0) | // Stereo
		I2S0_DAO_WS_SEL(0) | // Master
		I2S0_DAO_WS_HALFPERIOD(word_width - 1) |
		I2S0_DAO_STOP(1) | I2S0_DAO_RESET(1); // WS half-cycle: 16-bits

	I2S0_TXMODE = I2S0_TXMODE_TXCLKSEL(0);

	I2S0_TXBITRATE = (clk_n - 1);
	I2S0_TXRATE = y_div | (x_div << 8);
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

static void i2s_init_dma(int depth)
{
	// Set level of fifo at which to initiate a new transfer
	I2S0_DMA1 &= ~(0x0F << 16); //TODO: libcm3 Macros
	I2S0_DMA1 |= depth << 16;

	I2S0_DMA1 |= I2S0_DMA1_TX_DMA1_ENABLE_MASK;

	//GPDMA STUFF

	// Reset config on all channels
	for (int i = GPDMA_NUMBER_CHANNELS; i > 0; i--) {
		GPDMA_CCONFIG(i - 1) = 0;
	}

	/* Clear all DMA interrupt and error flags */
	GPDMA_INTTCCLEAR = 0xFF;
	GPDMA_INTERRCLR = 0xFF;

	nvic_enable_irq(NVIC_DMA_IRQ);
	nvic_set_priority(NVIC_DMA_IRQ, ((0x01 << 3) | 0x01));

	i2s_dma_lli[0].src = (uint32_t) & (audio[0]);
	i2s_dma_lli[0].dest = (uint32_t) & (I2S0_TXFIFO);
	i2s_dma_lli[0].next_lli = (uint32_t) & (i2s_dma_lli[0]);
	i2s_dma_lli[0].control = GPDMA_CCONTROL_TRANSFERSIZE(NUM_SAMPLES) |
		GPDMA_CCONTROL_SBSIZE(0)   // 1
		| GPDMA_CCONTROL_DBSIZE(0) // 1
		| GPDMA_CCONTROL_SWIDTH(2) // 32-bit word
		| GPDMA_CCONTROL_DWIDTH(2) // 32-bit word
		| GPDMA_CCONTROL_S(0)      // AHB Master 0
		| GPDMA_CCONTROL_D(1)      // AHB Master 1
		| GPDMA_CCONTROL_SI(1)     // increment source
		| GPDMA_CCONTROL_DI(0)     // do not increment destination
		| GPDMA_CCONTROL_PROT1(0)  // user mode
		| GPDMA_CCONTROL_PROT2(0)  // not bufferable
		| GPDMA_CCONTROL_PROT3(0)  // not cacheable
		| GPDMA_CCONTROL_I(1);     // interrupt enabled

	i2s_dma_lli[1].src = i2s_dma_lli[0].src;
	i2s_dma_lli[1].dest = i2s_dma_lli[0].dest;
	i2s_dma_lli[1].next_lli = (uint32_t) & (i2s_dma_lli[1]);
	i2s_dma_lli[1].control = i2s_dma_lli[0].control;

	i2s_dma_lli[2].src = i2s_dma_lli[0].src;
	i2s_dma_lli[2].dest = i2s_dma_lli[0].dest;
	i2s_dma_lli[2].next_lli = (uint32_t) & (i2s_dma_lli[0]);
	i2s_dma_lli[2].control = i2s_dma_lli[0].control;

	gpdma_controller_enable();

	GPDMA_CSRCADDR(DMA_CHANNEL) = i2s_dma_lli[0].src;
	GPDMA_CDESTADDR(DMA_CHANNEL) = i2s_dma_lli[0].dest;
	GPDMA_CLLI(DMA_CHANNEL) = i2s_dma_lli[0].next_lli;
	GPDMA_CCONTROL(DMA_CHANNEL) = i2s_dma_lli[0].control;
	GPDMA_CCONFIG(DMA_CHANNEL) = GPDMA_CCONFIG_DESTPERIPHERAL(0x9) // I2S0_DMA1
		| GPDMA_CCONFIG_FLOWCNTRL(1)               // memory-to-peripheral
		| GPDMA_CCONFIG_H(0)                       // do not halt
		| GPDMA_CCONFIG_ITC(1)
		| GPDMA_CCONFIG_IE(1);

	gpdma_channel_enable(DMA_CHANNEL);

	//sprintf(debug_buffer, "CREG_DMAMUX: 0x%08X, GPDMA_C0CONFIG: 0x%08X, CCU1_CLK_M4_DMA_STAT: 0x%02X", CREG_DMAMUX, GPDMA_C0CONFIG, CCU1_CLK_M4_DMA_STAT);
	//sprintf(debug_buffer, "INTTCSTAT: 0x%02X, INTERRSTAT: 0x%02X RAWINTTCSTAT: 0x%02X", GPDMA_INTTCSTAT, GPDMA_INTERRSTAT, GPDMA_RAWINTTCSTAT);
}

uint8_t i2s_get_tx_level()
{
	return (I2S0_STATE >> 16) & 0x0F;
}

void i2s_tx(uint32_t data)
{
	I2S0_TXFIFO = data;
}

void dma_isr(void)
{
	if (GPDMA_INTSTAT & (1 << DMA_CHANNEL))
	{
		if (GPDMA_INTTCSTAT & (1 << DMA_CHANNEL))
		{
			GPDMA_INTTCCLEAR = (1 << DMA_CHANNEL);
			transfers_completed++;
		}
		if (GPDMA_INTTCSTAT & (1 << DMA_CHANNEL))
		{
			GPDMA_INTERRCLR = (1 << DMA_CHANNEL);
			transfers_errored++;
		}
	}
}

void i2s_generate_test_tone()
{
	int x, y;
	double d;
	int16_t l;
	int sample_rate = 48000;
	int audio_samples;
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

	i2s_init_dma(I2S_DMA_FIFO_LEVEL);
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