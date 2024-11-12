#ifndef __I2S_H__
#define __I2S_H__

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/i2s.h>
#include <libopencm3/lpc43xx/creg.h>

#define I2S_DMA_CHANNEL         0
#define I2S_NUM_BUFFERS			2
#define I2S_BUFFER_DEPTH		1024 // 32-bit words (each one 16-bit L+R sample)
#define I2S_USB_TRANSFER_SIZE	(I2S_BUFFER_DEPTH * sizeof(uint32_t))
#define I2S_BUFFER_MASK			((I2S_USB_TRANSFER_SIZE * I2S_NUM_BUFFERS) - 1)

void i2s_init();
void i2s_streaming_enable();
void i2s_shutdown();
void i2s_startup();
void i2s_mute(bool mute);
void i2s_generate_test_tone();
void i2s_gpdma_isr();
int32_t i2s_bytes_transferred();

extern uint32_t i2s_audio_buffer[];
extern int32_t i2s_usb_bytes_transferred;

#endif /*__I2S_H__*/
