#ifndef __I2S_H__
#define __I2S_H__

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/i2s.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/m4/nvic.h>


void i2s_init();
void i2s_start();
void i2s_stop();
void i2s_mute(bool mute);
void i2s_generate_test_tone();

#endif /*__I2S_H__*/
