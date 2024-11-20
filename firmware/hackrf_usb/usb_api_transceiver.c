/*
 * Copyright 2012-2022 Great Scott Gadgets <info@greatscottgadgets.com>
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "usb_api_transceiver.h"

#include "hackrf_ui.h"
#include "operacake_sctimer.h"

#include <libopencm3/cm3/vector.h>
#include "usb_bulk_buffer.h"
#include "usb_api_m0_state.h"

#include "usb_api_cpld.h" // Remove when CPLD update is handled elsewhere

#include "max2837.h"
#include "max2839.h"
#include "rf_path.h"
#include "tuning.h"
#include "streaming.h"
#include "usb.h"
#include "usb_queue.h"
#include "platform_detect.h"

#include "hackdac.h"
#include "i2s.h"

#include <stddef.h>
#include <string.h>

#include "usb_endpoint.h"
#include "usb_api_sweep.h"

#define USB_TRANSFER_SIZE 0x4000

typedef struct {
	uint32_t freq_mhz;
	uint32_t freq_hz;
} set_freq_params_t;

set_freq_params_t set_freq_params;

struct set_freq_explicit_params {
	uint64_t if_freq_hz; /* intermediate frequency */
	uint64_t lo_freq_hz; /* front-end local oscillator frequency */
	uint8_t path;        /* image rejection filter path */
};

struct set_freq_explicit_params explicit_params;

typedef struct {
	uint32_t freq_hz;
	uint32_t divider;
} set_sample_r_params_t;

set_sample_r_params_t set_sample_r_params;

usb_request_status_t usb_vendor_request_set_baseband_filter_bandwidth(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		const uint32_t bandwidth =
			(endpoint->setup.index << 16) | endpoint->setup.value;
		if (baseband_filter_bandwidth_set(bandwidth)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_freq(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&set_freq_params,
			sizeof(set_freq_params_t),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		const uint64_t freq =
			set_freq_params.freq_mhz * 1000000ULL + set_freq_params.freq_hz;
		if (set_freq(freq)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_sample_rate_frac(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&set_sample_r_params,
			sizeof(set_sample_r_params_t),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		if (sample_rate_frac_set(
			    set_sample_r_params.freq_hz * 2,
			    set_sample_r_params.divider)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_amp_enable(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		switch (endpoint->setup.value) {
		case 0:
			rf_path_set_lna(&rf_path, 0);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			rf_path_set_lna(&rf_path, 1);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_lna_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint8_t value;
		value = max283x_set_lna_gain(&max283x, endpoint->setup.index);
		endpoint->buffer[0] = value;
		if (value) {
			hackrf_ui()->set_bb_lna_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_vga_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint8_t value;
		value = max283x_set_vga_gain(&max283x, endpoint->setup.index);
		endpoint->buffer[0] = value;
		if (value) {
			hackrf_ui()->set_bb_vga_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_txvga_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint8_t value;
		value = max283x_set_txvga_gain(&max283x, endpoint->setup.index);
		endpoint->buffer[0] = value;
		if (value) {
			hackrf_ui()->set_bb_tx_vga_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_antenna_enable(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		switch (endpoint->setup.value) {
		case 0:
			rf_path_set_antenna(&rf_path, 0);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			rf_path_set_antenna(&rf_path, 1);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_freq_explicit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&explicit_params,
			sizeof(struct set_freq_explicit_params),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		if (set_freq_explicit(
			    explicit_params.if_freq_hz,
			    explicit_params.lo_freq_hz,
			    explicit_params.path)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

static volatile hw_sync_mode_t _hw_sync_mode = HW_SYNC_MODE_OFF;
static volatile uint32_t _tx_underrun_limit;
static volatile uint32_t _rx_overrun_limit;

void set_hw_sync_mode(const hw_sync_mode_t new_hw_sync_mode)
{
	_hw_sync_mode = new_hw_sync_mode;
}

volatile transceiver_request_t transceiver_request = {
	.mode = TRANSCEIVER_MODE_OFF,
	.seq = 0,
};

// Must be called from an atomic context (normally USB ISR)
void request_transceiver_mode(transceiver_mode_t mode)
{
	usb_endpoint_flush(&usb_endpoint_bulk_in);
	usb_endpoint_flush(&usb_endpoint_bulk_out);
	usb_endpoint_flush(&usb_endpoint_audio_out);

	transceiver_request.mode = mode;
	transceiver_request.seq++;
}

void transceiver_shutdown(void)
{
	baseband_streaming_disable(&sgpio_config);
	operacake_sctimer_reset_state();

	usb_endpoint_flush(&usb_endpoint_bulk_in);
	usb_endpoint_flush(&usb_endpoint_bulk_out);
	usb_endpoint_flush(&usb_endpoint_audio_out);

	led_off(LED2);
	led_off(LED3);
	rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_OFF);
	m0_set_mode(M0_MODE_IDLE);
}

void transceiver_startup(const transceiver_mode_t mode)
{
	hackrf_ui()->set_transceiver_mode(mode);

	switch (mode) {
	case TRANSCEIVER_MODE_RX_SWEEP:
	case TRANSCEIVER_MODE_RX:
		led_off(LED3);
		led_on(LED2);
		rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_RX);
		m0_set_mode(M0_MODE_RX);
		m0_state.shortfall_limit = _rx_overrun_limit;
		break;
	case TRANSCEIVER_MODE_TX:
		led_off(LED2);
		led_on(LED3);
		rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_TX);
		m0_set_mode(M0_MODE_TX_START);
		m0_state.shortfall_limit = _tx_underrun_limit;
		break;
	default:
		break;
	}

	activate_best_clock_source();
	hw_sync_enable(_hw_sync_mode);
}

usb_request_status_t usb_vendor_request_set_transceiver_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		switch (endpoint->setup.value) {
		case TRANSCEIVER_MODE_OFF:
		case TRANSCEIVER_MODE_RX:
		case TRANSCEIVER_MODE_TX:
		case TRANSCEIVER_MODE_RX_SWEEP:
		case TRANSCEIVER_MODE_CPLD_UPDATE:
			request_transceiver_mode(endpoint->setup.value);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_hw_sync_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		set_hw_sync_mode(endpoint->setup.value);
		usb_transfer_schedule_ack(endpoint->in);
		return USB_REQUEST_STATUS_OK;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_tx_underrun_limit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint32_t value = (endpoint->setup.index << 16) + endpoint->setup.value;
		_tx_underrun_limit = value;
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_rx_overrun_limit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint32_t value = (endpoint->setup.index << 16) + endpoint->setup.value;
		_rx_overrun_limit = value;
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

static volatile uint32_t _sync_bytes_received;
static volatile uint32_t _bytes_until_in_sync;

void transceiver_bulk_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	(void) user_data;
	m0_state.m4_count += bytes_transferred;
}

void transceiver_audio_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	(void) user_data;
	i2s_usb_bytes_transferred += bytes_transferred;

	if (i2s_is_paused())
	{
		// Wait for buffer to re-fill
		if ((i2s_usb_bytes_transferred - i2s_bytes_transferred()) == I2S_BUFFER_SIZE)
			i2s_resume();
	}
}

void transceiver_sync_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	if (_bytes_until_in_sync == HACKDAC_SYNC_AWAITING &&
		*((uint32_t *)(user_data + (sizeof(uint32_t) * 0))) == HACKDAC_SYNC_MAGIC_1 &&
		*((uint32_t *)(user_data + (sizeof(uint32_t) * 1))) == HACKDAC_SYNC_MAGIC_2) {
		_bytes_until_in_sync = *(uint32_t *)(user_data + (sizeof(uint32_t) * 2)) + HACKDAC_SYNC_XFER_SIZE;
	}

	_sync_bytes_received += bytes_transferred;

	if (_bytes_until_in_sync && _bytes_until_in_sync != HACKDAC_SYNC_AWAITING)
		_bytes_until_in_sync -= bytes_transferred;
}

void rx_mode(uint32_t seq)
{
	uint32_t usb_count = 0;

	transceiver_startup(TRANSCEIVER_MODE_RX);

	baseband_streaming_enable(&sgpio_config);

	while (transceiver_request.seq == seq) {
		if ((m0_state.m0_count - usb_count) >= USB_TRANSFER_SIZE) {
			usb_transfer_schedule_block(
				&usb_endpoint_bulk_in,
				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
				USB_TRANSFER_SIZE,
				transceiver_bulk_transfer_complete,
				NULL);
			usb_count += USB_TRANSFER_SIZE;
		}
	}

	transceiver_shutdown();
}

void sync_with_hacktv(uint32_t seq)
{
	uint32_t sync_bytes_scheduled = 0;
	_bytes_until_in_sync = HACKDAC_SYNC_AWAITING;
	_sync_bytes_received = 0;

	while (transceiver_request.seq == seq && _bytes_until_in_sync)
	{
		if ((sync_bytes_scheduled - _sync_bytes_received) == 0)
		{
			// Check _bytes_until_in_sync again to ensure no race condition
			if (_bytes_until_in_sync)  {
				usb_transfer_schedule_block(
					&usb_endpoint_bulk_out,
					&usb_bulk_buffer[0],
					HACKDAC_SYNC_XFER_SIZE,
					transceiver_sync_transfer_complete,
					&usb_bulk_buffer[0]);

				sync_bytes_scheduled += HACKDAC_SYNC_XFER_SIZE;
			}
		}
	}
}

void tx_mode(uint32_t seq)
{
	unsigned int usb_count = 0;
	unsigned int usb_audio_count = 0;
	bool started = false;
	bool audio_started = false;
	bool async_audio = false;
	bool sync_audio = true;

	transceiver_startup(TRANSCEIVER_MODE_TX);
	
	if (sync_audio || async_audio) {
		i2s_startup(sync_audio);
	}

	if (sync_audio) {
		// Eat random quantity of zeros sent by PC until proper data from hacktv arrives.
		// failure to do so would result in audio/video data sequencing failure.
		sync_with_hacktv(seq);
	}

	// Set up OUT transfer of buffer 0.
	usb_transfer_schedule_block(
		&usb_endpoint_bulk_out,
		&usb_bulk_buffer[0x0000],
		USB_TRANSFER_SIZE,
		transceiver_bulk_transfer_complete,
		NULL);
	usb_count += USB_TRANSFER_SIZE;

	if (async_audio || sync_audio) {
		// Get first audio buffer
		usb_transfer_schedule_block(
			&usb_endpoint_audio_out,
			&i2s_audio_buffer[0x0000],
			I2S_USB_TRANSFER_SIZE,
			transceiver_audio_transfer_complete,
			NULL);
		usb_audio_count += I2S_USB_TRANSFER_SIZE;
	}

	while (transceiver_request.seq == seq) {
		if (!started && ((m0_state.m4_count == USB_BULK_BUFFER_SIZE))) {
			// Buffer is now full, start streaming.
			baseband_streaming_enable(&sgpio_config);
			video_led_on();
			started = true;
		}

		if ((sync_audio || async_audio) && !audio_started && (i2s_usb_bytes_transferred == (I2S_BUFFER_SIZE - I2S_USB_TRANSFER_SIZE))) {
			i2s_streaming_enable(); // Start audio
			audio_started = true;
		}

		if ((usb_count - m0_state.m0_count) <= (USB_BULK_BUFFER_SIZE - USB_TRANSFER_SIZE)) {
			usb_transfer_schedule_block(
				&usb_endpoint_bulk_out,
				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
				USB_TRANSFER_SIZE,
				transceiver_bulk_transfer_complete,
				NULL);
			usb_count += USB_TRANSFER_SIZE;
			if (sync_audio) {
				usb_transfer_schedule_block(
					&usb_endpoint_bulk_out,
					&i2s_audio_buffer[usb_audio_count & I2S_BUFFER_MASK],
					I2S_USB_TRANSFER_SIZE,
					transceiver_audio_transfer_complete,
					NULL);
				usb_audio_count += I2S_USB_TRANSFER_SIZE;
			}
		}
		if (async_audio && ((usb_audio_count - i2s_bytes_transferred()) <= (I2S_BUFFER_SIZE - I2S_USB_TRANSFER_SIZE))) {
			usb_transfer_schedule_block(
				&usb_endpoint_audio_out,
				&i2s_audio_buffer[usb_audio_count & I2S_BUFFER_MASK],
				I2S_USB_TRANSFER_SIZE,
				transceiver_audio_transfer_complete,
				NULL);
			usb_audio_count += I2S_USB_TRANSFER_SIZE;
		}
	}

	if (async_audio || sync_audio) {
		i2s_shutdown();
	}

	video_led_off();
	transceiver_shutdown();
}

void off_mode(uint32_t seq)
{
	hackrf_ui()->set_transceiver_mode(TRANSCEIVER_MODE_OFF);

	while (transceiver_request.seq == seq) {}
}
