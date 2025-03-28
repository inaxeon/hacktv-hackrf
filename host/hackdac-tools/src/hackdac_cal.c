/*
 * Copyright 2024 Matthew Millman (github: inaxeon)
 *
 * This file is part of HackTV.
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

 #define _FILE_OFFSET_BITS 64

 #include <hackrf.h>
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <stdbool.h>
 #include <getopt.h>
 #include <string.h>
 #include <fcntl.h>
 #include <errno.h>
 #include <unistd.h>
 
 #ifdef _WIN32
 #include <windows.h>
 #include <conio.h>
 #else
 #include <sys/ioctl.h>
 #include <termios.h>
 #endif /* _WIN32 */
 
 #define HACKDAC_ONEVOLT                        32767
 #define HACKDAC_ZEROVOLT                       0
 #define HACKDAC_MINUSONEVOLT                   -32768
 
 #define HACKDAC_SAMPLE_RATE                    13500000
 
 // hackrf_set_hackdac_mode() value flags
 #define HACKDAC_MODE_RF                        0x00
 #define HACKDAC_MODE_BASEBAND                  (1 << 7)
 #define HACKDAC_RFFC5071_HIJACK                (1 << 6)
 // NOTE: bit 0 is reserved (HW sync enable)
 // Audio mode field
 #define HACKDAC_AUDIO_MODE_SHIFT               (1)
 #define HACKDAC_AUDIO_MODE_MASK                (0x3 << HACKDAC_AUDIO_MODE_SHIFT)
 #define HACKDAC_AUDIO_MODE(x)                  ((x) << HACKDAC_AUDIO_MODE_SHIFT)
 // Audio mode values
 #define HACKDAC_NO_AUDIO                       0 // No audio. Don't send any audio data to HackRF by any means.
 #define HACKDAC_SYNC_AUDIO                     1 // Audio interleaved with video data
 #define HACKDAC_ASYNC_AUDIO                    2 // Audio transferred separately through USB_AUDIO_OUT_EP
 
 #define HACKDAC_REG_START                      0x20
 
 
 #define MCP47FEBXX_CMD_DISABLE_CFG_BIT         0x02
 #define MCP47FEBXX_CMD_ENABLE_CFG_BIT          0x04
 #define MCP47FEBXX_CMD_READ                    0x06
 #define MCP47FEBXX_CMD_WRITE                   0x00
 
 #define MCP47FEBXX_VOLATILE_DAC0               0x00
 #define MCP47FEBXX_VOLATILE_DAC1               0x01
 #define MCP47FEBXX_NONVOLATILE_DAC0            0x10
 #define MCP47FEBXX_NONVOLATILE_DAC1            0x11
 #define MCP47FEBXX_GAINCTRL_SLAVEADDR          0x1A
 
 #define MCP47FEBXX_RANGE                       4096
 
 #define hackrf_set_hackdac_mode(dev, value) hackrf_set_hw_sync_mode(dev, value);
 
 typedef enum
 {
     InvalidCmd,
     NoCmd,
     SetOneVolt,
     SetZeroVolt,
     SetMinusOneVolt,
     OffsetUp,
     OffsetDown,
     GainUp,
     GainDown,
     ToggleSync,
     CoarseAdjust,
     FineAdjust,
     Print,
     Write,
     Reset,
     Quit
 } test_cmd_t;
 
 int16_t dac_value;
 bool sync_bit;
 
 void usage()
 {
	printf(
		"\n"
		"Usage: hackdac_cal [options]\n"
		"\n"
		"  -d <serial>          Specify serial number of hackrf device to connect to\n"
		"\n"
	);
 }
 
 bool mcp47febxx_read(hackrf_device* device, uint8_t reg, uint16_t *value)
 {
     int result = hackrf_rffc5071_read(device, reg, value);
 
     if (result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_rffc5071_read() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         return false;
     }
 
     return true;
 }
 
 bool mcp47febxx_write(hackrf_device* device, uint8_t reg, uint16_t value)
 {
     int result = hackrf_rffc5071_write(device, reg, value);
 
     if (result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_rffc5071_write() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         return false;
     }
 
     return true;
 }
 
 #ifndef _WIN32
 
 bool posix_kbhit()
 {
     int byteswaiting;
     ioctl(0, FIONREAD, &byteswaiting);
     return byteswaiting > 0;
 }
 
 void terminal_set_raw_mode()
 {
     struct termios term;
     tcgetattr(STDIN_FILENO, &term);
     term.c_lflag &= ~(ICANON | ECHO); // Disable echo as well
     tcsetattr(STDIN_FILENO, TCSANOW, &term);
 }
 
 void terminal_unset_raw_mode()
 {
     struct termios term;
     tcgetattr(STDIN_FILENO, &term);
     term.c_lflag |= ICANON | ECHO;
     tcsetattr(STDIN_FILENO, TCSANOW, &term);
 }
 
 #define _kbhit posix_kbhit
 #define getch getchar
 
 #endif /* !_WIN32 */
 
 void adjust_cal(uint16_t *value, bool up, bool coarse)
 {
     int32_t new_value = *value;
 
     if (up)
         new_value += coarse ? 10 : 1;
     else
         new_value -= coarse ? 10 : 1;
 
     if (new_value < 0)
         new_value = 0;
     
     if (new_value > (MCP47FEBXX_RANGE - 1))
         new_value = (MCP47FEBXX_RANGE - 1);
 
     *value = new_value;
 }
 
 test_cmd_t get_cmd(bool non_block)
 {
     if (non_block && !_kbhit())
         return NoCmd;
 
     switch (getch())
     {
#ifdef _WIN32
     case 0xE0:
        switch (getch())
        {
        case 'H':
            return SetOneVolt;
        case 'P':
            return SetMinusOneVolt;
        case 'M':
        case 'K':
            return SetZeroVolt;
        default:
            return InvalidCmd;
        }
        break;
#else
     case 0x1B:
         switch (getch())
         {
             case '[':
             char c = getch();
                 switch (c)
                 {
                     case 'A':
                         return SetOneVolt;
                     case 'B':
                         return SetMinusOneVolt;
                     case 'C':
                     case 'D':
                         return SetZeroVolt;
                     case '5':
                         return OffsetUp;
                     case '6':
                         return OffsetDown;
                     default:
                         return InvalidCmd;
                 }
                 break;
             default:
                 return InvalidCmd;
         }
         break;
#endif /* ! _WIN32 */
     case '+':
         return GainUp;
     case '-':
         return GainDown;
     case 's':
     case 'S':
         return ToggleSync;
     case 'c':
     case 'C':
         return CoarseAdjust;
     case 'f':
     case 'F':
         return FineAdjust;
     case 'p':
     case 'P':
         return Print;
     case 'r':
     case 'R':
         return Reset;
     case 'w':
     case 'W':
         return Write;
     case 'q':
     case 'Q':
     case 0x03:
         return Quit;
     default:
         return InvalidCmd;
     }
 }
 
 bool cal_read(hackrf_device *device, uint16_t *offset, uint16_t *gain)
 {
     if (!mcp47febxx_read(device, MCP47FEBXX_VOLATILE_DAC0, offset))
         return false;
 
     if (!mcp47febxx_read(device, MCP47FEBXX_VOLATILE_DAC1, gain))
         return false;
 
     return true;
 }
 
 bool cal_write(hackrf_device *device, uint16_t offset, uint16_t gain)
 {
     if (!mcp47febxx_write(device, MCP47FEBXX_NONVOLATILE_DAC0, offset))
         return false;
 
     if (!mcp47febxx_write(device, MCP47FEBXX_NONVOLATILE_DAC1, gain))
         return false;
 
     return true;
 }
 
 void cal_print(const char *prefix, uint16_t offset, uint16_t gain)
 {
     printf(
         "\n%s calibration:\n"
         "Offset : %d / %u\n"
         "Gain   : %d / %u\n",
         prefix, offset, MCP47FEBXX_RANGE, gain, MCP47FEBXX_RANGE
     );
 }
 
 void cal_run(hackrf_device *device, uint16_t offset, uint16_t gain)
 {
     bool coarse = false;
 
     printf("\nHackDAC Calibrator Commands:\n\n");
     printf("Up arrow            : Set output to 1V\n");
     printf("Down arrow          : Set output to -1V\n");
     printf("Left/Right arrow    : Set output to 0V\n");
     printf("+/-                 : Increase/decrease gain\n");
     printf("Page up / Page down : Increase/decrease offset\n");
     printf("S                   : Toggle sync output\n");
     printf("C                   : Coarse adjustment\n");
     printf("F                   : Fine adjustment\n");
     printf("R                   : Reset calibration\n");
     printf("P                   : Print calibration\n");
     printf("W                   : Write calibrations to non-volatile memory\n");
     printf("Q                   : Quit application\n\n");
     printf("Ensure HackDAC output is terminated with a precision 75 ohm load during calibration.\n");
     printf("No further output to follow.\n");
 
 #ifndef _WIN32
     terminal_set_raw_mode();
 #endif /* _WIN32 */
 
     dac_value = HACKDAC_ZEROVOLT;
     sync_bit = false;
 
     while (1) {
         test_cmd_t cmd = get_cmd(true);
         switch (cmd)
         {
             case SetOneVolt:
                 dac_value = HACKDAC_ONEVOLT;
                 break;
             case SetMinusOneVolt:
                 dac_value = HACKDAC_MINUSONEVOLT;
                 break;
             case SetZeroVolt:
                 dac_value = HACKDAC_ZEROVOLT;
                 break;
             case GainUp:
                 adjust_cal(&gain, true, coarse);
                 mcp47febxx_write(device, MCP47FEBXX_VOLATILE_DAC1, gain);
                 break;
             case GainDown:
                 adjust_cal(&gain, false, coarse);
                 mcp47febxx_write(device, MCP47FEBXX_VOLATILE_DAC1, gain);
                 break;
             case OffsetUp:
                 adjust_cal(&offset, false, coarse);
                 mcp47febxx_write(device, MCP47FEBXX_VOLATILE_DAC0, offset);
                 break;
             case OffsetDown:
                 adjust_cal(&offset, true, coarse);
                 mcp47febxx_write(device, MCP47FEBXX_VOLATILE_DAC0, offset);
                 break;
             case ToggleSync:
                 sync_bit = !sync_bit;
                 break;
             case CoarseAdjust:
                 coarse = true;
                 break;
             case FineAdjust:
                 coarse = false;
                 break;
             case Print:
                 cal_print("Current", offset, gain);
                 break;
             case Reset:
                 gain = (MCP47FEBXX_RANGE / 2) - 1;
                 offset = (MCP47FEBXX_RANGE / 2) - 1;
                 mcp47febxx_write(device, MCP47FEBXX_VOLATILE_DAC1, gain);
                 mcp47febxx_write(device, MCP47FEBXX_VOLATILE_DAC0, offset);
                 break;
             case Write:
                 cal_write(device, offset, gain);
                 cal_print("Wrote", offset, gain);
                 break;
             case NoCmd:
             case InvalidCmd:
                 continue;
             case Quit:
                 goto out;
                 return;
         }
     }
 
 out:
 #ifndef _WIN32
     terminal_unset_raw_mode();
 #endif /* _WIN32 */
 }
 
 int tx_callback(hackrf_transfer *transfer)
 {
     uint8_t *buf = transfer->buffer;
     int i;
 
     for (i = 0; i < transfer->valid_length; i += 2)
     {
         buf[i + 0] = (dac_value >> 1) & 0xFF;
         buf[i + 1] = ((dac_value >> 9) & 0x7F) | (sync_bit << 7);
     }
 
     return 0;
 }
 
 int main(int argc, char** argv)
 {
     int opt;
     int result = HACKRF_SUCCESS;
     uint16_t offset;
     uint16_t gain;
     hackrf_device* device;
 
     const char* serial_number = NULL;
 
     while ((opt = getopt(argc, argv, "d:")) != EOF) {
         result = HACKRF_SUCCESS;
 
         switch (opt) {
         case 'd':
             serial_number = optarg;
             break;
         case 'h':
         case '?':
             usage();
             return EXIT_FAILURE;
         }
     }
 
     result = hackrf_init();
     if (result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_init() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         return EXIT_FAILURE;
     }
 
     result = hackrf_open_by_serial(serial_number, &device);
     if (result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_open() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         usage();
         return EXIT_FAILURE;
     }
 
     result = hackrf_set_hackdac_mode(device, HACKDAC_MODE_BASEBAND | HACKDAC_RFFC5071_HIJACK | HACKDAC_AUDIO_MODE(HACKDAC_NO_AUDIO));
     if(result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_set_hackdac_mode() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         return EXIT_FAILURE;
     }
 
     if (!cal_read(device, &offset, &gain)) {
         fprintf(stderr,
             "failed to dump current calibration registers\n");
             return(EXIT_FAILURE);
     }
 
     cal_print("Current", offset, gain);
 
     result = hackrf_start_tx(device, tx_callback, NULL);
     if(result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_start_tx() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         return(EXIT_FAILURE);
     }
 
     cal_run(device, offset, gain);
 
     result = hackrf_stop_tx(device);
     if(result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_stop_tx() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
         return(EXIT_FAILURE);
     }
 
     result = hackrf_close(device);
     if (result != HACKRF_SUCCESS) {
         fprintf(stderr,
             "hackrf_close() failed: %s (%d)\n",
             hackrf_error_name(result),
             result);
     }
 
     hackrf_exit();
 
     return EXIT_SUCCESS;
 }