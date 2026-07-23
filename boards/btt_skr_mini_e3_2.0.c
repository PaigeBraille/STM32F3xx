/*

  btt_skr_mini_e3_2.0.c - driver code for STM32F3xx ARM processors

  Trinamic TMC2209 UART interface over the SERIAL1 stream (USART2).
  Ported from the grblHAL STM32F1xx driver.

  Part of grblHAL

  Copyright (c) 2021-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#ifdef BOARD_BTT_SKR_MINI_E3_V20

#include <string.h>

#include "serial.h"

static io_stream_t tmc_uart;

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *dgr)
{
    static TMC_uart_write_datagram_t wdgr = {0};
    volatile uint32_t dly = 50, ms = hal.get_elapsed_ticks();

    tmc_uart.write_n(dgr->data, sizeof(TMC_uart_read_datagram_t));

    while(tmc_uart.get_tx_buffer_count());

    while(--dly);

    tmc_uart.reset_read_buffer();

    while(tmc_uart.get_rx_buffer_count() < 8) {
        if(hal.get_elapsed_ticks() - ms >= 3)
            break;
    }

    if(tmc_uart.get_rx_buffer_count() >= 8) {
        wdgr.data[0] = tmc_uart.read();
        wdgr.data[1] = tmc_uart.read();
        wdgr.data[2] = tmc_uart.read();
        wdgr.data[3] = tmc_uart.read();
        wdgr.data[4] = tmc_uart.read();
        wdgr.data[5] = tmc_uart.read();
        wdgr.data[6] = tmc_uart.read();
        wdgr.data[7] = tmc_uart.read();
    } else
        wdgr.msg.addr.value = 0xFF;

    dly = 5000;
    while(--dly);

    return &wdgr;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    tmc_uart.write_n(dgr->data, sizeof(TMC_uart_write_datagram_t));

    while(tmc_uart.get_tx_buffer_count());
}

void board_init (void)
{
    io_stream_t const *stream;

    // SAY WHICH IT IS. This fallback to a null stream was SILENT, and a null stream
    // swallows every Trinamic write without complaint - so a failure to claim the UART
    // looked identical, from outside, to a broken wire. That cost a full day of scoping
    // a line that the firmware may never have driven in the first place (23 Jul).
    //
    // Now it reports on every boot, before any driver comms is attempted:
    //   "Trinamic UART: claimed instance N" -> firmware side is fine, so silence on the
    //                                         wire is genuinely a hardware fault.
    //   "Trinamic UART: FAILED to claim ..." -> nothing was ever transmitted, and no
    //                                         amount of probing will find anything.
    if((stream = stream_open_instance(TRINAMIC_STREAM, 115200, NULL, "Trinamic UART")) == NULL) {
        stream = stream_null_init(115200);
        report_message("Trinamic UART: FAILED to claim stream - driver comms is DEAD in firmware, not wiring",
                        Message_Warning);
    } else
        report_message("Trinamic UART: claimed stream OK", Message_Info);

    memcpy(&tmc_uart, stream, sizeof(io_stream_t));
    tmc_uart.disable_rx(true);
    tmc_uart.set_enqueue_rt_handler(stream_buffer_all);
}

#endif
