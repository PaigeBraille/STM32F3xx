/*

  serial.c - serial port implementation for STM32F3xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2026 Terje Io

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

#include <string.h>

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"

// Default to USART3 on PB10/PB11, the original fixed assignment for this
// driver.
#ifndef SERIAL_PORT
#define SERIAL_PORT 3
#endif

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command;

#ifdef SERIAL1_PORT
static stream_rx_buffer_t rxbuf1 = {0};
static stream_tx_buffer_t txbuf1 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command1;
static const io_stream_t *serial1Init(uint32_t baud_rate);
#else
#define SERIAL1_PORT 0
#endif

#if SERIAL_PORT == 1
#define UART0 USART1
#define UART0_IRQ USART1_IRQn
#define UART0_IRQHandler USART1_IRQHandler
#define UART0_CLK_ENABLE __HAL_RCC_USART1_CLK_ENABLE
#define UART0_GPIO_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE
#define UART0_CLK HAL_RCC_GetPCLK2Freq
#define UART0_TX_PORT GPIOA
#define UART0_TX_PIN 9
#define UART0_RX_PORT GPIOA
#define UART0_RX_PIN 10
#define UART0_AF GPIO_AF7_USART1
#elif SERIAL_PORT == 2
#define UART0 USART2
#define UART0_IRQ USART2_IRQn
#define UART0_IRQHandler USART2_IRQHandler
#define UART0_CLK_ENABLE __HAL_RCC_USART2_CLK_ENABLE
#define UART0_GPIO_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE
#define UART0_CLK HAL_RCC_GetPCLK1Freq
#define UART0_TX_PORT GPIOA
#define UART0_TX_PIN 2
#define UART0_RX_PORT GPIOA
#define UART0_RX_PIN 3
#define UART0_AF GPIO_AF7_USART2
#elif SERIAL_PORT == 3
#define UART0 USART3
#define UART0_IRQ USART3_IRQn
#define UART0_IRQHandler USART3_IRQHandler
#define UART0_CLK_ENABLE __HAL_RCC_USART3_CLK_ENABLE
#define UART0_GPIO_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define UART0_CLK HAL_RCC_GetPCLK1Freq
#define UART0_TX_PORT GPIOB
#define UART0_TX_PIN 10
#define UART0_RX_PORT GPIOB
#define UART0_RX_PIN 11
#define UART0_AF GPIO_AF7_USART3
#else
#error Code has to be added to support serial port
#endif

#if SERIAL1_PORT

#if SERIAL1_PORT == SERIAL_PORT
#error Conflicting use of UART peripherals!
#endif

#if SERIAL1_PORT == 1
#define UART1 USART1
#define UART1_IRQ USART1_IRQn
#define UART1_IRQHandler USART1_IRQHandler
#define UART1_CLK_ENABLE __HAL_RCC_USART1_CLK_ENABLE
#define UART1_GPIO_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE
#define UART1_CLK HAL_RCC_GetPCLK2Freq
#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN 9
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN 10
#define UART1_AF GPIO_AF7_USART1
#elif SERIAL1_PORT == 2
#define UART1 USART2
#define UART1_IRQ USART2_IRQn
#define UART1_IRQHandler USART2_IRQHandler
#define UART1_CLK_ENABLE __HAL_RCC_USART2_CLK_ENABLE
#define UART1_GPIO_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE
#define UART1_CLK HAL_RCC_GetPCLK1Freq
#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN 2
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN 3
#define UART1_AF GPIO_AF7_USART2
#elif SERIAL1_PORT == 3
#define UART1 USART3
#define UART1_IRQ USART3_IRQn
#define UART1_IRQHandler USART3_IRQHandler
#define UART1_CLK_ENABLE __HAL_RCC_USART3_CLK_ENABLE
#define UART1_GPIO_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define UART1_CLK HAL_RCC_GetPCLK1Freq
#define UART1_TX_PORT GPIOB
#define UART1_TX_PIN 10
#define UART1_RX_PORT GPIOB
#define UART1_RX_PIN 11
#define UART1_AF GPIO_AF7_USART3
#else
#error Code has to be added to support serial port 1
#endif

#endif // SERIAL1_PORT

static bool uart_release(uint8_t instance);
static const io_stream_status_t *get_uart_status(uint8_t instance);

static io_stream_status_t stream_status[] = {
    {.baud_rate = 115200,
     .format =
         {
             .width = Serial_8bit,
             .stopbits = Serial_StopBits1,
             .parity = Serial_ParityNone,
         }},
#if SERIAL1_PORT
    {.baud_rate = 115200,
     .format =
         {
             .width = Serial_8bit,
             .stopbits = Serial_StopBits1,
             .parity = Serial_ParityNone,
         }},
#endif
};

static io_stream_properties_t serial[] = {{.type = StreamType_Serial,
                                           .instance = 0,
                                           .flags.claimable = On,
                                           .flags.claimed = Off,
                                           .flags.can_set_baud = On,
                                           .flags.modbus_ready = On,
                                           .claim = serialInit,
                                           .release = uart_release,
                                           .get_status = get_uart_status},
#if SERIAL1_PORT
                                          {.type = StreamType_Serial,
                                           .instance = 1,
                                           .flags.claimable = On,
                                           .flags.claimed = Off,
                                           .flags.can_set_baud = On,
                                           .flags.modbus_ready = On,
                                           .claim = serial1Init,
                                           .release = uart_release,
                                           .get_status = get_uart_status}
#endif
};

void serialRegisterStreams(void) {
  static io_stream_details_t streams = {
      .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
      .streams = serial,
  };

  static const periph_pin_t tx0 = {.function = Output_TX,
                                   .group = PinGroup_UART1,
                                   .port = UART0_TX_PORT,
                                   .pin = UART0_TX_PIN,
                                   .mode = {.mask = PINMODE_OUTPUT}};

  static const periph_pin_t rx0 = {.function = Input_RX,
                                   .group = PinGroup_UART1,
                                   .port = UART0_RX_PORT,
                                   .pin = UART0_RX_PIN,
                                   .mode = {.mask = PINMODE_NONE}};

  hal.periph_port.register_pin(&rx0);
  hal.periph_port.register_pin(&tx0);

#if SERIAL1_PORT

  static const periph_pin_t tx1 = {.function = Output_TX,
                                   .group = PinGroup_UART2,
                                   .port = UART1_TX_PORT,
                                   .pin = UART1_TX_PIN,
                                   .mode = {.mask = PINMODE_OUTPUT},
                                   .description = "UART1"};

  static const periph_pin_t rx1 = {.function = Input_RX,
                                   .group = PinGroup_UART2,
                                   .port = UART1_RX_PORT,
                                   .pin = UART1_RX_PIN,
                                   .mode = {.mask = PINMODE_NONE},
                                   .description = "UART1"};

  hal.periph_port.register_pin(&rx1);
  hal.periph_port.register_pin(&tx1);

#endif // SERIAL1_PORT

  stream_register_streams(&streams);
}

static const io_stream_status_t *get_uart_status(uint8_t instance) {
  stream_status[instance].flags = serial[instance].flags;

  return &stream_status[instance];
}

static bool uart_release(uint8_t instance) {
  bool ok;

  if ((ok = serial[instance].flags.claimed))
    serial[instance].flags.claimed = Off;

  return ok;
}

#ifdef RS485_DIR_PORT

static void rs485SetDirection(bool tx) {
  DIGITAL_OUT(RS485_DIR_PORT, RS485_DIR_PIN, tx);
}

#endif // RS485_DIR_PORT

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree(void) {
  uint16_t tail = rxbuf.tail, head = rxbuf.head;

  return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount(void) {
  uint32_t tail = rxbuf.tail, head = rxbuf.head;

  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush(void) { rxbuf.tail = rxbuf.head; }

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel(void) {
  rxbuf.data[rxbuf.head] = ASCII_CAN;
  rxbuf.tail = rxbuf.head;
  rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool serialPutC(const uint8_t c) {
  uint16_t next_head =
      BUFNEXT(txbuf.head, txbuf); // Get pointer to next free slot in buffer

  while (txbuf.tail == next_head) {      // While TX buffer full
    if (!hal.stream_blocking_callback()) // check if blocking for space,
      return false; // exit if not (leaves TX buffer in an inconsistent state)
  }

  txbuf.data[txbuf.head] = c;    // Add data to buffer,
  txbuf.head = next_head;        // update head pointer and
  UART0->CR1 |= USART_CR1_TXEIE; // enable TX interrupts

  return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer
// full
//
static void serialWriteS(const char *s) {
  uint8_t c, *ptr = (uint8_t *)s;

  while ((c = *ptr++) != '\0')
    serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream, blocks
// if buffer full
//
static void serialWrite(const uint8_t *s, uint16_t length) {
  uint8_t *ptr = (uint8_t *)s;

  while (length--)
    serialPutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serialTxFlush(void) {
  UART0->CR1 &= ~USART_CR1_TXEIE; // Disable TX interrupts
  txbuf.tail = txbuf.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serialTxCount(void) {
  uint32_t tail = txbuf.tail, head = txbuf.head;

  return BUFCOUNT(head, tail, TX_BUFFER_SIZE) +
         (UART0->ISR & USART_ISR_TC ? 0 : 1);
}

//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC(void) {
  uint_fast16_t tail = rxbuf.tail; // Get buffer pointer

  if (tail == rxbuf.head)
    return -1; // no data available

  int32_t data = (int32_t)rxbuf.data[tail]; // Get next character
  rxbuf.tail = BUFNEXT(tail, rxbuf);        // and update pointer

  return data;
}

static bool serialSuspendInput(bool suspend) {
  return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate(uint32_t baud_rate) {
  stream_status[0].baud_rate = baud_rate;

  UART0->CR1 = USART_CR1_RE | USART_CR1_TE;
  UART0->BRR = UART_DIV_SAMPLING16(UART0_CLK(), baud_rate);
  UART0->CR1 |= (USART_CR1_UE | USART_CR1_RXNEIE);

  return true;
}

static bool serialDisable(bool disable) {
  if (disable)
    UART0->CR1 &= ~USART_CR1_RXNEIE;
  else
    UART0->CR1 |= USART_CR1_RXNEIE;

  return true;
}

static bool serialEnqueueRtCommand(uint8_t c) {
  return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr
serialSetRtHandler(enqueue_realtime_command_ptr handler) {
  enqueue_realtime_command_ptr prev = enqueue_realtime_command;

  if (handler)
    enqueue_realtime_command = handler;

  return prev;
}

const io_stream_t *serialInit(uint32_t baud_rate) {
  static const io_stream_t stream = {
      .type = StreamType_Serial,
      .is_connected = stream_connected,
      .read = serialGetC,
      .write = serialWriteS,
      .write_char = serialPutC,
      .write_n = serialWrite,
      .enqueue_rt_command = serialEnqueueRtCommand,
      .get_rx_buffer_free = serialRxFree,
      .get_rx_buffer_count = serialRxCount,
      .get_tx_buffer_count = serialTxCount,
      .reset_write_buffer = serialTxFlush,
      .reset_read_buffer = serialRxFlush,
      .cancel_read_buffer = serialRxCancel,
      .disable_rx = serialDisable,
      .set_baud_rate = serialSetBaudRate,
      .suspend_read = serialSuspendInput,
#if MODBUS_RTU_STREAM == 0 && defined(RS485_DIR_PORT)
      .set_direction = rs485SetDirection,
#endif
      .set_enqueue_rt_handler = serialSetRtHandler};

  if (!serial[0].flags.claimable || serial[0].flags.claimed)
    return NULL;

  serial[0].flags.claimed = On;

  if (!serial[0].flags.init_ok) {

    UART0_GPIO_CLK_ENABLE();
    UART0_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {.Mode = GPIO_MODE_AF_PP,
                                           .Pull = GPIO_NOPULL,
                                           .Speed = GPIO_SPEED_FREQ_HIGH,
                                           .Pin = (1 << UART0_TX_PIN),
                                           .Alternate = UART0_AF};
    HAL_GPIO_Init(UART0_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = (1 << UART0_RX_PIN);
    HAL_GPIO_Init(UART0_RX_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(UART0_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(UART0_IRQ);

    serial[0].flags.init_ok = On;
  }

  stream_set_defaults(&stream, baud_rate);

  return &stream;
}

void UART0_IRQHandler(void) {
  if (UART0->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    UART0->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;

  if (UART0->ISR & USART_ISR_RXNE) {
    uint8_t data = UART0->RDR;
    if (!enqueue_realtime_command(
            data)) { // Check and strip realtime commands...
      uint16_t next_head =
          BUFNEXT(rxbuf.head, rxbuf); // Get and increment buffer pointer
      if (next_head == rxbuf.tail)    // If buffer full
        rxbuf.overflow = 1;           // flag overflow
      else {
        rxbuf.data[rxbuf.head] = data; // if not add data to buffer
        rxbuf.head = next_head;        // and update pointer
      }
    }
  }

  if ((UART0->ISR & USART_ISR_TXE) && (UART0->CR1 & USART_CR1_TXEIE)) {
    uint_fast16_t tail = txbuf.tail;          // Get buffer pointer
    UART0->TDR = txbuf.data[tail];            // Send next character
    txbuf.tail = tail = BUFNEXT(tail, txbuf); // and increment pointer
    if (tail == txbuf.head)                   // If buffer empty then
      UART0->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
  }
}

#if SERIAL1_PORT

//
// Returns number of free characters in serial1 input buffer
//
static uint16_t serial1RxFree(void) {
  uint16_t tail = rxbuf1.tail, head = rxbuf1.head;

  return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial1 input buffer
//
static uint16_t serial1RxCount(void) {
  uint32_t tail = rxbuf1.tail, head = rxbuf1.head;

  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial1 input buffer
//
static void serial1RxFlush(void) { rxbuf1.tail = rxbuf1.head; }

//
// Flushes and adds a CAN character to the serial1 input buffer
//
static void serial1RxCancel(void) {
  rxbuf1.data[rxbuf1.head] = ASCII_CAN;
  rxbuf1.tail = rxbuf1.head;
  rxbuf1.head = BUFNEXT(rxbuf1.head, rxbuf1);
}

//
// Writes a character to the serial1 output stream
//
static bool serial1PutC(const uint8_t c) {
  uint16_t next_head =
      BUFNEXT(txbuf1.head, txbuf1); // Get pointer to next free slot in buffer

  while (txbuf1.tail == next_head) {     // While TX buffer full
    if (!hal.stream_blocking_callback()) // check if blocking for space,
      return false; // exit if not (leaves TX buffer in an inconsistent state)
  }

  txbuf1.data[txbuf1.head] = c;  // Add data to buffer,
  txbuf1.head = next_head;       // update head pointer and
  UART1->CR1 |= USART_CR1_TXEIE; // enable TX interrupts

  return true;
}

//
// Writes a null terminated string to the serial1 output stream, blocks if
// buffer full
//
static void serial1WriteS(const char *s) {
  uint8_t c, *ptr = (uint8_t *)s;

  while ((c = *ptr++) != '\0')
    serial1PutC(c);
}

//
// Writes a number of characters from string to the serial1 output stream,
// blocks if buffer full
//
static void serial1Write(const uint8_t *s, uint16_t length) {
  uint8_t *ptr = (uint8_t *)s;

  while (length--)
    serial1PutC(*ptr++);
}

//
// Flushes the serial1 output buffer
//
static void serial1TxFlush(void) {
  UART1->CR1 &= ~USART_CR1_TXEIE; // Disable TX interrupts
  txbuf1.tail = txbuf1.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial1TxCount(void) {
  uint32_t tail = txbuf1.tail, head = txbuf1.head;

  return BUFCOUNT(head, tail, TX_BUFFER_SIZE) +
         (UART1->ISR & USART_ISR_TC ? 0 : 1);
}

//
// serial1GetC - returns -1 if no data available
//
static int32_t serial1GetC(void) {
  uint_fast16_t tail = rxbuf1.tail; // Get buffer pointer

  if (tail == rxbuf1.head)
    return -1; // no data available

  int32_t data = (int32_t)rxbuf1.data[tail]; // Get next character
  rxbuf1.tail = BUFNEXT(tail, rxbuf1);       // and update pointer

  return data;
}

static bool serial1SuspendInput(bool suspend) {
  return stream_rx_suspend(&rxbuf1, suspend);
}

static bool serial1SetBaudRate(uint32_t baud_rate) {
  stream_status[1].baud_rate = baud_rate;

  UART1->CR1 = USART_CR1_RE | USART_CR1_TE;
  UART1->BRR = UART_DIV_SAMPLING16(UART1_CLK(), baud_rate);
  UART1->CR1 |= (USART_CR1_UE | USART_CR1_RXNEIE);

  return true;
}

static bool serial1Disable(bool disable) {
  if (disable)
    UART1->CR1 &= ~USART_CR1_RXNEIE;
  else
    UART1->CR1 |= USART_CR1_RXNEIE;

  return true;
}

static bool serial1EnqueueRtCommand(uint8_t c) {
  return enqueue_realtime_command1(c);
}

static enqueue_realtime_command_ptr
serial1SetRtHandler(enqueue_realtime_command_ptr handler) {
  enqueue_realtime_command_ptr prev = enqueue_realtime_command1;

  if (handler)
    enqueue_realtime_command1 = handler;

  return prev;
}

static const io_stream_t *serial1Init(uint32_t baud_rate) {
  static const io_stream_t stream = {
      .type = StreamType_Serial,
      .instance = 1,
      .is_connected = stream_connected,
      .read = serial1GetC,
      .write = serial1WriteS,
      .write_n = serial1Write,
      .write_char = serial1PutC,
      .enqueue_rt_command = serial1EnqueueRtCommand,
      .get_rx_buffer_free = serial1RxFree,
      .get_rx_buffer_count = serial1RxCount,
      .get_tx_buffer_count = serial1TxCount,
      .reset_write_buffer = serial1TxFlush,
      .reset_read_buffer = serial1RxFlush,
      .cancel_read_buffer = serial1RxCancel,
      .suspend_read = serial1SuspendInput,
      .disable_rx = serial1Disable,
      .set_baud_rate = serial1SetBaudRate,
#if MODBUS_RTU_STREAM == 1 && defined(RS485_DIR_PORT)
      .set_direction = rs485SetDirection,
#endif
      .set_enqueue_rt_handler = serial1SetRtHandler};

  if (!serial[1].flags.claimable || serial[1].flags.claimed)
    return NULL;

  serial[1].flags.claimed = On;

  if (!serial[1].flags.init_ok) {

    UART1_GPIO_CLK_ENABLE();
    UART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {.Mode = GPIO_MODE_AF_PP,
                                           .Pull = GPIO_NOPULL,
                                           .Speed = GPIO_SPEED_FREQ_HIGH,
                                           .Pin = (1 << UART1_TX_PIN),
                                           .Alternate = UART1_AF};
    HAL_GPIO_Init(UART1_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = (1 << UART1_RX_PIN);
    HAL_GPIO_Init(UART1_RX_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(UART1_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(UART1_IRQ);

    serial[1].flags.init_ok = On;
  }

  stream_set_defaults(&stream, baud_rate);

  return &stream;
}

void UART1_IRQHandler(void) {
  if (UART1->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    UART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;

  if (UART1->ISR & USART_ISR_RXNE) {
    uint8_t data = UART1->RDR;
    if (!enqueue_realtime_command1(
            data)) { // Check and strip realtime commands...
      uint16_t next_head =
          BUFNEXT(rxbuf1.head, rxbuf1); // Get and increment buffer pointer
      if (next_head == rxbuf1.tail)     // If buffer full
        rxbuf1.overflow = 1;            // flag overflow
      else {
        rxbuf1.data[rxbuf1.head] = data; // if not add data to buffer
        rxbuf1.head = next_head;         // and update pointer
      }
    }
#ifdef BOARD_BTT_SKR_MINI_E3_V20 // restricting to just this board. Ref.
                                 // https://github.com/grblHAL/STM32F1xx/issues/49
    UART1->CR1 |= USART_CR1_RXNEIE;
#endif
  }

  if ((UART1->ISR & USART_ISR_TXE) && (UART1->CR1 & USART_CR1_TXEIE)) {
    uint_fast16_t tail = txbuf1.tail;           // Get buffer pointer
    UART1->TDR = txbuf1.data[tail];             // Send next character
    txbuf1.tail = tail = BUFNEXT(tail, txbuf1); // and increment pointer
    if (tail == txbuf1.head)                    // If buffer empty then
      UART1->CR1 &= ~USART_CR1_TXEIE;           // disable UART TX interrupt
  }
}

#endif // SERIAL1_PORT
