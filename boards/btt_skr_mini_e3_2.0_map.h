/*
  btt_skr_mini_e3_2.0_map.h - driver code for STM32F303CC ARM processors

  Pin mapping carried over from the BTT SKR MINI E3 V2.0 layout (STM32F103RC)
  with the Paige overrides: X/Y limits on PB7/PB6, Trinamic UART on USART2
  (PA2/PA3). Z axis has no hardware on this build - its pins are mapped to
  free GPIOB pins as the core requires a minimum of three axes.

  Part of grblHAL

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

#ifndef STM32F303xC
#error "This board map requires an STM32F303xC processor, select a corresponding build!"
#endif

#if N_ABC_MOTORS > 0
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "BTT SKR MINI E3 V2.0 (F303)"
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3"

#define SERIAL_PORT     1 // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT    2 // GPIOA: TX = 2, RX = 3 - to Trinamic drivers (USART2)
#define HAS_BOARD_INIT

#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE 2209
#endif
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 0
#endif
#define TRINAMIC_STREAM 1

// Define step pulse output pins.
// NOTE: X/Y assignments are swapped relative to the stock BTT SKR MINI E3 V2.0
// (F103) layout - verified against the actual board netlist ($pins vs reference):
// PB10/PB2/PB11 drive the X motor, PB13/PB12/PB14 drive the Y motor.
#define STEP_PORT               GPIOB
#define X_STEP_PIN              10 // PB10
#define Y_STEP_PIN              13 // PB13
#define Z_STEP_PIN              0  // PB0 - no Z hardware
#define STEP_OUTMODE            GPIO_MAP

// Define step direction output pins.
#define DIRECTION_PORT          GPIOB
#define X_DIRECTION_PIN         2  // PB2
#define Y_DIRECTION_PIN         12 // PB12
#define Z_DIRECTION_PIN         5  // PB5 - no Z hardware
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pins.
#define X_ENABLE_PORT           GPIOB
#define X_ENABLE_PIN            11 // PB11
#define Y_ENABLE_PORT           GPIOB
#define Y_ENABLE_PIN            14 // PB14
#define Z_ENABLE_PORT           GPIOB
#define Z_ENABLE_PIN            1  // PB1 - no Z hardware

// Define homing/hard limit switch input pins.
// All limit inputs must be on the same port with this driver.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             7  // PB7 - sensor originally wired to the Y connector
#define Y_LIMIT_PIN             6  // PB6
#define Z_LIMIT_PIN             15 // PB15 - no Z hardware
#define LIMIT_INMODE            GPIO_MAP
