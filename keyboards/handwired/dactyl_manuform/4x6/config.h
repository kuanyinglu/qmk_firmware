/*
Copyright 2012 Jun Wako <wakojun@gmail.com>
Copyright 2015 Jack Humbert

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */
#define PRODUCT_ID 0x3436
#define DEVICE_VER 0x0001
#define PRODUCT Keyboard

// wiring
#define MATRIX_ROW_PINS_MCU \
    { B0, B1, B2, B3, B7 }
#define MATRIX_COL_PINS_MCU \
    { F0, F1, F4, F5, F6, F7 }
#define UNUSED_PINS_MCU \
    { C6, C7, D2, D3, D4, D5, E6, D6, D7, B4, B5, B6 }
#define MATRIX_ROW_PINS_MCP \
    { A0, A1, A2, A3, A4 }
#define MATRIX_COL_PINS_MCP \
    { B5, B4, B3, B2, B1, B0 }
#define UNUSED_PINS_MCP \
    { B6, B7, A5, A6, A7 }

/* key matrix size */
// Rows are doubled-up
#define MATRIX_ROWS 10
#define MATRIX_ROWS_PER_SIDE (MATRIX_ROWS / 2)
#define MATRIX_COLS_PER_SIDE (MATRIX_COLS / 2)
#define MATRIX_COLS 12

#define UNUSED_MCU 12
#define UNUSED_MCP 7


/* COL2ROW or ROW2COL */
#define DIODE_DIRECTION COL2ROW
