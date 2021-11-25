/*
This is the c configuration file for the keymap

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


/* Select hand configuration */
//#define MASTER_LEFT
//#define MASTER_RIGHT
#undef MOUSEKEY_INTERVAL
#undef MOUSEKEY_DELAY
#undef MOUSEKEY_MOVE_DELTA
#undef MOUSEKEY_INITIAL_SPEED
#undef MOUSEKEY_BASE_SPEED
#undef MOUSEKEY_WHEEL_INITIAL_MOVEMENTS
#undef MOUSEKEY_WHEEL_BASE_MOVEMENTS
#undef MOUSEKEY_WHEEL_DELAY

#define MOUSEKEY_INTERVAL       5
#define MOUSEKEY_DELAY          10
#define MOUSEKEY_MOVE_DELTA     50
#define MOUSEKEY_INITIAL_SPEED  100
#define MOUSEKEY_BASE_SPEED     5000
#define MOUSEKEY_WHEEL_DELAY    10
#define MOUSEKEY_WHEEL_INITIAL_MOVEMENTS  20
#define MOUSEKEY_WHEEL_BASE_MOVEMENTS     2000

#define MK_KINETIC_SPEED