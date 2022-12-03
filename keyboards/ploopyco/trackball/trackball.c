/* Copyright 2020 Christopher Courtney, aka Drashna Jael're  (@drashna) <drashna@live.com>
 * Copyright 2019 Sunjun Kim
 * Copyright 2020 Ploopy Corporation
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "trackball.h"

#ifndef OPT_DEBOUNCE
#    define OPT_DEBOUNCE 5  // (ms) 			Time between scroll events
#endif
#ifndef SCROLL_BUTT_DEBOUNCE
#    define SCROLL_BUTT_DEBOUNCE 100  // (ms) 			Time between scroll events
#endif
#ifndef OPT_THRES
#    define OPT_THRES 150  // (0-1024) 	Threshold for actication
#endif
#ifndef OPT_SCALE
#    define OPT_SCALE 1  // Multiplier for wheel
#endif
#ifndef PLOOPY_DPI_OPTIONS
#    define PLOOPY_DPI_OPTIONS { 1200, 1600, 2400 }
#    ifndef PLOOPY_DPI_DEFAULT
#        define PLOOPY_DPI_DEFAULT 0
#    endif
#endif
#ifndef PLOOPY_DPI_DEFAULT
#    define PLOOPY_DPI_DEFAULT 0
#endif
#ifndef PLOOPY_DRAGSCROLL_DPI
#    define PLOOPY_DRAGSCROLL_DPI 100 // Fixed-DPI Drag Scroll
#endif
#ifndef PLOOPY_DRAGSCROLL_MULTIPLIER
#    define PLOOPY_DRAGSCROLL_MULTIPLIER 0.75 // Variable-DPI Drag Scroll
#endif

keyboard_config_t keyboard_config;
uint16_t          dpi_array[] = PLOOPY_DPI_OPTIONS;
#define DPI_OPTION_SIZE (sizeof(dpi_array) / sizeof(uint16_t))

bool         i2c_initialized = 0;
i2c_status_t bno055_status = I2C_ADDR;
#define I2C_TIMEOUT 1000
// TODO: Implement libinput profiles
// https://wayland.freedesktop.org/libinput/doc/latest/pointer-acceleration.html
// Compile time accel selection
// Valid options are ACC_NONE, ACC_LINEAR, ACC_CUSTOM, ACC_QUADRATIC

// Trackball State
bool     is_scroll_clicked = false;
bool     BurstState        = false;  // init burst state for Trackball module
uint16_t MotionStart       = 0;      // Timer for accel, 0 is resting state
uint16_t lastScroll        = 0;      // Previous confirmed wheel event
uint16_t lastMidClick      = 0;      // Stops scrollwheel from being read if it was pressed
uint8_t  OptLowPin         = OPT_ENC1;
bool     debug_encoder     = false;
bool     is_drag_scroll    = false;
bool     is_joystick       = false;
int16_t     opt_chg       = 0;
uint16_t lastJoystickTime = 0;
int      initialH          = 0;
int      initialR          = 0;
int      initialP          = 0;
// uint16_t lastJoystickPoll  = 0;      // integrating joystick
int8_t  xPos              = 0;
int8_t  yPos              = 0;
int8_t  zPos              = 0;

__attribute__((weak)) void process_wheel_user(report_mouse_t* mouse_report, int16_t h, int16_t v) {
    mouse_report->h = h;
    mouse_report->v = v;
}

__attribute__((weak)) void process_wheel(report_mouse_t* mouse_report) {
    // TODO: Replace this with interrupt driven code,  polling is S L O W
    // Lovingly ripped from the Ploopy Source

    // If the mouse wheel was just released, do not scroll.
    if (timer_elapsed(lastMidClick) < SCROLL_BUTT_DEBOUNCE) {
        return;
    }

    // Limit the number of scrolls per unit time.
    if (timer_elapsed(lastScroll) < OPT_DEBOUNCE) {
        return;
    }

    // Don't scroll if the middle button is depressed.
    if (is_scroll_clicked) {
#ifndef IGNORE_SCROLL_CLICK
        return;
#endif
    }

    lastScroll  = timer_read();
    uint16_t p1 = adc_read(OPT_ENC1_MUX);
    uint16_t p2 = adc_read(OPT_ENC2_MUX);
    // if (debug_encoder) dprintf("OPT1: %d, OPT2: %d\n", p1, p2);

    opt_chg = (int16_t)opt_encoder_handler(p1, p2);
    if (opt_chg == 0) return;

    // mh += opt_chg * 8;
    if (!is_joystick) {
        process_wheel_user(mouse_report, mouse_report->h, (int)(mouse_report->v + (opt_chg * OPT_SCALE)));
    }
    else
    {
        zPos = constrain(zPos + (opt_chg * 8), -127, 127);
    }
}

__attribute__((weak)) void process_mouse_user(report_mouse_t* mouse_report, int16_t x, int16_t y) {
    mouse_report->x = x;
    mouse_report->y = y;
}

__attribute__((weak)) void process_mouse(report_mouse_t* mouse_report) {
    report_pmw_t data = pmw_read_burst();
    if (data.isOnSurface && data.isMotion) {
        // Reset timer if stopped moving
        if (!data.isMotion) {
            if (MotionStart != 0) MotionStart = 0;
            return;
        }

        // Set timer if new motion
        if ((MotionStart == 0) && data.isMotion) {
            if (debug_mouse) dprintf("Starting motion.\n");
            MotionStart = timer_read();
        }

        if (debug_mouse) {
            dprintf("Delt] d: %d t: %u\n", abs(data.dx) + abs(data.dy), MotionStart);
        }
        if (debug_mouse) {
            dprintf("Pre ] X: %d, Y: %d\n", data.dx, data.dy);
        }
#if defined(PROFILE_LINEAR)
        float scale = float(timer_elapsed(MotionStart)) / 1000.0;
        data.dx *= scale;
        data.dy *= scale;
#elif defined(PROFILE_INVERSE)
        // TODO
#else
        // no post processing
#endif
        // apply multiplier
        // data.dx *= mouse_multiplier;
        // data.dy *= mouse_multiplier;

        // Wrap to HID size
        data.dx = constrain(data.dx, -127, 127);
        data.dy = constrain(data.dy, -127, 127);
        if (debug_mouse) dprintf("Cons] X: %d, Y: %d\n", data.dx, data.dy);
        // dprintf("Elapsed:%u, X: %f Y: %\n", i, pgm_read_byte(firmware_data+i));
        if (!is_joystick) {
            process_mouse_user(mouse_report, data.dx, -data.dy);
        }
        else 
        {
            xPos = constrain(xPos + data.dx, -127, 127);
            yPos = constrain(yPos - data.dy, -127, 127);
        }
    }
}

bool process_record_kb(uint16_t keycode, keyrecord_t* record) {
    if (true) {
        xprintf("KL: kc: %u, col: %u, row: %u, pressed: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed);
    }
    // printCalibrationData();
    // getAngles();
    // getAcc();
    // euler_data test = getSensorData();
    
    // print_val_decs(test.h);
    // print_val_decs(test.r);
    // print_val_decs(test.p);
    // print_val_decs(test.x);
    // print_val_decs(test.y);
    // print_val_decs(test.z);


    // Update Timer to prevent accidental scrolls
    if ((record->event.key.col == 1) && (record->event.key.row == 0)) {
        lastMidClick      = timer_read();
        is_scroll_clicked = record->event.pressed;
    }

    if (!process_record_user(keycode, record)) {
        return false;
    }

    if (keycode == DPI_CONFIG && record->event.pressed) {
        keyboard_config.dpi_config = (keyboard_config.dpi_config + 1) % DPI_OPTION_SIZE;
        eeconfig_update_kb(keyboard_config.raw);
        pmw_set_cpi(dpi_array[keyboard_config.dpi_config]);
    }

    if (keycode == DRAG_SCROLL) {
#ifndef PLOOPY_DRAGSCROLL_MOMENTARY
        if ((record->event.pressed) && (record->event.key.col==1))
#endif
        {
            is_drag_scroll ^= 1;
        }
#ifdef PLOOPY_DRAGSCROLL_FIXED
        pmw_set_cpi(is_drag_scroll ? PLOOPY_DRAGSCROLL_DPI : dpi_array[keyboard_config.dpi_config]);
#else
        pmw_set_cpi(is_drag_scroll ? (dpi_array[keyboard_config.dpi_config] * PLOOPY_DRAGSCROLL_MULTIPLIER) : dpi_array[keyboard_config.dpi_config]);
#endif
    }
    if (keycode == JOYSTICK_MODE && record->event.pressed)
    {
        is_joystick ^= 1;
        euler_data sensor = getSensorData();
        initialH = sensor.h;
        initialR = sensor.r;
        initialP = sensor.p;
        xPos = 0;
        yPos = 0;
        zPos = 0;
    }
    if (is_joystick && keycode == DRAG_SCROLL)
    {
        xPos = 0;
        yPos = 0;
        zPos = 0;
        // lastJoystickPoll = timer_read();
    }

/* If Mousekeys is disabled, then use handle the mouse button
 * keycodes.  This makes things simpler, and allows usage of
 * the keycodes in a consistent manner.  But only do this if
 * Mousekeys is not enable, so it's not handled twice.
 */
#ifndef MOUSEKEY_ENABLE
    if (IS_MOUSEKEY_BUTTON(keycode)) {
        report_mouse_t currentReport = pointing_device_get_report();
        if (record->event.pressed) {
            currentReport.buttons |= 1 << (keycode - KC_MS_BTN1);
        } else {
            currentReport.buttons &= ~(1 << (keycode - KC_MS_BTN1));
        }
        pointing_device_set_report(currentReport);
        pointing_device_send();
    }
#endif

    return true;
}

// Hardware Setup
void keyboard_pre_init_kb(void) {
    // debug_enable  = true;
    // debug_matrix  = true;
    // debug_mouse   = true;
    // debug_encoder = true;

    setPinInput(OPT_ENC1);
    setPinInput(OPT_ENC2);

    /* Ground all output pins connected to ground. This provides additional
     * pathways to ground. If you're messing with this, know this: driving ANY
     * of these pins high will cause a short. On the MCU. Ka-blooey.
     */
#ifdef UNUSED_PINS
    const pin_t unused_pins[] = UNUSED_PINS;

    for (uint8_t i = 0; i < (sizeof(unused_pins) / sizeof(pin_t)); i++) {
        setPinOutput(unused_pins[i]);
        writePinLow(unused_pins[i]);
    }
#endif

    // This is the debug LED.
#if defined(DEBUG_LED_PIN)
    setPinOutput(DEBUG_LED_PIN);
    writePin(DEBUG_LED_PIN, debug_enable);
#endif

    keyboard_pre_init_user();
}

void pointing_device_init(void) {
    // initialize ball sensor
    pmw_spi_init();

    // initialize the scroll wheel's optical encoder
    opt_encoder_init();
    
    // initilaize bno055
    init_bno055();
}


void pointing_device_task(void) {
    report_mouse_t mouse_report = pointing_device_get_report();
    process_wheel(&mouse_report);
    process_mouse(&mouse_report);

    if (!is_joystick) {
        if (is_drag_scroll) {
            mouse_report.h = mouse_report.x;
#ifdef PLOOPY_DRAGSCROLL_INVERT
            // Invert vertical scroll direction
            mouse_report.v = -mouse_report.y;
#else
            mouse_report.v = mouse_report.y;
#endif
            mouse_report.x = 0;
            mouse_report.y = 0;
        }
    }

    pointing_device_set_report(mouse_report);
    pointing_device_send();
}

void joystick_task(void) {
    if (is_joystick) {
        euler_data sensor = getSensorData();
        // print_val_decs(sensor.r);
        // print_val_decs(sensor.p);
        // print_val_decs(initialH);
        sensor.h = initialH - sensor.h;
        if (sensor.h > 180)
        {
            sensor.h = sensor.h - 360;
        }
        if (sensor.h < -180)
        {
            sensor.h = sensor.h + 360;
        }
        sensor.r = initialR - sensor.r;
        if (sensor.r > 180)
        {
            sensor.r = sensor.r - 360;
        }
        if (sensor.r < -180)
        {
            sensor.r = sensor.r + 360;
        }
        sensor.p = initialP - sensor.p;
        if (sensor.p > 180)
        {
            sensor.p = sensor.p - 360;
        }
        if (sensor.p < -180)
        {
            sensor.p = sensor.p + 360;
        }
        joystick_status.axes[3] = constrain(-sensor.r * 4, -127, 127);
        joystick_status.axes[4] = constrain(sensor.p * 4, -127, 127);
        joystick_status.axes[5] = constrain(-sensor.h * 4, -127, 127);
        if (is_joystick && is_drag_scroll)
        {
            // print_val_decs(sensor.x);
            // print_val_decs(sensor.y);
            // print_val_decs(sensor.z);
            // print_val_decs(timer_elapsed(lastJoystickPoll));
            // xVel = xVel + (sensor.x / timer_elapsed(lastJoystickPoll));
            // yVel = yVel + (sensor.y / timer_elapsed(lastJoystickPoll));
            // zVel = zVel + (sensor.z / timer_elapsed(lastJoystickPoll));
            // lastJoystickPoll = timer_read();
        }
        joystick_status.axes[0] = constrain(xPos, -127, 127);
        joystick_status.axes[1] = constrain(yPos, -127, 127);
        joystick_status.axes[2] = constrain(zPos, -127, 127);

        // print_val_decs(sensor.h);
        send_joystick_packet(&joystick_status);
    }
}

void eeconfig_init_kb(void) {
    keyboard_config.dpi_config = PLOOPY_DPI_DEFAULT;
    eeconfig_update_kb(keyboard_config.raw);
    eeconfig_init_user();
}

void matrix_init_kb(void) {
    // is safe to just read DPI setting since matrix init
    // comes before pointing device init.
    keyboard_config.raw = eeconfig_read_kb();
    if (keyboard_config.dpi_config > DPI_OPTION_SIZE) {
        eeconfig_init_kb();
    }
    matrix_init_user();
}

void keyboard_post_init_kb(void) {
    pmw_set_cpi(dpi_array[keyboard_config.dpi_config]);

    keyboard_post_init_user();
}

uint8_t init_bno055(void) {
    bno055_status = I2C_ADDR;

    // I2C subsystem
    if (i2c_initialized == 0) {
        i2c_init();  // on pins D(1,0)
        i2c_initialized = true;
        wait_ms(I2C_TIMEOUT);
    }

    uint8_t data = 0;
    bno055_status = i2c_readReg(I2C_ADDR << 1, BNO055_CHIP_ID_ADDR, &data, sizeof(data), I2C_TIMEOUT);
    if (!bno055_status) {
        bno055_init();
        bno055_status = i2c_readReg(I2C_ADDR << 1, BNO055_CHIP_ID_ADDR, &data, sizeof(data), I2C_TIMEOUT);
    }
    return bno055_status;
}