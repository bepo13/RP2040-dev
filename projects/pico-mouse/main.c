/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "tusb.h"

#include "usb_descriptors.h"

#include "PMW3360.h"

#define PIN_LED             25
#define PIN_MOUSE1          14
#define PIN_MOUSE2          15

#define TIMESTAMP_US()      (to_us_since_boot(get_absolute_time()))
#define TIMESTAMP_MS()      (to_ms_since_boot(get_absolute_time()))

// LED blink intervals
enum {
    BLINK_PMW_ERROR = 100,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_task(void);
void hid_task(void);

int main(void)
{
    // Initialize LED pin
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    // Initialize MOUSE1 pin
    gpio_init(PIN_MOUSE1);
    gpio_set_dir(PIN_MOUSE1, GPIO_IN);
    gpio_pull_up(PIN_MOUSE1);

    // Initialize MOUSE2 pin
    gpio_init(PIN_MOUSE2);
    gpio_set_dir(PIN_MOUSE2, GPIO_IN);
    gpio_pull_up(PIN_MOUSE2);

    // Initialize PMW3360 sensor
    if (!PMW3360_init()) {
        // Error while initializing, blink LED
        while (1) {
            blink_interval_ms = BLINK_PMW_ERROR;
            led_task();
        }
    }
    
    // Initialize TinyUSB
    tusb_init();

    // Run tasks in main loop
    while (1) {
        tud_task();
        led_task();
        hid_task();
    }

    return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us    to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

#define BUTTON_DEBOUNCE_MS      10

static uint8_t getButtonMask(void)
{
    uint32_t timestamp = TIMESTAMP_MS();

    // Check if MOUSE1 button has changed after debounce delay
    static bool buttonLastM1 = false;
    static uint32_t timestampLastM1 = 0;
    if ((timestamp - timestampLastM1) > BUTTON_DEBOUNCE_MS) {
        // Check if button has changed
        if (!gpio_get(PIN_MOUSE1) ^ buttonLastM1) {
            buttonLastM1 ^= true;
            timestampLastM1 = timestamp;
        }
    }

    // Check if MOUSE2 button has changed after debounce delay
    static bool buttonLastM2 = false;
    static uint32_t timestampLastM2 = 0;
    if ((timestamp - timestampLastM2) > BUTTON_DEBOUNCE_MS) {
        // Check if button has changed
        if (!gpio_get(PIN_MOUSE2) ^ buttonLastM2) {
            buttonLastM2 ^= true;
            timestampLastM2 = timestamp;
        }
    }

    // Return button mask
    return ((buttonLastM2 << 1) | (buttonLastM1 << 0));
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
    uint8_t buttonMask;
    PMW3360_data data;
    
    // Poll every 1000us
    const uint32_t interval_us = 1000;
    static uint32_t start_us = 0;
    if (TIMESTAMP_US() - start_us < interval_us) {
        // not enough time
        return;
    }
    start_us += interval_us;

    // Remote wakeup
    uint32_t const btn = false;
    if (tud_suspended() && btn) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    }
    else {

        // skip if hid is not ready yet
        if (!tud_hid_ready()) {
            return;
        }

        // Read buttons
        buttonMask = getButtonMask();

        // Read data from PMW3360 sensor
        PMW3360_read(&data);

        // no button, right + down, no scroll, no pan
        tud_hid_mouse_report(REPORT_ID_MOUSE, buttonMask, data.dx, data.dy, 0, 0);
    }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    return;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    return;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // blink is disabled
    if (!blink_interval_ms) {
        return;
    }

    // Blink every interval ms
    if (TIMESTAMP_MS() - start_ms < blink_interval_ms) {
        // not enough time
        return;
    }
    start_ms += blink_interval_ms;

    // toggle LED
    gpio_put(PIN_LED, led_state);
    led_state ^= 1;
}
