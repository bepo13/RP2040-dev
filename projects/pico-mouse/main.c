/* MIT License
 *
 * Copyright (c) 2023 Brent Peterson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "tusb.h"
#include "usb_descriptors.h"

#include "PMW3360.h"
#include "buttons.h"

// Poll mouse at a fixed rate
#define POLLING_RATE_HZ     1000

// GPIO pin for LED
#define PIN_LED             25
#define PIN_TEST            0

// LED blink intervals
enum {
    BLINK_PMW_ERROR = 100,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Polling tasks
void led_task(void);
void hid_task(void);

// Main routine
int main(void)
 {
    // Initialize LED pin
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    // Initialize TEST pin
    gpio_init(PIN_TEST);
    gpio_set_dir(PIN_TEST, GPIO_OUT);
    gpio_put(PIN_TEST, 0);

    // Initialize mouse buttons
    buttons_init();

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

// Toggle LED at variable interval
void led_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;
    uint32_t timestamp = to_ms_since_boot(get_absolute_time());

    // Blink every interval ms
    if (timestamp - start_ms >= blink_interval_ms) {
        // toggle LED
        led_state ^= true;
        gpio_put(PIN_LED, led_state);
        start_ms += blink_interval_ms;
    }
}

// Send HID report at a fixed polling rate
void hid_task(void)
{
    uint8_t buttonMask;
    uint32_t timestamp;
    PMW3360_data data;
    
    // Poll mouse at a fixed rate
    static uint32_t start_us = 0;
    const uint32_t interval_us = 1000000/POLLING_RATE_HZ;
    timestamp = to_us_since_boot(get_absolute_time());
    if (timestamp - start_us >= interval_us) {
        // Increment start counter
        start_us += interval_us;

        // Read buttons and get mask
        buttonMask = buttons_getMask();

        // Check if device is currently suspended and button is pressed
        if (tud_suspended() && buttonMask) {
            // Wake up host if REMOTE_WAKEUP feature is enabled by host
            tud_remote_wakeup();
        }
        else {
            // Read data from PMW3360 sensor
            PMW3360_read(&data);

            // Send mouse report to host if HID is ready
            if (tud_hid_ready()) {
                gpio_put(PIN_TEST, 1);
                tud_hid_mouse_report(REPORT_ID_MOUSE, buttonMask, data.dx, data.dy, 0, 0);
            }
        }
    }
}

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
void tud_suspend_cb(bool remote_wakeup_en)
{
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    gpio_put(PIN_TEST, 0);
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
