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

#define PIN_MOUSE1              14
#define PIN_MOUSE2              15

#define DEBOUNCE_MS             10

// Initialize mouse buttons
static void buttons_init(void)
{
    // Initialize MOUSE1 pin
    gpio_init(PIN_MOUSE1);
    gpio_set_dir(PIN_MOUSE1, GPIO_IN);
    gpio_pull_up(PIN_MOUSE1);

    // Initialize MOUSE2 pin
    gpio_init(PIN_MOUSE2);
    gpio_set_dir(PIN_MOUSE2, GPIO_IN);
    gpio_pull_up(PIN_MOUSE2);
}

// Poll mouse buttons and return mask
static uint8_t buttons_getMask(void)
{
    // Get current timestamp
    uint32_t timestamp = to_ms_since_boot(get_absolute_time());

    // Check if MOUSE1 button has changed after debounce delay
    static bool buttonLastM1 = false;
    static uint32_t timestampLastM1 = 0;
    if ((timestamp - timestampLastM1) > DEBOUNCE_MS) {
        // Check if button has changed
        if (!gpio_get(PIN_MOUSE1) ^ buttonLastM1) {
            buttonLastM1 ^= true;
            timestampLastM1 = timestamp;
        }
    }

    // Check if MOUSE2 button has changed after debounce delay
    static bool buttonLastM2 = false;
    static uint32_t timestampLastM2 = 0;
    if ((timestamp - timestampLastM2) > DEBOUNCE_MS) {
        // Check if button has changed
        if (!gpio_get(PIN_MOUSE2) ^ buttonLastM2) {
            buttonLastM2 ^= true;
            timestampLastM2 = timestamp;
        }
    }

    // Return button mask
    return ((buttonLastM2 << 1) | (buttonLastM1 << 0));
}
