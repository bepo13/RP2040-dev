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

#ifndef BUTTONS_H__
#define BUTTONS_H__

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

// Minimum time between button transitions
#define BUTTON_DEBOUNCE_MS  10

// Structure definition for mouse button
typedef struct {
    uint32_t pin;
    uint8_t mask;
} t_button;

// Mouse button configuration structure (add buttons here)
t_button buttons[] = {
    {
        .pin = 2,
        .mask = MOUSE_BUTTON_LEFT
    },
    {
        .pin = 22,
        .mask = MOUSE_BUTTON_RIGHT
    },
    {
        .pin = 6,
        .mask = MOUSE_BUTTON_MIDDLE
    },
    {
        .pin = 7,
        .mask = MOUSE_BUTTON_BACKWARD
    },
    {
        .pin = 8,
        .mask = MOUSE_BUTTON_FORWARD
    }
};

// Number of buttons configured
#define BUTTON_COUNT    (sizeof(buttons)/sizeof(t_button))

// Initialize mouse buttons
static void buttons_init(void)
{
    uint32_t i;

    // Initialize all mouse buttons
    for (i = 0; i < BUTTON_COUNT; i++) {
        gpio_init(buttons[i].pin);
        gpio_set_dir(buttons[i].pin, GPIO_IN);
        gpio_pull_up(buttons[i].pin);
    }
}

// Definition for mouse button state
typedef struct {
    bool last;
    uint32_t timestamp;
} t_buttonState;

// Poll mouse buttons and return mask
static uint8_t buttons_getMask(void)
{
    uint32_t i;
    uint32_t timestamp;
    static uint8_t mask = 0;
    static t_buttonState states[BUTTON_COUNT] = {0};

    // Get current timestamp
    timestamp = to_ms_since_boot(get_absolute_time());

    // Poll all mouse buttons
    for (i = 0; i < BUTTON_COUNT; i++) {
        // Check time since last state change; only check button if past debounce delay
        if ((timestamp - states[i].timestamp) > BUTTON_DEBOUNCE_MS) {
            // Check if button has changed (active low)
            if (!gpio_get(buttons[i].pin) ^ states[i].last) {
                // Update button state and timestamp
                states[i].last ^= true;
                states[i].timestamp = timestamp;

                // Toggle button mask
                mask ^= buttons[i].mask;
            }
        }
    }

    // Return button mask
    return mask;
}

#endif //BUTTONS_H__
