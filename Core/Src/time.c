/*
 * time.c
 *
 *  Created on: May 31, 2024
 *      Author: thoma
 */

#include "time.h"

static volatile uint32_t milliseconds = 0;

// Initialize timer or system tick interrupt to increment milliseconds variable
// This part is platform-specific and may require hardware-specific code

// Function to get current millisecond count
uint32_t millis() {
    return milliseconds;
}

// Function to delay for a specified number of milliseconds
void delay(uint32_t ms) {
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        // You may need to insert platform-specific delay mechanism here
    }
}
