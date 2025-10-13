#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "esp_attr.h"  // for RTC_DATA_ATTR if needed

// Enum definition for finite states
typedef enum {
    SENDING_STATE,
    RECEIVING_STATE,
    SLEEP_STATE
} FINITE_STATES;

// Global variables (defined in main.c)
extern FINITE_STATES currentState;
extern FINITE_STATES saved_mode;

// Button pin
#define BUTTON_PIN 34

#endif // MAIN_H
