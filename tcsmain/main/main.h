#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"  // for RTC_DATA_ATTR if needed
#include <rgb_ledc_controller.h>

void handleLightSleepState(void);

// Enum definition for finite states
typedef enum {
    SENDING_STATE,
    RECEIVING_STATE,
    SLEEP_STATE
} FINITE_STATES;

// Global variables
extern FINITE_STATES currentState;
extern FINITE_STATES saved_mode;
extern rgb_led_t led1;
extern const uint32_t RED;
extern const uint32_t BLUE;
extern const uint32_t YELLOW;


#endif // MAIN_H
