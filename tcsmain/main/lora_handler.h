#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

// LoRa functions
void lora_handler_init(void);       // Initialize LoRa
void lora_handler_start(void);      // Start telemetry/fault tasks
void lora_handler_stop(void);       // Stop tasks
void lora_send_telemetry_data(void);
void lora_send_fault_warnings(void);

#endif // LORA_HANDLER_H