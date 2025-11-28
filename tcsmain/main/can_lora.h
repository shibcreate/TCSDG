#pragma once

#include <stdint.h>
#include <stdbool.h>
#define CAN_LORA_TAG "CAN_LORA"

// Structure for LoRa commands
typedef struct {
    char key[32];
    int value;
    bool new_command;
} lora_command_t;

// Global LoRa command object
extern lora_command_t lora_cmd;

//Starts CAN task
void start_can_task(void);

// CAN task - call this from main.c with xTaskCreate
void canTask(void* arg);

// CAN receive task/function to call periodically
void canReceive(void);

// Parse incoming CAN messages by ID and data
void parseCanMessages(uint32_t msg_id, uint8_t data[8]);

// Handles LoRa commands coming from the LoRa subsystem
void handle_lora_can_command(void);