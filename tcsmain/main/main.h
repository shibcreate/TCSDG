#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
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

// Telemetry entry structure
// typedef struct {
//     const char* name;
//     float value;
//     uint8_t type; // 0 = float, 1 = uint8, 2 = int16, etc. if needed
// } telemetry_entry_t;

// // Enum for telemetry indices (makes it easier to access)
// typedef enum {
//     TELEM_EMETER_CURRENT,
//     TELEM_EMETER_VOLTAGE,
//     TELEM_MCM_MOTOR_SPEED,
//     TELEM_MCM_DC_BUS_CURRENT,
//     TELEM_MCM_DC_BUS_VOLTAGE,
//     TELEM_MCM_INT_INVERT_ENABLE_STATE,
//     TELEM_MCM_INT_INVERTER_STATE,
//     TELEM_MCM_TORQUE_FEEDBACK,
//     TELEM_MCM_COMMANDED_TORQUE,
//     TELEM_MCM_TORQUE_COMMAND,
//     TELEM_MCM_SPEED_COMMAND,
//     TELEM_MCM_SPEED_MODE_ENABLE,
//     TELEM_TPS1_THROTTLE_PERCENT,
//     TELEM_BPS0_BRAKE_PERCENT,
//     TELEM_VCU_WSS_FL_S,
//     TELEM_VCU_WSS_FR_S,
//     TELEM_VCU_WSS_RL_S,
//     TELEM_VCU_WSS_RR_S,
//     TELEM_VCU_FAULT_TPS_OUTOFRANGE,
//     TELEM_VCU_FAULT_BPS_OUTOFRANGE,
//     TELEM_VCU_FAULT_TPS_OUTOFSYNC,
//     TELEM_VCU_FAULT_TPSBPS_IMPLAUSIBLE,
//     TELEM_VCU_FAULT_BSPD_SOFTFAULT,
//     TELEM_VCU_WARNING_LVS_BATTERYLOW,
//     TELEM_VCU_NOTICE_HVIL_TERMSENSELOST,
//     TELEM_BATTERY_LOW_VOLTAGE,
//     TELEM_VCU_MCM_REGEN_MODE,
//     TELEM_VCU_MCM_REGEN_MAX_TORQUE,
//     TELEM_SPEED_KPH,
//     TELEM_STEERING_ANGLE,
//     TELEM_VCU_BMS_HIGHEST_CELL_TEMP,
//     TELEM_VCU_BMS_HIGHEST_CELL_VOLTAGE,
//     TELEM_VCU_BMS_LOWEST_CELL_VOLTAGE,
//     TELEM_VCU_BMS_HIGHEST_CELL_TEMP_DC,
//     TELEM_VCU_POWERLIMIT_PID_TOTAL_ERROR,
//     TELEM_VCU_POWERLIMIT_PID_PROPORTIONAL,
//     TELEM_VCU_POWERLIMIT_PID_INTEGRAL,
//     TELEM_VCU_POWERLIMIT_TORQUE_COMMAND,
//     TELEM_VCU_LAUNCH_CONTROL_TORQUE_CMD,
//     TELEM_VCU_LAUNCH_CONTROL_SLIP_RATIO,
//     TELEM_VCU_LAUNCH_CONTROL_PID_OUTPUT,
//     TELEM_VCU_LAUNCH_CONTROL_PID_PROP,
//     TELEM_VCU_LAUNCH_CONTROL_PID_INTEGRAL,
//     TELEM_VCU_LAUNCH_CONTROL_PID_TOTAL_ERROR,
//     TELEM_BMS_IMMINENT_CONTACTOR_WARNING,
//     TELEM_BMS_CELL_UNDER_VOLTAGE_FAULT,
//     TELEM_BMS_CELL_OVER_TEMP_FAULT,
//     TELEM_BMS_PACK_UNDER_VOLTAGE_FAULT,
//     TELEM_BMS_ISOLATION_LEAKAGE_FAULT,
//     TELEM_BMS_PRECHARGE_FAULT,
//     TELEM_BMS_FAILED_THERMISTOR_FAULT,
//     TELEM_BMS_CURRENT_STATE,
//     TELEM_BMS_MAIN_CONTACTOR_POS_CLOSED,
//     TELEM_BMS_MAIN_CONTACTOR_NEG_CLOSED,
//     TELEM_BMS_PACK_VOLTAGE,
//     TELEM_BMS_STATE_OF_CHARGE,
//     TELEM_BMS_HIGHEST_CELL_TEMPERATURE,
//     TELEM_HUMIDITY,
//     TELEM_TEMPERATURE,
//     TELEM_COUNT // Total number of telemetry entries
// } telemetry_index_t;

// LoRa command structure
typedef struct{
    char key[32];
    int value;
    bool new_command;
} lora_command_t;

extern lora_command_t lora_cmd; 

// Global declaration
//extern telemetry_entry_t telemetry_data[TELEM_COUNT];

// Function declarations
void init_telemetry_data(void);
void update_telemetry_value_by_name(const char* name, float value);
float get_telemetry_value_by_name(const char* name);
void print_all_telemetry(void);
void handle_lora_can_command(void);

#endif // MAIN_H
