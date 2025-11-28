#include <string.h>
#include <stdio.h>
#include "telemetry.h"

telemetry_entry_t telemetry_data[TELEM_COUNT] = {
    [TELEM_EMETER_CURRENT]                   = {"EMeter_Current", 0.0f},
    [TELEM_EMETER_VOLTAGE]                   = {"EMeter_Voltage", 0.0f},
    [TELEM_MCM_MOTOR_SPEED]                  = {"MCM_Motor_Speed", 0.0f},
    [TELEM_MCM_DC_BUS_CURRENT]               = {"MCM_DCBus_Current", 0.0f},
    [TELEM_MCM_DC_BUS_VOLTAGE]               = {"MCM_DCBus_Voltage", 0.0f},
    [TELEM_MCM_INT_INVERT_ENABLE_STATE]      = {"MCM_IntInvert_EnableState", 0.0f},
    [TELEM_MCM_INT_INVERTER_STATE]           = {"MCM_IntInverter_State", 0.0f},
    [TELEM_MCM_TORQUE_FEEDBACK]              = {"MCM_Torque_Feedback", 0.0f},
    [TELEM_MCM_COMMANDED_TORQUE]             = {"MCM_Commanded_Torque", 0.0f},
    [TELEM_MCM_TORQUE_COMMAND]               = {"MCM_Torque_Command", 0.0f},
    [TELEM_MCM_SPEED_COMMAND]                = {"MCM_Speed_Command", 0.0f},
    [TELEM_MCM_SPEED_MODE_ENABLE]            = {"MCM_Speed_Mode_Enable", 0.0f},
    [TELEM_TPS1_THROTTLE_PERCENT]            = {"TPS1_Throttle_Percent", 0.0f},
    [TELEM_BPS0_BRAKE_PERCENT]               = {"BPS0_Brake_Percent", 0.0f},
    [TELEM_VCU_WSS_FL_S]                     = {"VCU_WSS_FL_S", 0.0f},
    [TELEM_VCU_WSS_FR_S]                     = {"VCU_WSS_FR_S", 0.0f},
    [TELEM_VCU_WSS_RL_S]                     = {"VCU_WSS_RL_S", 0.0f},
    [TELEM_VCU_WSS_RR_S]                     = {"VCU_WSS_RR_S", 0.0f},
    [TELEM_VCU_FAULT_TPS_OUTOFRANGE]         = {"VCU_Fault_TPS_OutOfRange", 0.0f},
    [TELEM_VCU_FAULT_BPS_OUTOFRANGE]         = {"VCU_Fault_BPS_OutOfRange", 0.0f},
    [TELEM_VCU_FAULT_TPS_OUTOFSYNC]          = {"VCU_FAULT_TPS_OutOfSync", 0.0f},
    [TELEM_VCU_FAULT_TPSBPS_IMPLAUSIBLE]     = {"VCU_FAULT_TPSBPS_Implausible", 0.0f},
    [TELEM_VCU_FAULT_BSPD_SOFTFAULT]         = {"VCU_FAULT_BSPD_SoftFault", 0.0f},
    [TELEM_VCU_WARNING_LVS_BATTERYLOW]       = {"VCU_FAULT_LVS_BatteryLow", 0.0f},
    [TELEM_VCU_NOTICE_HVIL_TERMSENSELOST]    = {"VCU_NOTICE_HVIL_TermSenseLost", 0.0f},
    [TELEM_BATTERY_LOW_VOLTAGE]              = {"Battery_Low_Voltage", 0.0f},
    [TELEM_VCU_MCM_REGEN_MODE]               = {"VCU_MCM_RegenMode", 0.0f},
    [TELEM_VCU_MCM_REGEN_MAX_TORQUE]         = {"VCU_MCM_Regen_MaxTorqueNm", 0.0f},
    [TELEM_SPEED_KPH]                        = {"Speed_KPH", 0.0f},
    [TELEM_STEERING_ANGLE]                   = {"Steering_Angle", 0.0f},
    [TELEM_VCU_BMS_HIGHEST_CELL_TEMP]        = {"VCU_BMS_HighestCellTemp", 0.0f},
    [TELEM_VCU_BMS_HIGHEST_CELL_VOLTAGE]     = {"VCU_BMS_HighestCellVoltage", 0.0f},
    [TELEM_VCU_BMS_LOWEST_CELL_VOLTAGE]      = {"VCU_BMS_LowestCellVoltage", 0.0f},
    [TELEM_VCU_BMS_HIGHEST_CELL_TEMP_DC]     = {"VCU_BMS_HighestCellTemp_dC", 0.0f},
    [TELEM_VCU_POWERLIMIT_PID_TOTAL_ERROR]   = {"VCU_POWERLIMIT_PID_getTotalError", 0.0f},
    [TELEM_VCU_POWERLIMIT_PID_PROPORTIONAL]  = {"VCU_POWERLIMIT_PID_getProportional", 0.0f},
    [TELEM_VCU_POWERLIMIT_PID_INTEGRAL]      = {"VCU_POWERLIMIT_PID_getIntegral", 0.0f},
    [TELEM_VCU_POWERLIMIT_TORQUE_COMMAND]    = {"VCU_POWERLIMIT_getTorqueCommand_Nm", 0.0f},
    [TELEM_VCU_LAUNCH_CONTROL_TORQUE_CMD]    = {"VCU_LaunchControl_getTorqueCommand_Nm", 0.0f},
    [TELEM_VCU_LAUNCH_CONTROL_SLIP_RATIO]    = {"VCU_LaunchControl_getSlipRatioScaled", 0.0f},
    [TELEM_VCU_LAUNCH_CONTROL_PID_OUTPUT]    = {"VCU_LaunchControl_getPidOutput", 0.0f},
    [TELEM_VCU_LAUNCH_CONTROL_PID_PROP]      = {"VCU_LaunchControl_PID_Proportional", 0.0f},
    [TELEM_VCU_LAUNCH_CONTROL_PID_INTEGRAL]  = {"VCU_LaunchControl_PID_Integral", 0.0f},
    [TELEM_VCU_LAUNCH_CONTROL_PID_TOTAL_ERROR] = {"VCU_LaunchControl_PID_TotalError", 0.0f},
    [TELEM_BMS_IMMINENT_CONTACTOR_WARNING]   = {"BMS_Imminent_Contactor_Opening_Warning", 0.0f},
    [TELEM_BMS_CELL_UNDER_VOLTAGE_FAULT]     = {"BMS_Cell_Under_Voltage_Fault", 0.0f},
    [TELEM_BMS_CELL_OVER_TEMP_FAULT]         = {"BMS_Cell_Over_Temperature_Fault", 0.0f},
    [TELEM_BMS_PACK_UNDER_VOLTAGE_FAULT]     = {"BMS_Pack_Under_Voltage_Fault", 0.0f},
    [TELEM_BMS_ISOLATION_LEAKAGE_FAULT]      = {"BMS_Isolation_Leakage_Fault", 0.0f},
    [TELEM_BMS_PRECHARGE_FAULT]              = {"BMS_Precharge_Fault", 0.0f},
    [TELEM_BMS_FAILED_THERMISTOR_FAULT]      = {"BMS_Failed_Thermistor_Fault", 0.0f},
    [TELEM_BMS_CURRENT_STATE]                = {"BMS_Current_State", 0.0f},
    [TELEM_BMS_MAIN_CONTACTOR_POS_CLOSED]    = {"BMS_Main_Contactor_Positive_Closed", 0.0f},
    [TELEM_BMS_MAIN_CONTACTOR_NEG_CLOSED]    = {"BMS_Main_Contactor_Negative_Closed", 0.0f},
    [TELEM_BMS_PACK_VOLTAGE]                 = {"Pack_Voltage", 0.0f},
    [TELEM_BMS_STATE_OF_CHARGE]              = {"BMS_State_Of_Charge", 0.0f},
    [TELEM_BMS_HIGHEST_CELL_TEMPERATURE]     = {"BMS_Highest_Cell_Temperature", 0.0f},
    [TELEM_HUMIDITY]                         = {"Humidity", 0.0f},
    [TELEM_TEMPERATURE]                      = {"Temperature", 0.0f},
};

void print_all_telemetry(void) {
    printf("\n=== TELEMETRY DATA ===\n");
    for (int i = 0; i < TELEM_COUNT; i++) {
        if (telemetry_data[i].value != 0.0f) {
            printf("%-35s: %.4f\n", telemetry_data[i].name, telemetry_data[i].value);
        }
    }
    printf("======================\n");
}

float get_telemetry_value_by_name(const char* name) {
    for (int i = 0; i < TELEM_COUNT; i++) {
        if (strcmp(telemetry_data[i].name, name) == 0) {
            return telemetry_data[i].value;
        }
    }
    printf("Warning: Telemetry name '%s' not found\n", name);
    return 0.0f;
}

void update_telemetry_value_by_name(const char* name, float value) {
    for (int i = 0; i < TELEM_COUNT; i++) {
        if (strcmp(telemetry_data[i].name, name) == 0) {
            telemetry_data[i].value = value;
            return;
        }
    }
    printf("Warning: Telemetry name '%s' not found\n", name);
}

void init_telemetry_data(void) {
    for (int i = 0; i < TELEM_COUNT; i++) {
        telemetry_data[i].value = 0.0f;
    }
}
