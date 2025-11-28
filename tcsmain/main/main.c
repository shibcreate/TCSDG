#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "lora_handler.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/twai.h"
#include <dht.h>
#include <icm42670.h>
#include <time.h>
//new refactored .h
#include <rgb_ledc_controller.h>
#include "humidity.h"
#include "telemetry.h"

#define BUTTON_PIN 1
#define BUF_SIZE 128

//humidity sensor
// Hardcoded sensor type
//#define SENSOR_TYPE DHT_TYPE_AM2301

// Hardcoded GPIO pin for the data line
//#define DHT_GPIO 45  // Change to your GPIO pin

static const char *TAG_RGB = "rainbow_flash";

/* GPIOs for RGB LED */
#define GPIO_LED_RED   38
#define GPIO_LED_GREEN 39
#define GPIO_LED_BLUE  40

// Rainbow colors
#define RED     0xFF0000
#define BLUE  0x0000FF
#define YELLOW  0xFFFF00

rgb_led_t led1;

//RTC
static time_t start_time = 1700286000; // Update as needed

// Function to get relative time since start_time in seconds
time_t get_relative_time() {
    int64_t usec = esp_timer_get_time();
    time_t elapsed_sec = usec / 1000000;
    return start_time + elapsed_sec;
}


//CRASH STUFF
typedef struct {
    float g_force;
    float accel[3];
	uint32_t vcu_can_id;
    uint8_t vcu_can_dlc;
    uint8_t vcu_can_data[8];
	uint32_t bms_can_id;
    uint8_t bms_can_dlc;
    uint8_t bms_can_data[8];
    bool vcu_captured;
	bool bms_captured;
} crash_record_t;

static uint8_t imu_crash_event = 2; // 1: Write, 2: Read, 3: Erase
static bool vcu_can_captured = false;
static bool bms_can_captured = false;

// Global var for current mode
FINITE_STATES currentState = 0;
RTC_DATA_ATTR FINITE_STATES saved_mode;

void IRAM_ATTR button_isr(void *arg) {
    if (gpio_get_level(BUTTON_PIN) == 0) {
        currentState = RECEIVING_STATE;  // pressed and hold
    } else {
        currentState = SENDING_STATE;  // not pressed
    }
}

static const char *TAG = "CANDUMP";

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#if CONFIG_CAN_BITRATE_25
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
#define BITRATE "Bitrate is 25 Kbit/s"
#elif CONFIG_CAN_BITRATE_50
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_50KBITS();
#define BITRATE "Bitrate is 50 Kbit/s"
#elif CONFIG_CAN_BITRATE_100
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
#define BITRATE "Bitrate is 100 Kbit/s"
#elif CONFIG_CAN_BITRATE_125
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
#define BITRATE "Bitrate is 125 Kbit/s"
#elif CONFIG_CAN_BITRATE_250
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
#define BITRATE "Bitrate is 250 Kbit/s"
#elif CONFIG_CAN_BITRATE_500
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#define BITRATE "Bitrate is 500 Kbit/s"
#elif CONFIG_CAN_BITRATE_800
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_800KBITS();
#define BITRATE "Bitrate is 800 Kbit/s"
#elif CONFIG_CAN_BITRATE_1000
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
#define BITRATE "Bitrate is 1 Mbit/s"
#endif

static const char *TAG_IMU = "IMU";
#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
#define I2C_ADDR ICM42670_I2C_ADDR_GND
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
#define I2C_ADDR ICM42670_I2C_ADDR_VCC
#endif

static TaskHandle_t stateManager = NULL;
void stateManagerTask(void* parameter);
void handleSendState(void);
void handleReceiveState(void);
void handleLightSleepState(void);
void parseCanMessages(uint32_t msg_id, uint8_t data[8]);
void canReceive();

//CRASH STUFF
void vcu_save_crash_record(crash_record_t *record);
void bms_save_crash_record(crash_record_t *record);
void vcu_read_crash_record();
void bms_read_crash_record();
void vcu_erase_crash_record();
void bms_erase_crash_record();
void handle_crash_event();

//DATA STUFF
//void init_telemetry_data(void);
//void update_telemetry_value_by_name(const char* name, float value);
//float get_telemetry_value_by_name(const char* name);
//void print_all_telemetry(void);

//Humidity
//void dht_test(void *pvParameters);

//CAN TASK
void canTask(void* arg);

//IMU TASK
void icm42670_wom_test(void *pvParameters);
void icm42670_test(void *pvParameters);


static const twai_general_config_t g_config =
	TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CTX_GPIO, CONFIG_CRX_GPIO, TWAI_MODE_NORMAL);


void stateManagerTask(void* parameter){
    currentState = SENDING_STATE;
    
    for(;;){
        switch (currentState)
        {
        case SENDING_STATE: //Master sends to lora and receives from can
            
            rgb_led_set_color(&led1, BLUE);
            
            break;
        case RECEIVING_STATE: //Master receives from lora and sends to can
        
            rgb_led_set_color(&led1, RED);
            
            break;
        case SLEEP_STATE:
            handleLightSleepState();
            break;
        default:
            printf("Default\n");
            break;
        }

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// LoRa managing
lora_command_t lora_cmd = {.key = "", .value = 0, .new_command = false};
#define CAN_ID 0x7FF // Same CAN ID for all mode switches
void handle_lora_can_command(void);

// Managing Data
// Initialize the 2D telemetry data array
// telemetry_entry_t telemetry_data[TELEM_COUNT] = {
//     [TELEM_EMETER_CURRENT] = {"EMeter_Current", 0.0f},
//     [TELEM_EMETER_VOLTAGE] = {"EMeter_Voltage", 0.0f},
//     [TELEM_MCM_MOTOR_SPEED] = {"MCM_Motor_Speed", 0.0f},
//     [TELEM_MCM_DC_BUS_CURRENT] = {"MCM_DCBus_Current", 0.0f},
//     [TELEM_MCM_DC_BUS_VOLTAGE] = {"MCM_DCBus_Voltage", 0.0f},
//     [TELEM_MCM_INT_INVERT_ENABLE_STATE] = {"MCM_IntInvert_EnableState", 0.0f},
//     [TELEM_MCM_INT_INVERTER_STATE] = {"MCM_IntInverter_State", 0.0f},
//     [TELEM_MCM_TORQUE_FEEDBACK] = {"MCM_Torque_Feedback", 0.0f},
//     [TELEM_MCM_COMMANDED_TORQUE] = {"MCM_Commanded_Torque", 0.0f},
//     [TELEM_MCM_TORQUE_COMMAND] = {"MCM_Torque_Command", 0.0f},
//     [TELEM_MCM_SPEED_COMMAND] = {"MCM_Speed_Command", 0.0f},
//     [TELEM_MCM_SPEED_MODE_ENABLE] = {"MCM_Speed_Mode_Enable", 0.0f},
//     [TELEM_TPS1_THROTTLE_PERCENT] = {"TPS1_Throttle_Percent", 0.0f},
//     [TELEM_BPS0_BRAKE_PERCENT] = {"BPS0_Brake_Percent", 0.0f},
//     [TELEM_VCU_WSS_FL_S] = {"VCU_WSS_FL_S", 0.0f},
//     [TELEM_VCU_WSS_FR_S] = {"VCU_WSS_FR_S", 0.0f},
//     [TELEM_VCU_WSS_RL_S] = {"VCU_WSS_RL_S", 0.0f},
//     [TELEM_VCU_WSS_RR_S] = {"VCU_WSS_RR_S", 0.0f},
//     [TELEM_VCU_FAULT_TPS_OUTOFRANGE] = {"VCU_Fault_TPS_OutOfRange", 0.0f},
//     [TELEM_VCU_FAULT_BPS_OUTOFRANGE] = {"VCU_Fault_BPS_OutOfRange", 0.0f},
//     [TELEM_VCU_FAULT_TPS_OUTOFSYNC] = {"VCU_FAULT_TPS_OutOfSync", 0.0f},
//     [TELEM_VCU_FAULT_TPSBPS_IMPLAUSIBLE] = {"VCU_FAULT_TPSBPS_Implausible", 0.0f},
//     [TELEM_VCU_FAULT_BSPD_SOFTFAULT] = {"VCU_FAULT_BSPD_SoftFault", 0.0f},
//     [TELEM_VCU_WARNING_LVS_BATTERYLOW] = {"VCU_FAULT_LVS_BatteryLow", 0.0f},
//     [TELEM_VCU_NOTICE_HVIL_TERMSENSELOST] = {"VCU_NOTICE_HVIL_TermSenseLost", 0.0f},
//     [TELEM_BATTERY_LOW_VOLTAGE] = {"Battery_Low_Voltage", 0.0f},
//     [TELEM_VCU_MCM_REGEN_MODE] = {"VCU_MCM_RegenMode", 0.0f},
//     [TELEM_VCU_MCM_REGEN_MAX_TORQUE] = {"VCU_MCM_Regen_MaxTorqueNm", 0.0f},
//     [TELEM_SPEED_KPH] = {"Speed_KPH", 0.0f},
//     [TELEM_STEERING_ANGLE] = {"Steering_Angle", 0.0f},
//     [TELEM_VCU_BMS_HIGHEST_CELL_TEMP] = {"VCU_BMS_HighestCellTemp", 0.0f},
//     [TELEM_VCU_BMS_HIGHEST_CELL_VOLTAGE] = {"VCU_BMS_HighestCellVoltage", 0.0f},
//     [TELEM_VCU_BMS_LOWEST_CELL_VOLTAGE] = {"VCU_BMS_LowestCellVoltage", 0.0f},
//     [TELEM_VCU_BMS_HIGHEST_CELL_TEMP_DC] = {"VCU_BMS_HighestCellTemp_dC", 0.0f},
//     [TELEM_VCU_POWERLIMIT_PID_TOTAL_ERROR] = {"VCU_POWERLIMIT_PID_getTotalError", 0.0f},
//     [TELEM_VCU_POWERLIMIT_PID_PROPORTIONAL] = {"VCU_POWERLIMIT_PID_getProportional", 0.0f},
//     [TELEM_VCU_POWERLIMIT_PID_INTEGRAL] = {"VCU_POWERLIMIT_PID_getIntegral", 0.0f},
//     [TELEM_VCU_POWERLIMIT_TORQUE_COMMAND] = {"VCU_POWERLIMIT_getTorqueCommand_Nm", 0.0f},
//     [TELEM_VCU_LAUNCH_CONTROL_TORQUE_CMD] = {"VCU_LaunchControl_getTorqueCommand_Nm", 0.0f},
//     [TELEM_VCU_LAUNCH_CONTROL_SLIP_RATIO] = {"VCU_LaunchControl_getSlipRatioScaled", 0.0f},
//     [TELEM_VCU_LAUNCH_CONTROL_PID_OUTPUT] = {"VCU_LaunchControl_getPidOutput", 0.0f},
//     [TELEM_VCU_LAUNCH_CONTROL_PID_PROP] = {"VCU_LaunchControl_PID_Proportional", 0.0f},
//     [TELEM_VCU_LAUNCH_CONTROL_PID_INTEGRAL] = {"VCU_LaunchControl_PID_Integral", 0.0f},
//     [TELEM_VCU_LAUNCH_CONTROL_PID_TOTAL_ERROR] = {"VCU_LaunchControl_PID_TotalError", 0.0f},
//     [TELEM_BMS_IMMINENT_CONTACTOR_WARNING] = {"BMS_Imminent_Contactor_Opening_Warning", 0.0f},
//     [TELEM_BMS_CELL_UNDER_VOLTAGE_FAULT] = {"BMS_Cell_Under_Voltage_Fault", 0.0f},
//     [TELEM_BMS_CELL_OVER_TEMP_FAULT] = {"BMS_Cell_Over_Temperature_Fault", 0.0f},
//     [TELEM_BMS_PACK_UNDER_VOLTAGE_FAULT] = {"BMS_Pack_Under_Voltage_Fault", 0.0f},
//     [TELEM_BMS_ISOLATION_LEAKAGE_FAULT] = {"BMS_Isolation_Leakage_Fault", 0.0f},
//     [TELEM_BMS_PRECHARGE_FAULT] = {"BMS_Precharge_Fault", 0.0f},
//     [TELEM_BMS_FAILED_THERMISTOR_FAULT] = {"BMS_Failed_Thermistor_Fault", 0.0f},
//     [TELEM_BMS_CURRENT_STATE] = {"BMS_Current_State", 0.0f},
//     [TELEM_BMS_MAIN_CONTACTOR_POS_CLOSED] = {"BMS_Main_Contactor_Positive_Closed", 0.0f},
//     [TELEM_BMS_MAIN_CONTACTOR_NEG_CLOSED] = {"BMS_Main_Contactor_Negative_Closed", 0.0f},
//     [TELEM_BMS_PACK_VOLTAGE] = {"Pack_Voltage", 0.0f},
//     [TELEM_BMS_STATE_OF_CHARGE] = {"BMS_State_Of_Charge", 0.0f},
//     [TELEM_BMS_HIGHEST_CELL_TEMPERATURE] = {"BMS_Highest_Cell_Temperature", 0.0f},
//     [TELEM_HUMIDITY] = {"Humidity", 0.0f},
//     [TELEM_TEMPERATURE] = {"Temperature", 0.0f},
// };

void print_crash_timestamp() {
    time_t curr_time = get_relative_time();
    struct tm timeinfo;
    localtime_r(&curr_time, &timeinfo);

    ESP_LOGI(TAG, "Crash detected at relative time: %04d-%02d-%02d %02d:%02d:%02d",
        timeinfo.tm_year + 1900,
        timeinfo.tm_mon + 1,
        timeinfo.tm_mday,
        timeinfo.tm_hour,
        timeinfo.tm_min,
        timeinfo.tm_sec);
}

void handleLightSleepState(){
    
    esp_err_t ret;
    ret = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0);

    printf("\nentering deepsleep\n");
    fflush(stdout);
    esp_deep_sleep_start();
}
int count = 0;
void canReceive() {
    twai_message_t rx_msg;
    esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(10));
    
    if (result == ESP_OK) {
		count = 0;
        if (rx_msg.extd == 0 && rx_msg.rtr == 0) {
            parseCanMessages(rx_msg.identifier, rx_msg.data);
            //vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            printf("Ignored message: extended=%d, rtr=%d\n", rx_msg.extd, rx_msg.rtr);
        }
    }
    else {
        printf("Error receiving CAN message: %s\n", esp_err_to_name(result));
		count++;
		if(count == 500){
			currentState = SLEEP_STATE;
		}
		
    }
}

void parseCanMessages(uint32_t msg_id, uint8_t data[8]){
	switch (msg_id)
	{
		case 0x100: //EMeter_Measurement

			//EMeter_Current (big endian-)
			uint32_t rawCurrent = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);

			//EMeter_Voltage (big endian-)
			uint32_t rawVoltage = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7]);

			//type case to signed
			int32_t signedCurrent = (int32_t)rawCurrent;
			int32_t signedVoltage = (int32_t)rawVoltage;
		
			//Scale factors
			float emeterCurrent = (float)signedCurrent * 1.5258789063e-005f;
			float emeterVoltage = (float)signedVoltage * 1.5258789063e-005f;
			printf("CAN R: EMeter Current: %.6f A\n", emeterCurrent);
			printf("CAN R: EMeter Voltage: %.6f V\n", emeterVoltage);

            update_telemetry_value_by_name("EMeter_Current", emeterCurrent);
            update_telemetry_value_by_name("EMeter_Voltage", emeterVoltage);
			break;
		
		case 0xA5: //MCM_Motor_Position_Info

			//MCM_Motor_Speed (little endian-)
			uint16_t rawMotorSpeed = (data[3] << 8) | (data[2]);
			int16_t sMotorSpeed = (int16_t)rawMotorSpeed;
			float mcmMotorSpeed = (float)sMotorSpeed;
			printf("CAN R: MCM MotorSpeed: %.1f RPM\n",mcmMotorSpeed);

            update_telemetry_value_by_name("MCM_Motor_Speed", mcmMotorSpeed);
			break;
		case 0xA6: //MCM_Current_Info

			//MCM_DC_Bus_Current (little endian-)
			uint16_t rawDCBusCurrent = (data[7] << 8) | (data[6]);
			int16_t sDCBusCurrent = (int16_t)rawDCBusCurrent;
			float mcmDCBusCurrent = (float)sDCBusCurrent * 0.1f;
			printf("CAN R: MCM DCBus Current: %.2f A\n",mcmDCBusCurrent);

            update_telemetry_value_by_name("MCM_DCBus_Current", mcmDCBusCurrent);
			break;

		case 0xA7: //MCM_Voltage_Info
			//MCM_DC_Bus_Voltage (little e-)
			uint16_t rawDCBusVoltage = (data[1] << 8) | (data[0]);
			int16_t sDCBusVoltage = (int16_t)rawDCBusVoltage;
			float mcmDCBusVoltage = (float)sDCBusVoltage * 0.1f;
			printf("CAN R: MCM DCBus Voltage: %.2f V\n",mcmDCBusVoltage);

            update_telemetry_value_by_name("MCM_DCBus_Voltage", mcmDCBusVoltage);
			break;

		case 0xAA: //MCM_Internal_States
			//MCM_Int_Invert_Enable_State (little e+)
			uint8_t rawIntInvertEnableState = (data[6] >> 0) & 0x01;

			//MCM_Int_Inverter_State (little e+)
			uint8_t rawIntInverterState = (data[2]);
			
			float mcmIntInvertEnableState = (float)rawIntInvertEnableState;
			float mcmIntInverterState = (float)rawIntInverterState;
			printf("CAN R: MCM IntInvert EnableState: %.0f \n",mcmIntInvertEnableState);
			printf("CAN R: MCM IntInverter State: %.0f \n",mcmIntInverterState);

            update_telemetry_value_by_name("MCM_IntInvert_EnableState", mcmIntInvertEnableState);
            update_telemetry_value_by_name("MCM_IntInverter_State", mcmIntInverterState);
			break;

		case 0xAC: //MCM_Torque_And_Timer_Info
			//MCM_Torque_Feedback (little e-)
			uint16_t rawTorqueFeedback = (data[3] << 8) | (data[2]);

			//MCM_Commanded_Torque (little e-)
			uint16_t rawCommandedTorque = (data[1] << 8) | (data[0]);
			
			int16_t sTorqueFeedback = (int16_t)rawTorqueFeedback;
			int16_t sCommandedTorque = (int16_t)rawCommandedTorque;

			float mcmTorqueFeedback = (float)sTorqueFeedback * 0.1f;
			float mcmCommandedTorque = (float)sCommandedTorque * 0.1f;
			printf("CAN R: MCM TorqueFeedback: %.2f Nm\n",mcmTorqueFeedback);
			printf("CAN R: MCM CommandedTorque: %.2f Nm\n",mcmCommandedTorque);

            update_telemetry_value_by_name("MCM_Torque_Feedback", mcmTorqueFeedback);
            update_telemetry_value_by_name("MCM_Commanded_Torque", mcmCommandedTorque);
			break;

		case 0xC0: //MCM_Command_Messages
			//MCM_Speed_Command (little endian-)
            uint16_t rawSpeedCommand = (data[3] << 8) | (data[2]);

            //MCM_Torque_Command (little endian-)
            uint16_t rawTorqueCommand = (data[1] << 8) | (data[0]);

            //MCM_Speed_Mode_Enable (bit 2 of byte 5, little endian+)
            uint8_t rawSpeedModeEnable = (data[5] >> 2) & 0x01;

            int16_t signedTorqueCommand = (int16_t)rawTorqueCommand;
            int16_t signedSpeedCommand  = (int16_t)rawSpeedCommand;

            float mcmTorqueCommand = (float)signedTorqueCommand * 0.1f;   
            float mcmSpeedCommand  = (float)signedSpeedCommand * 1.0f;    
            float mcmSpeedModeEnable = (float)rawSpeedModeEnable;         

            printf("CAN R: MCM TorqueCmd: %.1f Nm\n", mcmTorqueCommand);
            printf("CAN R: MCM SpeedCmd: %.0f rpm\n", mcmSpeedCommand);
            printf("CAN R: MCM SpeedModeEnable: %.0f\n", mcmSpeedModeEnable);

            update_telemetry_value_by_name("MCM_Torque_Command", mcmTorqueCommand);
            update_telemetry_value_by_name("MCM_Speed_Command", mcmSpeedCommand);
            update_telemetry_value_by_name("MCM_Speed_Mode_Enable", mcmSpeedModeEnable);
			break;

		case 0x501: //VCU_TPS1
			
			//ThrottlePercent0FF (little endian +)
            uint8_t rawThrottlePercent = data[0];

            float tps1ThrottlePercent = (float)rawThrottlePercent;

            printf("CAN R: TPS1 Throttle Percent: %.1f %%\n", tps1ThrottlePercent);

            update_telemetry_value_by_name("TPS1_Throttle_Percent", tps1ThrottlePercent);
			break;
        
        case 0x502: //VCU_BPS0
			
			//BrakePercent0FF (little e +)
            uint8_t rawBrakePercent = data[0];

            float bps0BrakePercent = (float)rawBrakePercent * 0.392156862746f;

            printf("CAN R: BPS0 Brake Percent: %.2f %%\n", bps0BrakePercent);

            update_telemetry_value_by_name("BPS0_Brake_Percent", bps0BrakePercent);
			break;

		case 0x505: //VCU_WSS_Smooth
			//VCU_WSS_FL_S (little e +)
			uint16_t rawWSS_FL_S = (data[1] << 8) | (data[0]);
			//VCU_WSS_FR_S (little e +)
			uint16_t rawWSS_FR_S = (data[3] << 8) | (data[2]);
			//VCU_WSS_RL_S (little e +)
			uint16_t rawWSS_RL_S = (data[5] << 8) | (data[4]);
			//VCU_WSS_RR_S (little e +)
			uint16_t rawWSS_RR_S = (data[7] << 8) | (data[6]);

			float VCU_WSS_FL_S = (float)rawWSS_FL_S;
			float VCU_WSS_FR_S = (float)rawWSS_FR_S;
			float VCU_WSS_RL_S = (float)rawWSS_RL_S;
			float VCU_WSS_RR_S = (float)rawWSS_RR_S;

			printf("CAN R: VCU_WSS_FL_S: %.2f RPM\n",VCU_WSS_FL_S);
			printf("CAN R: VCU_WSS_FR_S: %.2f RPM\n",VCU_WSS_FR_S);
			printf("CAN R: VCU_WSS_RL_S: %.2f RPM\n",VCU_WSS_RL_S);
			printf("CAN R: VCU_WSS_RR_S: %.2f RPM\n",VCU_WSS_RR_S);

            update_telemetry_value_by_name("VCU_WSS_FL_S", VCU_WSS_FL_S);
            update_telemetry_value_by_name("VCU_WSS_FR_S", VCU_WSS_FR_S);
            update_telemetry_value_by_name("VCU_WSS_RL_S", VCU_WSS_RL_S);
            update_telemetry_value_by_name("VCU_WSS_RR_S", VCU_WSS_RR_S);
			break;

		case 0x506: //VCU_Safety_Checker
			//VCU_FAULT_TPS_OutOfRange (little endian +)
            uint8_t rawFAULT_TPS_OutOfRange = (data[0] >> 0) & 0x01;

            //VCU_FAULT_BPS_OutOfRange (little endian +)
            uint8_t rawFAULT_BPS_OutOfRange = (data[0] >> 1) & 0x01;

            //VCU_FAULT_TPS_OutOfSync (little endian +)
            uint8_t rawFAULT_TPS_OutOfSync = (data[1] >> 0) & 0x01;

            //VCU_FAULT_TPSBPS_Implausible (little endian +)
            uint8_t rawFAULT_TPSBPS_Implausible = (data[1] >> 2) & 0x01;

            //VCU_FAULT_BSPD_SoftFault (little endian +)
            uint8_t rawFAULT_BSPD_SoftFault = (data[1] >> 4) & 0x01;

            //VCU_WARNING_LVS_BatteryLow (little endian +)
            uint8_t rawWARNING_LVS_BatteryLow = (data[4] >> 0) & 0x01;

            //VCU_NOTICE_HVIL_TermSenseLost (little endian +)
            uint8_t rawNOTICE_HVIL_TermSenseLost = (data[6] >> 0) & 0x01;

            printf("CAN R: VCU_FAULT_TPS_OutOfRange: %d\n", rawFAULT_TPS_OutOfRange);
            printf("CAN R: VCU_FAULT_BPS_OutOfRange: %d\n", rawFAULT_BPS_OutOfRange);
            printf("CAN R: VCU_FAULT_TPS_OutOfSync: %d\n", rawFAULT_TPS_OutOfSync);
            printf("CAN R: VCU_FAULT_TPSBPS_Implausible: %d\n", rawFAULT_TPSBPS_Implausible);
            printf("CAN R: VCU_FAULT_BSPD_SoftFault: %d\n", rawFAULT_BSPD_SoftFault);
            printf("CAN R: VCU_WARNING_LVS_BatteryLow: %d\n", rawWARNING_LVS_BatteryLow);
            printf("CAN R: VCU_NOTICE_HVIL_TermSenseLost: %d\n", rawNOTICE_HVIL_TermSenseLost);

            update_telemetry_value_by_name("VCU_Fault_TPS_OutOfRange", rawFAULT_TPS_OutOfRange);
            update_telemetry_value_by_name("VCU_Fault_BPS_OutOfRange", rawFAULT_BPS_OutOfRange);
            update_telemetry_value_by_name("VCU_FAULT_TPS_OutOfSync", rawFAULT_TPS_OutOfSync);
            update_telemetry_value_by_name("VCU_FAULT_TPSBPS_Implausible", rawFAULT_TPSBPS_Implausible);
            update_telemetry_value_by_name("VCU_FAULT_BSPD_SoftFault", rawFAULT_BSPD_SoftFault);
            update_telemetry_value_by_name("VCU_FAULT_LVS_BatteryLow", rawWARNING_LVS_BatteryLow);
            update_telemetry_value_by_name("VCU_NOTICE_HVIL_TermSenseLost", rawNOTICE_HVIL_TermSenseLost);

			if (imu_crash_event == 1 && !vcu_can_captured) {
				crash_record_t vcu_record = {0};
				vcu_record.g_force = 6.6f;
				vcu_record.accel[0] = 6.6f;
				vcu_record.accel[1] = 6.7f;
				vcu_record.accel[2] = 6.6f;
				vcu_record.vcu_can_id = msg_id;
				vcu_record.vcu_can_dlc = 8;
				memcpy(vcu_record.vcu_can_data, data, 8);
				vcu_record.vcu_captured = true;
				vcu_save_crash_record(&vcu_record);
				vcu_can_captured = true;
			}
			
			break;

        case 0x507: //Low_Voltage
			//Voltage (little e +)
            uint16_t rawLVvoltage = (data[1] << 8) | data[0];

            float LVVoltage = (float)rawLVvoltage * 0.001f;

            printf("CAN R: Voltage: %.3f V\n", LVVoltage);

            update_telemetry_value_by_name("Battery_Low_Voltage", LVVoltage);
			break;

        case 0x508: //VCU_Regen_Settings
			//VCU_MCM_RegenMode (little e +)
            uint8_t VCU_MCM_RegenMode = data[0];

            //VCU_MCM_MaxTorqueNm (little e +)
            uint8_t rawMaxTorque = data[2];
            float VCU_MCM_MaxTorqueNm = (float)rawMaxTorque;

            printf("CAN R: VCU_MCM_RegenMode: %u\n", VCU_MCM_RegenMode);
            printf("CAN R: VCU_MCM_MaxTorqueNm: %.1f Nm\n", VCU_MCM_MaxTorqueNm);

            update_telemetry_value_by_name("VCU_MCM_RegenMode", VCU_MCM_RegenMode);
            update_telemetry_value_by_name("VCU_MCM_Regen_MaxTorqueNm", VCU_MCM_MaxTorqueNm);
			
            break;
        
		case 0x50A: //Ground_Speed
			//speedKph (little e +)
			uint16_t rawSpeedKPH = (data[1] << 8) | (data[0]);

			float SpeedKPH = (float)rawSpeedKPH;
			printf("CAN R: SpeedKPH: %.2f km/h\n", SpeedKPH);

            update_telemetry_value_by_name("Speed_KPH", SpeedKPH);
			break;

		case 0x50B: //VCU_Launch_Control_Status_A
			//VCU_LaunchControl_getTorqueCommand_Nm (little e-)
            uint16_t rawLCTorqueCommand = (data[1] << 8) | data[0];
            //VCU_LaunchControl_getSlipRatioScaled (little e-)
            uint16_t rawLCSlipRatioScaled = (data[3] << 8) | data[2];
            //VCU_LaunchControl_getPidOutput (little e-)
            uint16_t rawLCPidOutput = (data[7] << 8) | data[6];

            int16_t torqueCommand = (int16_t)rawLCTorqueCommand;
            int16_t slipRatioScaled = (int16_t)rawLCSlipRatioScaled;
            int16_t pidOutput = (int16_t)rawLCPidOutput;

            float VCU_LaunchControl_getTorqueCommand_Nm = (float)torqueCommand;
            float VCU_LaunchControl_getSlipRatioScaled = (float)slipRatioScaled;
            float VCU_LaunchControl_getPidOutput = (float)pidOutput;

            printf("CAN R: VCU_LaunchControl_getTorqueCommand_Nm: %.1f Nm\n", VCU_LaunchControl_getTorqueCommand_Nm);
            printf("CAN R: VCU_LaunchControl_getSlipRatioScaled: %.1f\n", VCU_LaunchControl_getSlipRatioScaled);
            printf("CAN R: VCU_LaunchControl_getPidOutput: %.1f\n", VCU_LaunchControl_getPidOutput);

            update_telemetry_value_by_name("VCU_LaunchControl_getTorqueCommand_Nm", VCU_LaunchControl_getTorqueCommand_Nm);
            update_telemetry_value_by_name("VCU_LaunchControl_getSlipRatioScaled", VCU_LaunchControl_getSlipRatioScaled);
            update_telemetry_value_by_name("VCU_LaunchControl_getPidOutput", VCU_LaunchControl_getPidOutput);

			break;

		case 0x50C: //DRS_SAS
			//Steering_Angle (little e -)
			uint16_t rawSteering_Angle = (data[1] << 8) | (data[0]);

			int16_t sSteering_Angle = (int16_t)rawSteering_Angle;
			float Steering_Angle = (float)sSteering_Angle;

			printf("CAN R: Steering_Angle: %.2f Degrees\n", Steering_Angle);

            update_telemetry_value_by_name("Steering_Angle", Steering_Angle);
			break;
        
        case 0x50E: //VCU_BMS_Debug_1
			//VCU_BMS_HighestCellTemp (little endian +)
            uint16_t rawHighestCellTemp = (data[4] << 8) | data[3];

            float VCU_BMS_HighestCellTemp = (float)rawHighestCellTemp;

            printf("CAN R: VCU_BMS_HighestCellTemp: %.0f\n", VCU_BMS_HighestCellTemp);

            update_telemetry_value_by_name("VCU_BMS_HighestCellTemp", VCU_BMS_HighestCellTemp);
			break;    

        case 0x50F: //VCU_BMS_Debug_2
			//VCU_BMS_HighestCellVoltage (little endian +)
            uint16_t rawHighestCellVoltage = (data[1] << 8) | data[0];
            float VCU_BMS_HighestCellVoltage = (float)rawHighestCellVoltage;

            //VCU_BMS_LowestCellVoltage (little endian +)
            uint16_t rawLowestCellVoltage = (data[3] << 8) | data[2];
            float VCU_BMS_LowestCellVoltage = (float)rawLowestCellVoltage;

            //VCU_BMS_HighestCellTemp_dC (little endian -)
            uint16_t rawHighestCellTemp_dC = (data[5] << 8) | data[4];
            int16_t sHighestCellTemp_dC = (int16_t)rawHighestCellTemp_dC;
            float VCU_BMS_HighestCellTemp_dC = (float)sHighestCellTemp_dC;

            printf("CAN R: VCU_BMS_HighestCellVoltage: %.0f V\n", VCU_BMS_HighestCellVoltage);
            printf("CAN R: VCU_BMS_LowestCellVoltage: %.0f V\n", VCU_BMS_LowestCellVoltage);
            printf("CAN R: VCU_BMS_HighestCellTemp_dC: %.0f C\n", VCU_BMS_HighestCellTemp_dC);

            update_telemetry_value_by_name("VCU_BMS_HighestCellVoltage", VCU_BMS_HighestCellVoltage);
            update_telemetry_value_by_name("VCU_BMS_LowestCellVoltage", VCU_BMS_LowestCellVoltage);
            update_telemetry_value_by_name("VCU_BMS_HighestCellTemp_dC", VCU_BMS_HighestCellTemp_dC);
			break;   

        case 0x511: //VCU_Power_Limit_Status_AMsg
			//VCU_POWERLIMIT_PID_getTotalError (little endian -)
            uint16_t rawTotalError = (data[4] << 8) | data[3];
            int16_t sTotalError = (int16_t)rawTotalError;
            float VCU_POWERLIMIT_PID_getTotalError = (float)sTotalError;

            //VCU_POWERLIMIT_PID_getProportional (little endian -)
            uint16_t rawProportional = (data[6] << 8) | data[5];
            int16_t sProportional = (int16_t)rawProportional;
            float VCU_POWERLIMIT_PID_getProportional = (float)sProportional;

            printf("CAN R: VCU_POWERLIMIT_PID_getTotalError: %.0f\n", VCU_POWERLIMIT_PID_getTotalError);
            printf("CAN R: VCU_POWERLIMIT_PID_getProportional: %.0f\n", VCU_POWERLIMIT_PID_getProportional);

            update_telemetry_value_by_name("VCU_POWERLIMIT_PID_getTotalError", VCU_POWERLIMIT_PID_getTotalError);
            update_telemetry_value_by_name("VCU_POWERLIMIT_PID_getProportional", VCU_POWERLIMIT_PID_getProportional);
			break;  

        case 0x512: //VCU_Power_Limit_Status_BMsg
			//VCU_POWERLIMIT_PID_getIntegral (little endian -)
            uint16_t rawIntegral = (data[1] << 8) | data[0];
            int16_t sIntegral = (int16_t)rawIntegral;
            float VCU_POWERLIMIT_PID_getIntegral = (float)sIntegral;

            //VCU_POWERLIMIT_getTorqueCommand_Nm (little endian -)
            uint16_t PLrawTorqueCommand = (data[3] << 8) | data[2];
            int16_t sTorqueCommand = (int16_t)PLrawTorqueCommand;
            float VCU_POWERLIMIT_getTorqueCommand_Nm = (float)sTorqueCommand;

            printf("CAN R: VCU_POWERLIMIT_PID_getIntegral: %.0f\n", VCU_POWERLIMIT_PID_getIntegral);
            printf("CAN R: VCU_POWERLIMIT_getTorqueCommand_Nm: %.1f Nm\n", VCU_POWERLIMIT_getTorqueCommand_Nm);

            update_telemetry_value_by_name("VCU_POWERLIMIT_PID_getIntegral", VCU_POWERLIMIT_PID_getIntegral);
            update_telemetry_value_by_name("VCU_POWERLIMIT_getTorqueCommand_Nm", VCU_POWERLIMIT_getTorqueCommand_Nm);
			break;  

        case 0x513: //VCU_Launch_Control_Status_B
			//VCU_LaunchControl_PID_Proportional (little endian -)
            uint16_t rawPID_Proportional = (data[1] << 8) | data[0];
            int16_t sPID_Proportional = (int16_t)rawPID_Proportional;
            float VCU_LaunchControl_PID_Proportional = (float)sPID_Proportional;

            //VCU_LaunchControl_PID_Integral (little endian -)
            uint16_t rawPID_Integral = (data[3] << 8) | data[2];
            int16_t sPID_Integral = (int16_t)rawPID_Integral;
            float VCU_LaunchControl_PID_Integral = (float)sPID_Integral;

            //VCU_LaunchControl_PID_TotalError (little endian -)
            uint16_t rawPID_TotalError = (data[5] << 8) | data[4];
            int16_t sPID_TotalError = (int16_t)rawPID_TotalError;
            float VCU_LaunchControl_PID_TotalError = (float)sPID_TotalError;

            printf("CAN R: VCU_LaunchControl_PID_Proportional: %.0f\n", VCU_LaunchControl_PID_Proportional);
            printf("CAN R: VCU_LaunchControl_PID_Integral: %.0f\n", VCU_LaunchControl_PID_Integral);
            printf("CAN R: VCU_LaunchControl_PID_TotalError: %.0f\n", VCU_LaunchControl_PID_TotalError);

            update_telemetry_value_by_name("VCU_LaunchControl_PID_Proportional", VCU_LaunchControl_PID_Proportional);
            update_telemetry_value_by_name("VCU_LaunchControl_PID_Integral", VCU_LaunchControl_PID_Integral);
            update_telemetry_value_by_name("VCU_LaunchControl_PID_TotalError", VCU_LaunchControl_PID_TotalError);
            break;

        case 0x602: //BMS_Master_Faults
            // BMS_Imminent_Contactor_Opening_Warning (bit 16)
            uint8_t rawImminentContactorWarning = (data[2] >> 0) & 0x01;

            // BMS_Cell_Under_Voltage_Fault (bit 9)
            uint8_t rawCellUnderVoltageFault = (data[1] >> 1) & 0x01;

            // BMS_Cell_Over_Temperature_Fault (bit 10)
            uint8_t rawCellOverTempFault = (data[1] >> 2) & 0x01;

            // BMS_Pack_Under_Voltage_Fault (bit 13)
            uint8_t rawPackUnderVoltageFault = (data[1] >> 5) & 0x01;

            // BMS_Isolation_Leakage_Fault (bit 0)
            uint8_t rawIsolationLeakageFault = (data[0] >> 0) & 0x01;

            // BMS_Precharge_Fault (bit 2)
            uint8_t rawPrechargeFault = (data[0] >> 2) & 0x01;

            // BMS_Failed_Thermistor_Fault (bit 5)
            uint8_t rawFailedThermistorFault = (data[0] >> 5) & 0x01;

            printf("CAN R: BMS_Imminent_Contactor_Opening_Warning: %d\n", rawImminentContactorWarning);
            printf("CAN R: BMS_Cell_Under_Voltage_Fault: %d\n", rawCellUnderVoltageFault);
            printf("CAN R: BMS_Cell_Over_Temperature_Fault: %d\n", rawCellOverTempFault);
            printf("CAN R: BMS_Pack_Under_Voltage_Fault: %d\n", rawPackUnderVoltageFault);
            printf("CAN R: BMS_Isolation_Leakage_Fault: %d\n", rawIsolationLeakageFault);
            printf("CAN R: BMS_Precharge_Fault: %d\n", rawPrechargeFault);
            printf("CAN R: BMS_Failed_Thermistor_Fault: %d\n", rawFailedThermistorFault);

            update_telemetry_value_by_name("BMS_Imminent_Contactor_Opening_Warning", rawImminentContactorWarning);
            update_telemetry_value_by_name("BMS_Cell_Under_Voltage_Fault", rawCellUnderVoltageFault);
            update_telemetry_value_by_name("BMS_Cell_Over_Temperature_Fault", rawCellOverTempFault);
            update_telemetry_value_by_name("BMS_Pack_Under_Voltage_Fault", rawPackUnderVoltageFault);
            update_telemetry_value_by_name("BMS_Isolation_Leakage_Fault", rawIsolationLeakageFault);
            update_telemetry_value_by_name("BMS_Precharge_Fault", rawPrechargeFault);
            update_telemetry_value_by_name("BMS_Failed_Thermistor_Fault", rawFailedThermistorFault);

			if (imu_crash_event == 1 && !bms_can_captured) {
				crash_record_t bms_record = {0};
				bms_record.g_force = 6.6f;
				bms_record.accel[0] = 6.6f;
				bms_record.accel[1] = -0.6f;
				bms_record.accel[2] = 6.6f;
				bms_record.bms_can_id = msg_id;
				bms_record.bms_can_dlc = 8;
				memcpy(bms_record.bms_can_data, data, 8);
				bms_record.bms_captured = true;
				bms_save_crash_record(&bms_record);
				bms_can_captured = true;
			}

			break;

		case 0x610: //BMS_Master_System_Status
			// BMS_Current_State (8-bit, little endian)
            uint8_t rawBMS_Current_State = data[6]; // bit 48 → byte 6
            float BMS_Current_State = (float)rawBMS_Current_State;

            // BMS_Main_Contactor_Positive_Closed (bit 25)
            uint8_t rawMainContactorPosClosed = (data[3] >> 1) & 0x01;

            // BMS_Main_Contactor_Negative_Closed (bit 26)
            uint8_t rawMainContactorNegClosed = (data[3] >> 2) & 0x01;

            printf("CAN R: BMS_Current_State: %.0f\n", BMS_Current_State);
            printf("CAN R: BMS_Main_Contactor_Positive_Closed: %d\n", rawMainContactorPosClosed);
            printf("CAN R: BMS_Main_Contactor_Negative_Closed: %d\n", rawMainContactorNegClosed);

            update_telemetry_value_by_name("BMS_Current_State", BMS_Current_State);
            update_telemetry_value_by_name("BMS_Main_Contactor_Positive_Closed", rawMainContactorPosClosed);
            update_telemetry_value_by_name("BMS_Main_Contactor_Negative_Closed", rawMainContactorNegClosed);
			break;

        case 0x620: //BMS_Pack_Level_Measurements_1
			// BMS_Pack_Voltage (32-bit, little endian)
            uint32_t rawBMS_Pack_Voltage = 
            (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];

            float BMS_Pack_Voltage = (float)rawBMS_Pack_Voltage * 0.001f;

            printf("CAN R: BMS_Pack_Voltage: %.3f V\n", BMS_Pack_Voltage);
            update_telemetry_value_by_name("Pack_Voltage", BMS_Pack_Voltage);
			break;

        case 0x621: //BMS_Pack_Level_Measurements_2
            //BMS_State_Of_Charge, little e+
			uint16_t rawBMS_SOC = (data[7] << 8) | data[6];
            float BMS_State_Of_Charge = (float)rawBMS_SOC * 0.1f;

            printf("CAN R: BMS_State_Of_Charge: %.1f %%\n", BMS_State_Of_Charge);
            update_telemetry_value_by_name("BMS_State_Of_Charge", BMS_State_Of_Charge);
			break;

        case 0x623: //BMS_Cell_Temperature_Summary
			// BMS_Highest_Cell_Temperature (little e-)
            int16_t rawBMS_HighestCellTemp = (data[7] << 8) | data[6];
            float BMS_HighestCellTemp = (float)rawBMS_HighestCellTemp * 0.1f;

            printf("CAN R: BMS_Highest_Cell_Temperature: %.1f DegC\n", BMS_HighestCellTemp);
            update_telemetry_value_by_name("BMS_Highest_Cell_Temperature", BMS_HighestCellTemp);
			break;

		default:
		printf("Unknown CAN ID: 0x%03" PRIX32 "\n", msg_id);
			break;
		}
}

//DATA
// void print_all_telemetry(void) {
//     printf("\n=== TELEMETRY DATA ===\n");
//     for (int i = 0; i < TELEM_COUNT; i++) {
//         if (telemetry_data[i].value != 0.0f) { // Only print non-zero values
//             printf("%-35s: %.4f\n", telemetry_data[i].name, telemetry_data[i].value);
//         }
//     }
//     printf("======================\n");
// }

// // Get by name
// float get_telemetry_value_by_name(const char* name) {
//     for (int i = 0; i < TELEM_COUNT; i++) {
//         if (strcmp(telemetry_data[i].name, name) == 0) {
//             return telemetry_data[i].value;
//         }
//     }
//     printf("Warning: Telemetry name '%s' not found\n", name);
//     return 0.0f;
// }

// // Update by name
// void update_telemetry_value_by_name(const char* name, float value) {
//     for (int i = 0; i < TELEM_COUNT; i++) {
//         if (strcmp(telemetry_data[i].name, name) == 0) {
//             telemetry_data[i].value = value;
//             return;
//         }
//     }
//     printf("Warning: Telemetry name '%s' not found\n", name);
// }

// // Initialize telemetry data
// void init_telemetry_data(void) {
//     for (int i = 0; i < TELEM_COUNT; i++) {
//         telemetry_data[i].value = 0.0f;
//     }
// }

void vcu_save_crash_record(crash_record_t *record) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("v_crash_log", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        printf("VCU NVS open failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(nvs, "v_crash_flag", 1);
    if (err != ESP_OK) printf("VCU flag set failed: %s\n", esp_err_to_name(err));

    err = nvs_set_blob(nvs, "v_crash_record", record, sizeof(crash_record_t));
    if (err != ESP_OK) printf("VCU blob set failed: %s\n", esp_err_to_name(err));

    err = nvs_commit(nvs);
    if (err != ESP_OK) printf("VCU commit failed: %s\n", esp_err_to_name(err));
    else printf("VCU crash record saved successfully.\n");

    nvs_close(nvs);
}

void bms_save_crash_record(crash_record_t *record) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("b_crash_log", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        printf("BMS NVS open failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(nvs, "b_crash_flag", 1);
    if (err != ESP_OK) printf("BMS flag set failed: %s\n", esp_err_to_name(err));

    err = nvs_set_blob(nvs, "b_crash_record", record, sizeof(crash_record_t));
    if (err != ESP_OK) printf("BMS blob set failed: %s\n", esp_err_to_name(err));

    err = nvs_commit(nvs);
    if (err != ESP_OK) printf("BMS commit failed: %s\n", esp_err_to_name(err));
    else printf("BMS crash record saved successfully.\n");

    nvs_close(nvs);
}

void vcu_read_record() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("v_crash_log", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        printf("Failed to open VCU NVS namespace: %s\n", esp_err_to_name(err));
        return;
    }

    uint8_t flag = 0;
    err = nvs_get_u8(nvs, "v_crash_flag", &flag);
    if (err == ESP_ERR_NVS_NOT_FOUND || flag != 1) {
        printf("No VCU crash record found.\n");
        nvs_close(nvs);
        return;
    } else if (err != ESP_OK) {
        printf("Error reading VCU crash flag: %s\n", esp_err_to_name(err));
        nvs_close(nvs);
        return;
    }

    crash_record_t record;
    size_t size = sizeof(record);
    err = nvs_get_blob(nvs, "v_crash_record", &record, &size);
    if (err == ESP_OK) {
        printf("=== VCU CRASH ===\n");
        printf("G-Force: %.2f g\n", record.g_force);
        printf("Accel: [%.2f, %.2f, %.2f]\n", record.accel[0], record.accel[1], record.accel[2]);
        printf("CAN Msg ID: 0x%lX, DLC: %d\n", record.vcu_can_id, record.vcu_can_dlc);
        printf("Data: ");
        for (int i = 0; i < record.vcu_can_dlc; i++) printf("%02X ", record.vcu_can_data[i]);
        printf("\n");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("VCU crash record not found in NVS.\n");
    } else if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        printf("VCU crash record size mismatch (invalid length).\n");
    } else {
        printf("Failed to read VCU crash record: %s\n", esp_err_to_name(err));
    }

    nvs_close(nvs);
}

void bms_read_record() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("b_crash_log", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        printf("Failed to open BMS NVS namespace: %s\n", esp_err_to_name(err));
        return;
    }

    uint8_t flag = 0;
    err = nvs_get_u8(nvs, "b_crash_flag", &flag);
    if (err == ESP_ERR_NVS_NOT_FOUND || flag != 1) {
        printf("No BMS crash record found.\n");
        nvs_close(nvs);
        return;
    } else if (err != ESP_OK) {
        printf("Error reading BMS crash flag: %s\n", esp_err_to_name(err));
        nvs_close(nvs);
        return;
    }

    crash_record_t record;
    size_t size = sizeof(record);
    err = nvs_get_blob(nvs, "b_crash_record", &record, &size);
    if (err == ESP_OK) {
        printf("=== BMS CRASH ===\n");
        printf("G-Force: %.2f g\n", record.g_force);
        printf("Accel: [%.2f, %.2f, %.2f]\n", record.accel[0], record.accel[1], record.accel[2]);
        printf("CAN Msg ID: 0x%lX, DLC: %d\n", record.bms_can_id, record.bms_can_dlc);
        printf("Data: ");
        for (int i = 0; i < record.bms_can_dlc; i++) printf("%02X ", record.bms_can_data[i]);
        printf("\n");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("BMS crash record not found in NVS.\n");
    } else if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        printf("BMS crash record size mismatch (invalid length).\n");
    } else {
        printf("Failed to read BMS crash record: %s\n", esp_err_to_name(err));
    }

    nvs_close(nvs);
}


void vcu_erase_record() {
    nvs_handle_t nvs;
    if (nvs_open("v_crash_log", NVS_READWRITE, &nvs) != ESP_OK) return;

    nvs_erase_key(nvs, "v_crash_flag");
    nvs_erase_key(nvs, "v_crash_record");
    nvs_commit(nvs);
    nvs_close(nvs);
    vcu_can_captured = false;
    printf("VCU Crash record erased.\n");
}

void bms_erase_record() {
    nvs_handle_t nvs;
    if (nvs_open("b_crash_log", NVS_READWRITE, &nvs) != ESP_OK) return;

    nvs_erase_key(nvs, "b_crash_flag");
    nvs_erase_key(nvs, "b_crash_record");
    nvs_commit(nvs);
    nvs_close(nvs);
    bms_can_captured = false;
    printf("BMS Crash record erased.\n");
}

void handle_crash_event() {
    if (imu_crash_event == 2) {
		vcu_read_record();
		bms_read_record();
	}
    else if (imu_crash_event == 3){
		vcu_erase_record();
		bms_erase_record();
	} 
}

// void dht_test(void *pvParameters)
// {
//     float temperature = 0.0f;
//     float humidity = 0.0f;

//     while (1)
//     {
//         if (dht_read_float_data(SENSOR_TYPE, DHT_GPIO, &humidity, &temperature) == ESP_OK)
//         {
//             printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
//             update_telemetry_value_by_name("Temperature", temperature);
//             update_telemetry_value_by_name("Humidity", humidity);
//         }
//         else
//         {
//             printf("Could not read data from sensor\n");
//         }

//         // Wait 2 seconds between readings
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

void canTask(void* arg){
 while (1) {
        switch (currentState) {
            case SENDING_STATE:
                // In SENDING_STATE, CAN should RECEIVE
              //  rgb_led_set_color(&led1, RED);
                canReceive();  
                break;

            case RECEIVING_STATE:
                // In RECEIVING_STATE, CAN should SEND
                // rgb_led_set_color(&led1, BLUE);
                handle_lora_can_command();
                break;

            case SLEEP_STATE:
                // Idle loop while sleeping
                //vTaskDelay(pdMS_TO_TICKS(500));
                break;

            default:
                
                break;
        }
    }

}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "System starting, init start_time");
    start_time = 1700286000;

	handle_crash_event();   
    init_telemetry_data();

    ESP_LOGI(TAG, "%s",BITRATE);
	ESP_LOGI(TAG, "CTX_GPIO=%d",CONFIG_CTX_GPIO);
	ESP_LOGI(TAG, "CRX_GPIO=%d",CONFIG_CRX_GPIO);

	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(TAG, "Driver installed");
	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(TAG, "Driver started");

	ESP_ERROR_CHECK(i2cdev_init());

	//BUTTON STUFF
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
	//BUTTON STUFF

    //RGB
     ESP_LOGI(TAG, "Starting Fast Rainbow Flash");

    // Create and initialize RGB LED instance
    led1 = rgb_led_new(GPIO_LED_RED, GPIO_LED_GREEN, GPIO_LED_BLUE,
                                 LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2);
    ESP_ERROR_CHECK(rgb_led_init(&led1));

    // Initialize LoRa
    lora_handler_init();
    lora_handler_start();

	xTaskCreate(stateManagerTask, "stateManager", 4096, NULL, 5, &stateManager);

  	xTaskCreatePinnedToCore(
        canTask,                      // Task function
        "canTask",                    // Name of task
        4096,  // Stack size
        NULL,                          // Task parameters
        1,                             // Priority
        NULL,                          // Task handle
        0                              // Core ID (0 = first core, 1 = second core)
    );

// Pin task to core 1 (the "second" core on ESP32)
    // xTaskCreatePinnedToCore(
    //     dht_test,                      // Task function
    //     "dht_test",                    // Name of task
    //     configMINIMAL_STACK_SIZE * 3,  // Stack size
    //     NULL,                          // Task parameters
    //     6,                             // Priority
    //     NULL,                          // Task handle
    //     1                              // Core ID (0 = first core, 1 = second core)
    // );
    humidity_start_task();

    xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, 1);

    vTaskSuspend(NULL);

}

void icm42670_test(void *pvParameters)
{
    // init device descriptor and device
    icm42670_t dev = { 0 };
    ESP_ERROR_CHECK(
        icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // enable accelerometer and gyro in low-noise (LN) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

    // read temperature sensor value once
    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(TAG, "Temperature reading: %f", temperature);

    int16_t raw_reading;
    uint8_t data_register;
    uint8_t CRASH_THRESHOLD = 18;
    /* select which acceleration or gyro value should be read: */
    // data_register = ICM42670_REG_ACCEL_DATA_X1;
    // data_register = ICM42670_REG_ACCEL_DATA_Y1;
    // data_register = ICM42670_REG_ACCEL_DATA_Z1;
    data_register = ICM42670_REG_GYRO_DATA_X1;
    // data_register = ICM42670_REG_GYRO_DATA_Y1;
    // data_register = ICM42670_REG_GYRO_DATA_Z1;

    // now poll selected accelerometer or gyro raw value directly from registers
    while (1)
    {
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, data_register, &raw_reading));

        ESP_LOGI(TAG, "Raw accelerometer / gyro reading: %d", raw_reading);

        if (raw_reading > CRASH_THRESHOLD) {
            if (imu_crash_event != 1) {  // prevent repeated writes
                imu_crash_event = 1; // Mark crash happened
                vTaskDelay(pdMS_TO_TICKS(1000));
                print_crash_timestamp();
                printf("I am in IMU deepsleep...\n");
                handleLightSleepState();
            }
        }

        //vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void handle_lora_can_command(void) {
    if (!lora_cmd.new_command)
        return;

    if (lora_cmd.value == 0) {
        ESP_LOGW("CAN", "Value 0 received for %s, ignoring", lora_cmd.key);
        lora_cmd.new_command = false;
        return;
    }

    twai_message_t msg = {
        .identifier = CAN_ID,
        .extd = 0,
        .data_length_code = 8,
        .data = {0}  // Clear all bytes
    };

    // Map key to byte 1
    if (strcmp(lora_cmd.key, "PL") == 0) {
        msg.data[0] = (uint8_t)lora_cmd.value;  // Mode 1-3
    } else if (strcmp(lora_cmd.key, "Regen") == 0) {
        msg.data[0] = (uint8_t)(lora_cmd.value + 3); // Mode 1-2 mapped to 4-5
    } else if (strcmp(lora_cmd.key, "Efficiency") == 0) {
        msg.data[0] = (uint8_t)(lora_cmd.value + 5); // Mode 1-3 mapped to 6-8
    } else {
        ESP_LOGW("CAN", "Unknown key %s", lora_cmd.key);
        lora_cmd.new_command = false;
        return;
    }

    // Check TWAI state
    twai_status_info_t status;
    twai_get_status_info(&status);
    if (status.state != TWAI_STATE_RUNNING) {
        ESP_LOGE("CAN", "TWAI not running");
        return;
    }

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI("CAN", "Sent %s command on CAN ID 0x%03" PRIX32 " value=%d",
                 lora_cmd.key, msg.identifier, msg.data[0]);
        rgb_led_set_color(&led1, YELLOW);
    } else {
        ESP_LOGE("CAN", "CAN transmit failed: %s", esp_err_to_name(ret));
    }

    // Clear the flag
    lora_cmd.new_command = false;
}
