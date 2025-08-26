#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "esp_flash.h"
#include "font8x8_basic.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/twai.h"

#define BUTTON_PIN 11
#define tag "SSD1306"

#define BUF_SIZE 128

//CRASH STUFF
typedef struct {
    float g_force;
    float accel[3];
	// uint32_t vcu_can_id;
    // uint8_t vcu_can_dlc;
    // uint8_t vcu_can_data[8];
	uint32_t bms_can_id;
    uint8_t bms_can_dlc;
    uint8_t bms_can_data[8];
    // bool vcu_captured;
	bool bms_captured;
} crash_record_t;

static uint8_t imu_crash_event = 2; // 1: Write, 2: Read, 3: Erase
// static bool vcu_can_captured = false;
static bool bms_can_captured = false;

typedef enum {
    SENDING_STATE,
    RECEIVING_STATE,
    SLEEP_STATE
} FINITE_STATES;

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

static TaskHandle_t stateManager = NULL;
SSD1306_t dev;
void stateManagerTask(void* parameter);
void handleSendState(void);
void handleReceiveState(void);
void handleLightSleepState(void);
void setup_i2c_and_ssd1306();
void parseCanMessages(uint32_t msg_id, uint8_t data[8]);
void canReceive();
void canSend();

//CRASH STUFF
void save_crash_record(crash_record_t *record);
void read_crash_record();
void erase_crash_record();
void handle_crash_event();

static const twai_general_config_t g_config =
	TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CTX_GPIO, CONFIG_CRX_GPIO, TWAI_MODE_NORMAL);

int64_t mode_start_time = 0;
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	handle_crash_event();

    setup_i2c_and_ssd1306();
    ESP_LOGI(TAG, "%s",BITRATE);
	ESP_LOGI(TAG, "CTX_GPIO=%d",CONFIG_CTX_GPIO);
	ESP_LOGI(TAG, "CRX_GPIO=%d",CONFIG_CRX_GPIO);

	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(TAG, "Driver installed");
	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(TAG, "Driver started");

	int center, top, bottom;

	top = 2;
	center = 3; 
	bottom = 8;

	//BUTTON STUFF
	gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);
	//BUTTON STUFF

	ssd1306_clear_screen(&dev, false);
	xTaskCreate(stateManagerTask, "stateManager", 4096, NULL, 10, &stateManager);
    vTaskSuspend(NULL);
}

void stateManagerTask(void* parameter){
    currentState = SENDING_STATE;
    
    for(;;){
        switch (currentState)
        {
        case SENDING_STATE: //Master sends to lora and receives from can
            canReceive();
            handleSendState();
            break;
        case RECEIVING_STATE: //Master receives from lora and sends to can
            canSend();
            handleReceiveState();
            break;
        case SLEEP_STATE:
            handleLightSleepState();
            break;
        default:
            printf("Default\n");
            break;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void handleSendState(){
    printf("M1: Sending\n");
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "Mode: Sending", 13, false);
}
void handleReceiveState(){
    printf("M1: Receiving\n");
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "Mode: Receiving", 15, false);
}

void handleLightSleepState(){
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 2, "Mode: Sleep", 11, false);

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
        } else {
            printf("Ignored message: extended=%d, rtr=%d\n", rx_msg.extd, rx_msg.rtr);
        }
    }
    else {
        printf("Error receiving CAN message: %s\n", esp_err_to_name(result));
		count++;
		if(count == 15){
			currentState = SLEEP_STATE;
		}
		
    }
}

void canSend(){
    static const char *TAG = "CAN_SEND";

    twai_message_t msg = {
        .identifier = 0x7FF,
        .extd = 0,
        .data_length_code = 8,
        .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}
    };

    twai_status_info_t status_info;
    twai_get_status_info(&status_info);

    if (status_info.state != TWAI_STATE_RUNNING) {
        ESP_LOGE(TAG, "TWAI not running");
        return;
    }

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN S: Sent ID 0x%03" PRIX32, msg.identifier);
    } else {
        ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(ret));
    }
}

void setup_i2c_and_ssd1306() {
    #if CONFIG_I2C_INTERFACE
        ESP_LOGI(tag, "INTERFACE is i2c");
        ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
        ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
        ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
        i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    #endif // CONFIG_I2C_INTERFACE
    
    #if CONFIG_FLIP
        dev._flip = true;
        ESP_LOGW(tag, "Flip upside down");
    #endif
    
    #if CONFIG_SSD1306_128x64
        ESP_LOGI(tag, "Panel is 128x64");
        ssd1306_init(&dev, 128, 64);
    #endif // CONFIG_SSD1306_128x64
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
			break;
		
		case 0xA5: //MCM_Motor_Position_Info

			//MCM_Motor_Speed (little endian-)
			uint16_t rawMotorSpeed = (data[3] << 8) | (data[2]);
			int16_t sMotorSpeed = (int16_t)rawMotorSpeed;
			float mcmMotorSpeed = (float)sMotorSpeed;
			printf("CAN R: MCM MotorSpeed: %.1f RPM\n",mcmMotorSpeed);
			break;
		case 0xA6: //MCM_Current_Info

			//MCM_DC_Bus_Current (little endian-)
			uint16_t rawDCBusCurrent = (data[7] << 8) | (data[6]);
			int16_t sDCBusCurrent = (int16_t)rawDCBusCurrent;
			float mcmDCBusCurrent = (float)sDCBusCurrent * 0.1f;
			printf("CAN R: MCM DCBus Current: %.2f A\n",mcmDCBusCurrent);
			break;

		case 0xA7: //MCM_Voltage_Info
			//MCM_DC_Bus_Voltage (little e-)
			uint16_t rawDCBusVoltage = (data[1] << 8) | (data[0]);
			int16_t sDCBusVoltage = (int16_t)rawDCBusVoltage;
			float mcmDCBusVoltage = (float)sDCBusVoltage * 0.1f;
			printf("CAN R: MCM DCBus Voltage: %.2f V\n",mcmDCBusVoltage);
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
			break;

		case 0xC0: //MCM_Command_Messages
			//MCM_Torque_Limit_Command (little e-)
			uint16_t rawTorqueLimitCommand = (data[7] << 8) | (data[6]);

			//MCM_Speed_Mode_Enable (little e+)
			uint8_t rawSpeedModeEnable = (data[5] >> 2) & 0x01;
			
			int16_t sTorqueLimitCommand = (int16_t)rawTorqueLimitCommand;
			float mcmTorqueLimitCommand = (float)sTorqueLimitCommand * 0.1f;
			float mcmSpeedModeEnable = (float)rawSpeedModeEnable;

			printf("CAN R: MCM TorqueLimitCommand: %.2f Nm\n",mcmTorqueLimitCommand);
			printf("CAN R: MCM SpeedModeEnable: %.0f \n",mcmSpeedModeEnable);
			break;

		case 0x500: //VCU_TPS0
			//TPS0CalibMin (little e +)
			uint16_t rawCalibMin = (data[5] << 8) | (data[4]);
			//TPS0CalibMax (little e +)
			uint16_t rawCalibMax = (data[7] << 8) | (data[6]);

			float TPS0CalibMin = (float)rawCalibMin * 0.001f;
			float TPS0CalibMax = (float)rawCalibMax * 0.001f;

			printf("CAN R: TPS0 Calibration Min: %.3f V\n",TPS0CalibMin);
			printf("CAN R: TPS0 Calibration Max: %.3f V\n",TPS0CalibMax);
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
			break;

		case 0x506: //VCU_Safety_Checker
			//VCU_FAULT_TPS_OutOfRange (little e +)
			uint8_t rawFAULT_TPS_OutOfRange = (data[0] >> 0) & 0x01;
			//VCU_FAULT_BPS_OutOfRange (little e +)
			uint8_t rawFAULT_BPS_OutOfRange = (data[0] >> 1) & 0x01;
			//VCU_FAULT_TPS_PowerFailure (little e +)
			uint8_t rawFAULT_TPS_PowerFailure = (data[0] >> 2) & 0x01;
			//VCU_FAULT_BPS_PowerFailure (little e +)
			uint8_t rawFAULT_BPS_PowerFailure = (data[0] >> 3) & 0x01;

			//VCU_FAULT_TPS_SignalFailure (little e +)
			uint8_t rawFAULT_TPS_SignalFailure = (data[0]  >> 4) & 0x01;
			//VCU_FAULT_BPS_SignalFailure (little e +)
			uint8_t rawFAULT_BPS_SignalFailure = (data[0] >> 5) & 0x01;
			//VCU_FAULT_TPS_NotCalibrated (little e +)
			uint8_t rawFAULT_TPS_NotCalibrated = (data[0] >> 6) & 0x01;
			//VCU_FAULT_BPS_NotCalibrated (little e +)
			uint8_t rawFAULT_BPS_NotCalibrated = (data[0] >> 7) & 0x01;

			//VCU_FAULT_TPS_OutOfSync (little e +)
			uint8_t rawFAULT_TPS_OutOfSync = (data[1] >> 0) & 0x01;
			//VCU_FAULT_TPSBPS_Implausible (little e +)
			uint8_t rawFAULT_TPSBPS_Implausible = (data[1] >> 2) & 0x01;
			//VCU_FAULT_BSPD_SoftFault (little e +)
			uint8_t rawFAULT_BSPD_SoftFault = (data[1] >> 4) & 0x01;
			//VCU_FAULT_LVS_BatteryEmpty (little e +)
			uint8_t rawFAULT_LVS_BatteryEmpty = (data[2] >> 0) & 0x01;
			//VCU_WARNING_LVS_BatteryLow (little e +)
			uint8_t rawWARNING_LVS_BatteryLow = (data[4] >> 0) & 0x01;
			//VCU_NOTICE_HVIL_TermSenseLost (little e +)
			uint8_t rawNOTICE_HVIL_TermSenseLost = (data[6] >> 0) & 0x01;
			
			printf("CAN R: VCU_FAULT_TPS_OutOfRange: %d\n", rawFAULT_TPS_OutOfRange);
			printf("CAN R: VCU_FAULT_BPS_OutOfRange: %d\n", rawFAULT_BPS_OutOfRange);
			printf("CAN R: VCU_FAULT_TPS_PowerFailure: %d\n", rawFAULT_TPS_PowerFailure);
			printf("CAN R: VCU_FAULT_BPS_PowerFailure: %d\n", rawFAULT_BPS_PowerFailure);
		
			printf("CAN R: VCU_FAULT_TPS_SignalFailure: %d\n", rawFAULT_TPS_SignalFailure);
			printf("CAN R: VCU_FAULT_BPS_SignalFailure: %d\n", rawFAULT_BPS_SignalFailure);
			printf("CAN R: VCU_FAULT_TPS_NotCalibrated: %d\n", rawFAULT_TPS_NotCalibrated);
			printf("CAN R: VCU_FAULT_BPS_NotCalibrated: %d\n", rawFAULT_BPS_NotCalibrated);
		
			printf("CAN R: VCU_FAULT_TPS_OutOfSync: %d\n", rawFAULT_TPS_OutOfSync);
			printf("CAN R: VCU_FAULT_TPSBPS_Implausible: %d\n", rawFAULT_TPSBPS_Implausible);
			printf("CAN R: VCU_FAULT_BSPD_SoftFault: %d\n", rawFAULT_BSPD_SoftFault);
		
			printf("CAN R: VCU_FAULT_LVS_BatteryEmpty: %d\n", rawFAULT_LVS_BatteryEmpty);
			printf("CAN R: VCU_WARNING_LVS_BatteryLow: %d\n", rawWARNING_LVS_BatteryLow);
			printf("CAN R: VCU_NOTICE_HVIL_TermSenseLost: %d\n", rawNOTICE_HVIL_TermSenseLost);
			break;

		case 0x50A: //Ground_Speed
			//speedKph (little e +)
			uint16_t rawSpeedKPH = (data[1] << 8) | (data[0]);

			float SpeedKPH = (float)rawSpeedKPH;
			printf("CAN R: SpeedKPH: %.2f km/h\n", SpeedKPH);
			break;

		case 0x50B: //Launch_Control
			//LCReady (little e +)
			uint8_t rawLCReady = (data[0]);
			//LCStatus (little e +)
			uint8_t rawLCStatus = (data[1]);
			//Torque (little e -)
			uint16_t rawTorque = (data[3] << 8) | (data[2]);
			//SlipRatio (little e -)
			uint16_t rawSlipRatio = (data[5] << 8) | (data[4]);
			//StartTorque (little e +)
			uint8_t rawStartTorque = (data[6]);

			int16_t sTorque = (int16_t)rawTorque;
			int16_t sSlipRatio = (int16_t)rawSlipRatio;

			float Torque = (float)sTorque * 0.1f;
			float SlipRatio = (float)sSlipRatio * 0.1f;
			float StartTorque = (float)rawStartTorque * 0.1f;

			printf("CAN R: LCReady: %d Launch Control Ready\n", rawLCReady);
			printf("CAN R: LCStatus: %d Launch Control Status\n", rawLCStatus);
			printf("CAN R: Torque: %.2f (Nm) Calculated Torque\n", Torque);
			printf("CAN R: SlipRatio: %.2f Slip Ratio\n", SlipRatio);
			printf("CAN R: StartTorque: %.2f Nm\n", StartTorque);
			break;

		case 0x50C: //DRS_SAS
			//Steering_Angle (little e -)
			uint16_t rawSteering_Angle = (data[1] << 8) | (data[0]);
			//DRS_Enable (little e +)
			uint8_t rawDRS_Enable = (data[2]);
			//DRS_Mode (little e +)
			uint8_t rawDRS_Mode = (data[3]);

			int16_t sSteering_Angle = (int16_t)rawSteering_Angle;
			float Steering_Angle = (float)sSteering_Angle;

			printf("CAN R: Steering_Angle: %.2f Degrees\n", Steering_Angle);
			printf("CAN R: DRS_Enable: %d \n", rawDRS_Enable);
			printf("CAN R: DRS_Mode: %d \n", rawDRS_Mode);
			break;

		case 0x600: //BMS_Safety_Checker
			//Pack_Voltage (little e +)
			uint32_t rawPack_Voltage = (data[6] << 24) | (data[5] << 16)| (data[4] << 8)| (data[3]);
			//Balancing_State (little e +)
			uint8_t rawBalancing_State = (data[2]>>7) & 0x01;
			//Pack_High_Volt_Warning (little e +)
			uint8_t rawPack_High_Volt_Warning = (data[1]>>7) & 0x01;
			//Pack_Low_Volt_Warning (little e +)
			uint8_t rawPack_Low_Volt_Warning = (data[1]>>6) & 0x01;
			
			//Cell_Low_Volt_Warning (little e +)
			uint8_t rawCell_Low_Volt_Warning = (data[1]>>5) & 0x01;
			//Cell_High_Volt_Warning (little e +)
			uint8_t rawCell_High_Volt_Warning = (data[1]>>4) & 0x01;
			//Cell_High_Temp_Warning (little e +)
			uint8_t rawCell_High_Temp_Warning = (data[1]>>3) & 0x01;
			//Cell_Low_Temp_Warning (little e +)
			uint8_t rawCell_Low_Temp_Warning = (data[1]>>2) & 0x01;

			//Cell_Volt_Imbalance_Warning (little e +)
			uint8_t rawCell_Volt_Imbalance_Warning = (data[1]>>1) & 0x01;
			//Pack_High_Volt_Fault (little e +)
			uint8_t rawPack_High_Volt_Fault = (data[0]>>7) & 0x01;
			//Pack_Low_Volt_Fault (little e +)
			uint8_t rawPack_Low_Volt_Fault = (data[0]>>6) & 0x01;
			//Cell_Low_Volt_Fault (little e +)
			uint8_t rawCell_Low_Volt_Fault = (data[0]>>5) & 0x01;

			//Cell_High_Volt_Fault (little e +)
			uint8_t rawCell_High_Volt_Fault = (data[0]>>4) & 0x01;
			//Cell_High_Temp_Fault (little e +)
			uint8_t rawCell_High_Temp_Fault = (data[0]>>3) & 0x01;
			//Cell_Volt_Imbalance_Fault (little e +)
			uint8_t rawCell_Volt_Imbalance_Fault = (data[0]>>2) & 0x01;
			//Balacing_End_Fault (little e +)
			uint8_t rawBalacing_End_Fault = (data[0]>>1) & 0x01;

			float Pack_Voltage = (float)rawPack_Voltage * 0.0001f;
			
			printf("CAN R: BMS_Pack_Voltage: %.4f\n", Pack_Voltage);
			printf("CAN R: BMS_Balancing_State: %d\n", rawBalancing_State);
			printf("CAN R: BMS_Pack_High_Volt_Warning: %d\n", rawPack_High_Volt_Warning);
			printf("CAN R: BMS_Pack_Low_Volt_Warning: %d\n", rawPack_Low_Volt_Warning);
		
			printf("CAN R: BMS_Cell_Low_Volt_Warning: %d\n", rawCell_Low_Volt_Warning);
			printf("CAN R: BMS_Cell_High_Volt_Warning: %d\n", rawCell_High_Volt_Warning);
			printf("CAN R: BMS_Cell_High_Temp_Warning: %d\n", rawCell_High_Temp_Warning);
			printf("CAN R: BMS_Cell_Low_Temp_Warning: %d\n", rawCell_Low_Temp_Warning);
		
			printf("CAN R: BMS_Cell_Volt_Imbalance_Warning: %d\n", rawCell_Volt_Imbalance_Warning);
			printf("CAN R: BMS_Pack_High_Volt_Fault: %d\n", rawPack_High_Volt_Fault);
			printf("CAN R: BMS_Pack_Low_Volt_Fault: %d\n", rawPack_Low_Volt_Fault);
			printf("CAN R: BMS_Cell_Low_Volt_Fault: %d\n", rawCell_Low_Volt_Fault);
		
			printf("CAN R: BMS_Cell_High_Volt_Fault: %d\n", rawCell_High_Volt_Fault);
			printf("CAN R: BMS_Cell_High_Temp_Fault: %d\n", rawCell_High_Temp_Fault);
			printf("CAN R: BMS_Cell_Volt_Imbalance_Fault: %d\n", rawCell_Volt_Imbalance_Fault);
			printf("CAN R: BMS_Balacing_End_Fault: %d\n", rawBalacing_End_Fault);

			if (imu_crash_event == 1 && !bms_can_captured) {
				crash_record_t bms_record = {0};
				bms_record.g_force = 3.7f;
				bms_record.accel[0] = 1.2f;
				bms_record.accel[1] = -0.8f;
				bms_record.accel[2] = 0.4f;
				bms_record.bms_can_id = msg_id;
				bms_record.bms_can_dlc = 8;
				memcpy(bms_record.bms_can_data, data, 8);
				bms_record.bms_captured = true;
				save_crash_record(&bms_record);
				bms_can_captured = true;
			}

			break;

		case 0x622: //BMS_Cell_Summary
			//Lowest_Cell_Temperature (little e +)
			uint16_t rawLowest_Cell_Temperature = (data[7] << 8) | (data[6]);
			//Higest_Cell_Temperature (little e +)
			uint16_t rawHigest_Cell_Temperature = (data[5] << 8) | (data[4]);
			//Lowest_Cell_Voltage (little e +)
			uint16_t rawLowest_Cell_Voltage = (data[3] << 8) | (data[2]);
			//Highest_Cell_Voltage (little e +)
			uint16_t rawHighest_Cell_Voltage = (data[1] << 8) | (data[0]);

			
			float Lowest_Cell_Temperature = (float)rawLowest_Cell_Temperature;
			float Higest_Cell_Temperature = (float)rawHigest_Cell_Temperature;
			float Lowest_Cell_Voltage = (float)rawLowest_Cell_Voltage * 0.0001f;
			float Highest_Cell_Voltage = (float)rawHighest_Cell_Voltage * 0.0001f;
			printf("CAN R: Lowest_Cell_Temperature: %.2f C\n", Lowest_Cell_Temperature);
			printf("CAN R: Higest_Cell_Temperature: %.2f C\n", Higest_Cell_Temperature);
			printf("CAN R: Lowest_Cell_Voltage: %.4f V\n", Lowest_Cell_Voltage);
			printf("CAN R: Highest_Cell_Voltage: %.4f V\n", Highest_Cell_Voltage);
			break;

		default:
		printf("Unknown CAN ID: 0x%03" PRIX32 "\n", msg_id);
			break;
		}
}

void save_crash_record(crash_record_t *record) {
    nvs_handle_t nvs;
    if (nvs_open("crash_log", NVS_READWRITE, &nvs) != ESP_OK) return;

    nvs_set_u8(nvs, "crash_flag", 1);
    nvs_set_blob(nvs, "crash_record", record, sizeof(crash_record_t));
    nvs_commit(nvs);
    nvs_close(nvs);
    printf("Crash record saved.\n");
}

void read_crash_record() {
    nvs_handle_t nvs;
    if (nvs_open("crash_log", NVS_READONLY, &nvs) != ESP_OK) return;

    uint8_t crash_flag = 0;
    if (nvs_get_u8(nvs, "crash_flag", &crash_flag) != ESP_OK || crash_flag != 1) {
        nvs_close(nvs);
        printf("No crash record found.\n");
        return;
    }

    crash_record_t record;
    size_t size = sizeof(record);
    if (nvs_get_blob(nvs, "crash_record", &record, &size) == ESP_OK) {
        printf("=== CRASH DETECTED ===\n");
        printf("G-Force: %.2f g\n", record.g_force);
        printf("Accel: [%.2f, %.2f, %.2f]\n", record.accel[0], record.accel[1], record.accel[2]);
        printf("CAN Msg ID: 0x%lX, DLC: %d\n", record.bms_can_id, record.bms_can_dlc);
        printf("Data: ");
        for (int i = 0; i < record.bms_can_dlc; i++)
            printf("%02X ", record.bms_can_data[i]);
        printf("\n");
    }
    nvs_close(nvs);
}

void erase_crash_record() {
    nvs_handle_t nvs;
    if (nvs_open("crash_log", NVS_READWRITE, &nvs) != ESP_OK) return;

    nvs_erase_key(nvs, "crash_flag");
    nvs_erase_key(nvs, "crash_record");
    nvs_commit(nvs);
    nvs_close(nvs);
    //vcu_can_captured = false;
	bms_can_captured = false;
    printf("Crash record erased.\n");
}

void handle_crash_event() {
    if (imu_crash_event == 2) read_crash_record();
    else if (imu_crash_event == 3) erase_crash_record();
}
