#include "can_lora.h"
#include "main.h"    
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/twai.h"
#include "telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rgb_ledc_controller.h>
#include "crash_imu.h"

static const char *TAG = CAN_LORA_TAG; 

lora_command_t lora_cmd = {.key = "", .value = 0, .new_command = false};
#define CAN_ID 0x7FF // Same CAN ID for all mode switches

void canTask(void* arg){
 while (1) {
        switch (currentState) {
            case SENDING_STATE:
                // In SENDING_STATE, CAN should RECEIVE
                canReceive();  
                break;

            case RECEIVING_STATE:
                // In RECEIVING_STATE, CAN should SEND
                handle_lora_can_command();
                break;

            case SLEEP_STATE:
                break;

            default:
                
                break;
        }
    }

}

// Call periodically or inside CAN task
static int count = 0;
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

void start_can_task(void){
    xTaskCreatePinnedToCore(
        canTask,                      // Task function
        "canTask",                    // Name of task
        4096,  // Stack size
        NULL,                          // Task parameters
        1,                             // Priority
        NULL,                          // Task handle
        0                              // Core ID (0 = first core, 1 = second core)
    );
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
        ESP_LOGI("CAN:", "Sent %s command on CAN ID 0x%03" PRIX32 " value=%d",
                 lora_cmd.key, msg.identifier, msg.data[0]);
        rgb_led_set_color(&led1, YELLOW);
    } else {
        ESP_LOGE("CAN", "CAN transmit failed: %s", esp_err_to_name(ret));
    }

    // Clear the flag
    lora_cmd.new_command = false;
}

