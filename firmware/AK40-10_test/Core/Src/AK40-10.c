#include "AK40-10.h"
#include "main.h"

// Pull the CAN peripheral handle from main.c
extern CAN_HandleTypeDef hcan1;

// Private helper function (doesn't need to be in the header)
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}

void AK40_Wake(void) {
    CAN_TxHeaderTypeDef WakeHeader = {0};
    WakeHeader.TransmitGlobalTime = DISABLE;
    WakeHeader.StdId = 0x01;
    WakeHeader.ExtId = 0x01;
    WakeHeader.IDE = CAN_ID_STD;
    WakeHeader.RTR = CAN_RTR_DATA;
    WakeHeader.DLC = 8;
    
    uint8_t wake_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint32_t TxMailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &WakeHeader, wake_data, &TxMailbox);

      if (status != HAL_OK) {
        for (int i = 0; i < 10; i++) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(50);  // fast blink = TX failed
        }
    } else {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(500);     // slow blink = TX queued
    }
}

void AK40_SetVelocity(float target_rad_sec) {
    // Corrected bounds for the AK40-10
    uint16_t p_int  = float_to_uint(0.0, -12.5f, 12.5f, 16);
    uint16_t v_int  = float_to_uint(target_rad_sec, -45.5f, 45.5f, 12);
    uint16_t kp_int = float_to_uint(0.0, 0.0f, 500.0f, 12); 
    uint16_t kd_int = float_to_uint(1.0, 0.0f, 5.0f, 12);   
    uint16_t t_int  = float_to_uint(0.0, -5.0f, 5.0f, 12);

    uint8_t tx_data[8];
    tx_data[0] = p_int >> 8;
    tx_data[1] = p_int & 0xFF;
    tx_data[2] = v_int >> 4;
    tx_data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    tx_data[4] = kp_int & 0xFF;
    tx_data[5] = kd_int >> 4;
    tx_data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    tx_data[7] = t_int & 0xFF;

    CAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.StdId = 0x01; // Assuming motor ID is 1
    TxHeader.ExtId = 0x01;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    
    uint32_t TxMailbox;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);
}

void AK40_ReadResponse(float *pos, float *vel, float *current, uint8_t *fault_code, uint8_t *motor_id) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rx_data);

        // ONLY parse if it's an Extended Frame and matches the Servo Telemetry ID (0x29) 
        // Extended ID format for telemetry from Motor 1 is: 0x01 | (0x29 << 8) = 0x2901
        if (RxHeader.IDE == CAN_ID_EXT) {
            
            *motor_id = RxHeader.ExtId & 0xFF; // Extract motor ID from the lower byte of the Extended ID
            
            // Reconstruct the 16-bit integers from the Servo telemetry payload [cite: 3, 954, 982, 983, 984]
            int16_t p_int = (rx_data[0] << 8) | rx_data[1];
            int16_t v_int = (rx_data[2] << 8) | rx_data[3];
            int16_t c_int = (rx_data[4] << 8) | rx_data[5];

            // Apply the Servo mode scaling factors [cite: 3, 985, 986, 987]
            *fault_code = rx_data[7]; // Motor fault code is in the last byte of the telemetry stream
            *pos     = (float)(p_int * 0.1f);   // Degrees
            *vel     = (float)(v_int * 10.0f);  // ERPM
            *current = (float)(c_int * 0.01f);  // Amps
        }
    }
}

void AK40_SetServoVelocity(int32_t erpm) {
    CAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t tx_data[4];
    uint32_t TxMailbox;

    tx_data[0] = (erpm >> 24) & 0xFF;
    tx_data[1] = (erpm >> 16) & 0xFF;
    tx_data[2] = (erpm >> 8) & 0xFF;
    tx_data[3] = erpm & 0xFF;

    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.ExtId = 0x00 | (3 << 8); 
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 4; 
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Check if the message was successfully added to a mailbox
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox) != HAL_OK) {
        // If this executes, the CAN bus is failing to transmit (No ACK)
        // Turn the LED ON solid to indicate a TX failure
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    } else {
        // Toggle the LED so it flickers rapidly when successfully transmitting
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}

void AK40_SetDutyCycle(float duty) {
    // Prevent sending commands before we know the motor's ID
    CAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t tx_data[4];
    uint32_t TxMailbox;

    // The protocol requires scaling the duty cycle by 100,000 (e.g. 0.05 = 5000) [cite: 3, 806]
    int32_t send_duty = (int32_t)(duty * 100000.0f);

    tx_data[0] = (send_duty >> 24) & 0xFF;
    tx_data[1] = (send_duty >> 16) & 0xFF;
    tx_data[2] = (send_duty >> 8) & 0xFF;
    tx_data[3] = send_duty & 0xFF;

    TxHeader.IDE = CAN_ID_EXT;
    
    // Command ID for Duty Cycle is 0 [cite: 3, 750, 807, 808]
    TxHeader.ExtId = 69 | (0 << 8); 
    
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 4; 
    TxHeader.TransmitGlobalTime = DISABLE;
    
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox) != HAL_OK) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}