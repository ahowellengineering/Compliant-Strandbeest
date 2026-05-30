#ifndef AK40_10_H
#define AK40_10_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Function Prototypes
void AK40_Wake(void);
void AK40_SetVelocity(float target_rad_sec);
void AK40_ReadResponse(float* p, float* v, float* I, uint8_t* fault_code, uint8_t* motor_id);
void AK40_SetServoVelocity(int32_t erpm);
void AK40_SetDutyCycle(float duty);

#endif /* AK40_10_H */