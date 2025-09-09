/*
 * MPU6050_with_Kalman.h
 *
 *  Created on: Aug 23, 2025
 *      Author: Kauê Lucas
 */

#ifndef INC_MPU6050_WITH_KALMAN_H_
#define INC_MPU6050_WITH_KALMAN_H_

#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>
#include "main.h"

// Escalas e constantes
#define ACCEL_SCALE 16384.0f   // LSB/g (±2g)
#define GYRO_SCALE  131.0f     // LSB/(°/s) (±250°/s)
#define GRAVITY     9.80665f
#define RAD2DEG     57.29577951308232f

// Endereços/registradores MPU6050
#define MPU6050_ADDR       0xD0        // 0x68 << 1
#define WHO_AM_I_REG       0x75
#define PWR_MGMT_1_REG     0x6B
#define ACCEL_XOUT_H_REG   0x3B
#define GYRO_XOUT_H_REG    0x43

// Kalman 2x2 para ângulo e bias
typedef struct {
  float angle;      // x[0] = ângulo (graus)
  float bias;       // x[1] = bias do giroscópio (graus/s)
  float P[2][2];    // covariância
  float Q_angle;    // ruído de processo (ângulo)
  float Q_bias;     // ruído de processo (bias)
  float R_measure;  // ruído de medição (acelerômetro)
} Kalman_t;

extern I2C_HandleTypeDef hi2c1;

void MPU6050_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Kalman_Init(Kalman_t *kf, float Q_angle, float Q_bias, float R_measure, float angle0);
float Kalman_Update(Kalman_t *kf, float angle_meas, float gyro_rate, float dt);
void MPU6050_ReadRaw(void);
void Convert_To_Physical(void);
void Gyro_Calibrate(uint16_t samples, uint16_t delay_ms);
void Compute_Accel_Angles(float *roll_deg, float *pitch_deg);

#endif /* INC_MPU6050_WITH_KALMAN_H_ */
