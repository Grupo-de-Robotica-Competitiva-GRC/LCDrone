/*
 * MPU6050_with_Kalman.c
 *
 *  Created on: Aug 23, 2025
 *      Author: Kauê Lucas
 */
#include "MPU6050_with_Kalman.h"

// Buffer para armazenar os dados
int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
uint8_t accel_data[6];
uint8_t gyro_data[6];

// Variáveis em float (convertidas)
float ax_ms2, ay_ms2, az_ms2;   // aceleração em m/s²
float gx_dps, gy_dps, gz_dps;   // giroscópio em °/s

// Bias do gyro (calibração simples no boot)
float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;

void MPU6050_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	// Acorda MPU6050
	  uint8_t wake = 0x00;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &wake, 1, HAL_MAX_DELAY);

	  // Teste WHO_AM_I
	  uint8_t who = 0;
	  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who, 1, 1000);
	  if (ret == HAL_OK && who == 0x68) {
	    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	  } else {
	    for (int i = 1; i < 5; i++) { HAL_GPIO_TogglePin(GPIOx, GPIO_Pin); HAL_Delay(1000); }
	  }
}

void Kalman_Init(Kalman_t *kf, float Q_angle, float Q_bias, float R_measure, float angle0) {
  kf->angle = angle0;
  kf->bias  = 0.0f;
  kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
  kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
  kf->Q_angle = Q_angle;
  kf->Q_bias  = Q_bias;
  kf->R_measure = R_measure;
}

float Kalman_Update(Kalman_t *kf, float angle_meas, float gyro_rate, float dt) {
  // 1) Predição
  float rate = gyro_rate - kf->bias;     // remove bias do gyro
  kf->angle += rate * dt;

  // Jacobiana F = [[1, -dt],[0,1]]
  // Atualiza P = F P F^T + Q
  float P00 = kf->P[0][0], P01 = kf->P[0][1];
  float P10 = kf->P[1][0], P11 = kf->P[1][1];

  kf->P[0][0] = P00 + dt * ( -P01 - P10 + dt * P11 ) + kf->Q_angle;
  kf->P[0][1] = P01 + dt * ( -P11 )                 ;
  kf->P[1][0] = P10 + dt * ( -P11 )                 ;
  kf->P[1][1] = P11 + kf->Q_bias;

  // 2) Atualização (medição z = ângulo do acelerômetro)
  float S = kf->P[0][0] + kf->R_measure; // inov. cov
  float K0 = kf->P[0][0] / S;
  float K1 = kf->P[1][0] / S;

  float y = angle_meas - kf->angle;      // inov. (resíduo)

  // Atualiza estado
  kf->angle += K0 * y;
  kf->bias  += K1 * y;

  // Atualiza covariância: P = (I - K H) P
  float P00_new = (1.0f - K0) * kf->P[0][0];
  float P01_new = (1.0f - K0) * kf->P[0][1];
  float P10_new =    - K1     * kf->P[0][0] + kf->P[1][0];
  float P11_new =    - K1     * kf->P[0][1] + kf->P[1][1];

  kf->P[0][0] = P00_new; kf->P[0][1] = P01_new;
  kf->P[1][0] = P10_new; kf->P[1][1] = P11_new;

  return kf->angle;
}

void MPU6050_ReadRaw(void) {
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, accel_data, 6, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG,  I2C_MEMADD_SIZE_8BIT, gyro_data,  6, HAL_MAX_DELAY);

  accel_x = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  accel_y = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  accel_z = (int16_t)((accel_data[4] << 8) | accel_data[5]);

  // CORREÇÃO: usar gyro_data para os 6 bytes do gyro
  gyro_x  = (int16_t)((gyro_data[0]  << 8) | gyro_data[1]);
  gyro_y  = (int16_t)((gyro_data[2]  << 8) | gyro_data[3]);
  gyro_z  = (int16_t)((gyro_data[4]  << 8) | gyro_data[5]);
}

void Convert_To_Physical(void) {
  // Em m/s²
  ax_ms2 = ((float)accel_x / ACCEL_SCALE) * GRAVITY;
  ay_ms2 = ((float)accel_y / ACCEL_SCALE) * GRAVITY;
  az_ms2 = ((float)accel_z / ACCEL_SCALE) * GRAVITY;

  // Em °/s (já compensando bias)
  gx_dps = ((float)gyro_x / GYRO_SCALE) - gx_bias;
  gy_dps = ((float)gyro_y / GYRO_SCALE) - gy_bias;
  gz_dps = ((float)gyro_z / GYRO_SCALE) - gz_bias;
}

void Gyro_Calibrate(uint16_t samples, uint16_t delay_ms) {
  int32_t sx = 0, sy = 0, sz = 0;
  for (uint16_t i = 0; i < samples; i++) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, gyro_data, 6, HAL_MAX_DELAY);
    int16_t gx = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    int16_t gy = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    int16_t gz = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
    sx += gx; sy += gy; sz += gz;
    HAL_Delay(delay_ms);
  }
  // Converte média para °/s e guarda como bias
  gx_bias = ((float)(sx / (int32_t)samples)) / GYRO_SCALE;
  gy_bias = ((float)(sy / (int32_t)samples)) / GYRO_SCALE;
  gz_bias = ((float)(sz / (int32_t)samples)) / GYRO_SCALE;
}

void Compute_Accel_Angles(float *roll_deg, float *pitch_deg) {
  // Roll = rotação em X (olhando para frente, o balanço “para os lados”)
  // Pitch = rotação em Y (nariz pra cima/baixo)
  // Fórmulas clássicas:
  // roll  = atan2(ay, az)
  // pitch = atan2(-ax, sqrt(ay^2 + az^2))
  *roll_deg  = atan2f(ay_ms2, az_ms2) * RAD2DEG;
  float denom = sqrtf(ay_ms2*ay_ms2 + az_ms2*az_ms2);
  if (denom < 1e-6f) denom = 1e-6f;
  *pitch_deg = atan2f(-ax_ms2, denom) * RAD2DEG;
}
