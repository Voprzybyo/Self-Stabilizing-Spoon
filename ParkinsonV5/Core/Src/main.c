/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"

#define SENSITIVITY 5                             // Used for make servo movement more smoothly
#define ACC_SENSITIVITY_SCALE_FACTOR (float)16384 // Sensitivity scale factor for accelerometer (MPU6050 datasheet)
#define GYR_SENSITIVITY_SCALE_FACTOR (float)131   // Sensitivity scale factor for accelerometer (MPU6050 datasheet)
#define ERROR_ACCURATE 300

#ifndef M_PI
#define M_PI (float)3.14159265
#endif

// Declaration of accelerometer and gyroscope parameters
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

int main(void)
{
  MPU_ConfigTypeDef myMpuConfig;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();

  volatile int rollCounter = 0;
  volatile int pitchCounter = 0;

  volatile int previousRoll = 1400;
  volatile int previousPitch = 1900;

  MPU6050_Init(&hi2c1); // Initialize the MPU6050 module and I2C

  // Configure accelerometer and gyroscope parameters
  myMpuConfig.Accel_Full_Scale = AFS_SEL_2g; // Accelerometer Full Scale Range (1g = 9.81m/s2)
  myMpuConfig.ClockSource = Internal_8MHz;
  myMpuConfig.CONFIG_DLPF = DLPF_5_Hz;      // Digital Low Pass Filter
  myMpuConfig.Gyro_Full_Scale = FS_SEL_250; // Gyroscope Full Scale Range
  myMpuConfig.Sleep_Mode_Bit = 0;           // 1: Sleep mode, 0: Normal mode
  MPU6050_Config(&myMpuConfig);

  // Initialize channel 1-2 (Timer1) used for generate PWM signal
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // Set servomechanisms on basic position
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1400);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1900);

  Calculate_error(myMpuConfig); // Get average error from MPU6050

  HAL_Delay(4000);

  while (1)
  {
    // Get accelerometer scaled data and divide by sensitivity scale factor (from MPU6050 datasheet)
    MPU6050_Get_Accel_Scale(&myAccelScaled);
    AccX = myAccelScaled.x / ACC_SENSITIVITY_SCALE_FACTOR;
    AccY = myAccelScaled.y / ACC_SENSITIVITY_SCALE_FACTOR;
    AccZ = myAccelScaled.z / ACC_SENSITIVITY_SCALE_FACTOR;

    // Calculate roll and pitch values based on accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / M_PI) - 0.89f;
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / M_PI) + 0.146f;

    // Get gyroscope scaled data
    previousTime = currentTime;
    currentTime = HAL_GetTick();                       // Get time in ms
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

    // Get gyroscope scaled data and divide by sensitivity scale factor (from MPU6050 datasheet)
    MPU6050_Get_Gyro_Scale(&myGyroScaled);
    GyroX = myGyroScaled.x / GYR_SENSITIVITY_SCALE_FACTOR;
    GyroY = myGyroScaled.y / GYR_SENSITIVITY_SCALE_FACTOR;
    GyroZ = myGyroScaled.z / GYR_SENSITIVITY_SCALE_FACTOR;

    // Calculate output (modification with calculated error values)
    GyroX = GyroX - 0.033f;
    GyroY = GyroY - 0.028f;
    GyroZ = GyroZ + 0.004f;

    gyroAngleX = GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = GyroY * elapsedTime; // deg/s * s = deg

    // Calculate yaw, pitch, roll values and apply complementary filter (combination of accelerometer and gyroscope angle values)
    yaw = GyroZ * elapsedTime;
    roll = (0.96f * gyroAngleX + 0.04f * accAngleX) * 180 / M_PI;
    pitch = (0.96f * gyroAngleY + 0.04f * accAngleY) * 180 / M_PI;

    // Map roll and pitch values accordingly to servomechanisms edge positions
    rollCounter = Map((int)roll, -180, 180, 450, 2350);
    pitchCounter = Map((int)pitch, -180, 180, 700, 2600);

    // Make servo movement more smoothly
    if (rollCounter > previousRoll)
      previousRoll += SENSITIVITY;
    else
      previousRoll -= SENSITIVITY;

    if (pitchCounter > previousPitch)
      previousPitch += SENSITIVITY;
    else
      previousPitch -= SENSITIVITY;

    // Set servo output
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, previousRoll);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, previousPitch);

    HAL_Delay(5);
  }
}

float Map(float val, float I_Min, float I_Max, float O_Min, float O_Max)
{
  return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}

void Calculate_error(MPU_ConfigTypeDef myMpuConfig)
{
  int c = 0;

  // Read accelerometer value
  while (c < ERROR_ACCURATE)
  {
    MPU6050_Get_Accel_Scale(&myAccelScaled);
    AccX = myAccelScaled.x / ACC_SENSITIVITY_SCALE_FACTOR;
    AccY = myAccelScaled.y / ACC_SENSITIVITY_SCALE_FACTOR;
    AccZ = myAccelScaled.z / ACC_SENSITIVITY_SCALE_FACTOR;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / M_PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / M_PI));
    c++;
  }

  //Divide the sum by number of iteration to get the error value
  AccErrorX = AccErrorX / ERROR_ACCURATE;
  AccErrorY = AccErrorY / ERROR_ACCURATE;
  c = 0;

  // Read gyro value
  while (c < ERROR_ACCURATE)
  {
    MPU6050_Get_Gyro_Scale(&myGyroScaled);
    GyroX = myGyroScaled.x;
    GyroY = myGyroScaled.y;
    GyroZ = myGyroScaled.z;

    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / GYR_SENSITIVITY_SCALE_FACTOR);
    GyroErrorY = GyroErrorY + (GyroY / GYR_SENSITIVITY_SCALE_FACTOR);
    GyroErrorZ = GyroErrorZ + (GyroZ / GYR_SENSITIVITY_SCALE_FACTOR);
    c++;
  }

  //Divide the sum by number of iteration to get the error value
  GyroErrorX = GyroErrorX / ERROR_ACCURATE;
  GyroErrorY = GyroErrorY / ERROR_ACCURATE;
  GyroErrorZ = GyroErrorZ / ERROR_ACCURATE;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
