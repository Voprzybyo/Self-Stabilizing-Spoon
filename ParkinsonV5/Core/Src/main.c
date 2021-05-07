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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>

#include "TJ_MPU6050.h"
#include "Config.h"

#define SENSITIVITY 5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//volatile uint32_t Duty = 0;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

float map(float val, float I_Min, float I_Max, float O_Min, float O_Max);
void calculate_IMU_error(MPU_ConfigTypeDef myMpuConfig);

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

  MPU6050_Init(&hi2c1); //  Initialise the MPU6050 module and I2C

  //Configure Accel and Gyro parameters
  myMpuConfig.Accel_Full_Scale = AFS_SEL_2g;
  myMpuConfig.ClockSource = Internal_8MHz;
  myMpuConfig.CONFIG_DLPF = DLPF_5_Hz;
  myMpuConfig.Gyro_Full_Scale = FS_SEL_250;
  myMpuConfig.Sleep_Mode_Bit = 0; //1: sleep mode, 0: normal mode
  MPU6050_Config(&myMpuConfig);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1400);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1900);

  calculate_IMU_error(myMpuConfig);
  HAL_Delay(4000);

  while (1)
  {
    MPU6050_Get_Accel_Scale(&myAccelScaled);
    AccX = myAccelScaled.x / 16384.0f;
    AccY = myAccelScaled.y / 16384.0f;
    AccZ = myAccelScaled.z / 16384.0f;

    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / (float) M_PI) - 0.89f;
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / (float) M_PI) + 0.146f;

    previousTime = currentTime;
    currentTime = HAL_GetTick();
    elapsedTime = (currentTime - previousTime) / 1000;

    MPU6050_Get_Gyro_Scale(&myGyroScaled);
    GyroX = myGyroScaled.x / 131.0f;
    GyroY = myGyroScaled.y / 131.0f;
    GyroZ = myGyroScaled.z / 131.0f;

    GyroX = GyroX - 0.033f; // GyroErrorX ~(-0.56)
    GyroY = GyroY - 0.028f; // GyroErrorY ~(2)
    GyroZ = GyroZ + 0.004f;

    gyroAngleX = GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = GyroY * elapsedTime;

    yaw = GyroZ * elapsedTime;
    roll = (0.96f * gyroAngleX + 0.04f * accAngleX) * 180 / (float)M_PI;
    pitch = (0.96f * gyroAngleY + 0.04f * accAngleY) * 180 / (float)M_PI;

    rollCounter = map((int)roll, -180, 180, 450, 2350);
    pitchCounter = map((int)pitch, -180, 180, 700, 2600);

		if(rollCounter > previousRoll )
			previousRoll += SENSITIVITY;
		else
			previousRoll -= SENSITIVITY;
	
		if(pitchCounter > previousPitch )
			previousPitch += SENSITIVITY;
		else
			previousPitch -= SENSITIVITY;
		
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, previousRoll);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, previousPitch);

    HAL_Delay(5);
  }
}

float map(float val, float I_Min, float I_Max, float O_Min, float O_Max)
{
  return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}

void calculate_IMU_error(MPU_ConfigTypeDef myMpuConfig)
{
  while (c < 200)
  {
    MPU6050_Get_Accel_Scale(&myAccelScaled);
    AccX = myAccelScaled.x / 16384.0f;
    AccY = myAccelScaled.y / 16384.0f;
    AccZ = myAccelScaled.z / 16384.0f;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / M_PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / M_PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200)
  {
    MPU6050_Get_Gyro_Scale(&myGyroScaled);
    GyroX = myGyroScaled.x;
    GyroY = myGyroScaled.y;
    GyroZ = myGyroScaled.z;
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0f);
    GyroErrorY = GyroErrorY + (GyroY / 131.0f);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0f);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
