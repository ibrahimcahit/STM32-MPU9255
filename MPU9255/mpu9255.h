/*
 * mpu9255.h
 *
 *  Created on: Dec 14, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit   
 */

#ifndef INC_9255_H_
#define INC_9255_H_

#endif /* INC_9255_H_ */

#include <stdint.h>
#include "i2c.h"

#include <stdint.h>
#include "i2c.h"

// MPU9255 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
	
	int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;
    double Mx;
    double My;
    double Mz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU9255_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2Cx);

void MPU9255_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct);

void MPU9255_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct);

void MPU9255_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct);

void MPU9255_Read_All(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
