/*
 * mpu9255.h
 *
 *  Created on: Dec 14, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit   
 */


#include <math.h>
#include "mpu9255.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU9255_ADDR 0x68<<1
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define CONFIG 0x1A


//magnetometer
#define MAG_ADDR 0x0C
#define MAG_XOUT_L 0x03
#define MAG_ID 0x00
#define INT_PIN_CFG 0x37
#define CNTL 0x0A
#define CNTL2 0x0B
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12

uint8_t checkStat;
int isWorking;

const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU9255_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    checkStat = check;
    isWorking = 1;

    Data = 0x03;
    HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, CONFIG, 1, &Data, 1, i2c_timeout);

	// power management register 0X6B we should write all 0's to wake the sensor up
	Data = 0;
	HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	Data = 0x07;
	HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

	// Set accelerometer configuration in ACCEL_CONFIG Register
	// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
	Data = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

	// Set Gyroscopic configuration in GYRO_CONFIG Register
	// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
	Data = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

	Data = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, INT_PIN_CFG, 1, &Data, 1, i2c_timeout);

	Data = 0x16;
	HAL_I2C_Mem_Write(I2Cx, MPU9255_ADDR, CNTL, 1, &Data, 1, i2c_timeout);

	return 0;
}

void MPU9255_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU9255_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU9255_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU9255_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

// Read from magnetometer
void MPU9255_Read_Mag(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from MAG_XOUT_L register    v-- Request 8 bytes of data, not 6!

    HAL_I2C_Mem_Read(I2Cx, MAG_ADDR, MAG_XOUT_L, 1, Rec_Data, 8, i2c_timeout);

    DataStruct->Mag_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Mag_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Mag_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);


    DataStruct->Gx = DataStruct->Gyro_X_RAW;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW;
}

void MPU9255_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU9255_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU9255_Read_All(I2C_HandleTypeDef *I2Cx, MPU9255_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(I2Cx, MPU9255_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

    // Read from magnetometer
	HAL_I2C_Mem_Read(I2Cx, MAG_ADDR, MAG_XOUT_L, 1, Rec_Data, 8, i2c_timeout);

	DataStruct->Mag_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Mag_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Mag_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);


	DataStruct->Mx = DataStruct->Mag_X_RAW;
	DataStruct->My = DataStruct->Mag_Y_RAW;
	DataStruct->Mz = DataStruct->Mag_Z_RAW;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
