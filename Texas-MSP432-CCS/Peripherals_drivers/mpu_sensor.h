/*
 * Drivers_Control.h
 *
 *  Created on: 21 de mar de 2020
 *      Author: davia
 * Biblioteca para auxiliar o uso da SDK com alguns exemplos
 */
#ifndef MPU_SENSOR_H_
#define MPU_SENSOR_H_

//*****************************************************************************
//
//! \addtogroup gpio_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************************************************************************************************
// Includes necessï¿½rios
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h> // usar essa definiï¿½ï¿½o para o CodeComposer
#include "Drivers_Control.h"
#include <math.h>

#define g 9.8f
#define ACC_RESOLUTION (1.0f/16384.0f)
#define GYRO_RESOLUTION 1.3323e-04f
#define SIZE_MAGNETOMETER_CALIBRATION 2001
typedef struct
{
    uint8_t identification;
    /* Sensor Address*/
    uint8_t address;
    /* Sensor I2C in use */
    uint8_t I2C;
    /* Axis Coordinates for sensor*/
    // arc tangent (axis[0]/axis[1])
    int8_t Axis_Magnetometer[2];
    int8_t Axis_Gyroscope;
    int8_t Axis_Accelerometer[2];

    /* Acceleration values from Accelerometer*/
    int16_t ax;
    int16_t ay;
    int16_t az;
    /* Gyroscope extreme Values from Giroscope */
    int16_t max_ax;
    int16_t max_ay;
    int16_t max_az;
    int16_t min_ax;
    int16_t min_ay;
    int16_t min_az;

    /* Temperature value */
    int16_t temp;
    /* Gyroscope values from gyroscope in Euler Angle */
    int16_t gx;
    int16_t gy;
    int16_t gz;
    /* Gyroscope extreme Values from Giroscope */
    int16_t max_gx;
    int16_t max_gy;
    int16_t max_gz;
    int16_t min_gx;
    int16_t min_gy;
    int16_t min_gz;

    /* Magnet Field from Magnetometer */
    int16_t mx;
    int16_t my;
    int16_t mz;
    /* Magnet Field Extreme Values from Magnetometer */
    int16_t max_mx;
    int16_t max_my;
    int16_t max_mz;
    int16_t min_mx;
    int16_t min_my;
    int16_t min_mz;
    /* Accelerometer Calibration Variables*/
    float acc_offset_x;
    float acc_offset_y;
    float acc_offset_z;
    /* Magnetometer Calibration Variables*/
    float mag_offset_x;
    float mag_offset_y;
    float mag_offset_z;

    /* Giroscope Calibration Variables*/
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;

    /*defasagem entre o zero do magnetômetro e o zero do motor*/
    float calibration_offset_mag;
    /* Input variables for the Kalman and LuebergerFilter*/
    float acc_Signal;
    float mag_Signal;
    float gyro_Signal;

    /* Input variables for the Kalman and LuebergerFilter*/
    float acc_Signal_Filtered;
    float mag_Signal_Filtered;
    float gyro_Signal_Filtered;

    /* Final Angle Update by Sensor Fusion*/
    float ang_updated;
    float ang_Kalman;
    float ang_Luenberger;

} dr_mpu_data_t;
// variables to compensate for communication errors with the sensor
int16_t erro_watch[5];
uint32_t diagnostic_erro[5];
// Variable to store Data
uint8_t leitura[14];

// Update values of gyroscope and accelerometer in MPU6050
extern int DR_mpu6050_atualizar(dr_mpu_data_t *sensor);
/*  Initialize MPU6050
 *  1) Set accelerometer resolution in +-2g
 *  2) Set low pass filter
 *  3) Set interruption data ready in MPU
 *  4) Wake-Up device
 * */
extern int DR_mpu6050_init(dr_mpu_data_t *sensor);
/* Turn on and reset the sensor power with delay in tempo_ms
 * set in-GPIO 6.1
 * */
extern void DR_mpu_ligar(uint16_t tempo_ms);
extern int DR_mpu_read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                       uint8_t *data);
extern int DR_mpu_readraw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                          uint8_t byteCount, uint8_t *data);
extern void DR_gyroscope_calibrate(dr_mpu_data_t *sensor,
                                   uint16_t calibration_size);
extern int8_t DR_magnetometer_calibrate(dr_mpu_data_t *sensor);
extern int DR_mpu9250_init(dr_mpu_data_t *sensor);
extern int DR_mpu9250_atualizar(dr_mpu_data_t *sensor);
extern void Dr_mag_signal_update(dr_mpu_data_t *sensor);
extern void Dr_acc_signal_update(dr_mpu_data_t *sensor);
extern void Dr_gyro_signal_update(dr_mpu_data_t *sensor);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CONTROL_H_ */


- identification;      :uint8_t
- address;             :uint8_t
- i2c;                 :uint8_t
- axisMagnetometer[2]; :int8_t
- axisGiroscope;       :int8_t
- axisAccelerometer[2];:int8_t
- ax;                  :int16_t
- ay;                  :int16_t
- az;                  :int16_t
- maxAx;               :int16_t
- maxAy;               :int16_t
- maxAz;               :int16_t
- minAx;               :int16_t
- minAy;               :int16_t
- minAz;               :int16_t
- temp;                :int16_t
- gx;                  :int16_t
- gy;                  :int16_t
- gz;                  :int16_t
- maxGx;               :int16_t
- maxGy;               :int16_t
- maxGz;               :int16_t
- minGx;               :int16_t
- minGy;               :int16_t
- minGz;               :int16_t
- mx;                  :int16_t
- my;                  :int16_t
- mz;                  :int16_t
- maxMx;               :int16_t
- maxMy;               :int16_t
- maxMz;               :int16_t
- minMx;               :int16_t
- minMy;               :int16_t
- minMz;               :int16_t
- accOffsetX;          :float
- accOffsetY;          :float
- accOffsetZ;          :float
- magOffsetX;          :float
- magOffsetY;          :float
- magOffsetZ;          :float
- girOffsetX;          :float
- girOffsetY;          :float
- girOffsetZ;          :float
- calibrationOffsetMag;:float
- accSignal;           :float
- magSignal;           :float
- gyrSignal;           :float
- accSignalFiltered;   :float
- magSignalFiltered;   :float
- girSignalFiltered;   :float
- angUpdated;          :float
- angKalman;           :float
- angLuenberger;       :float


+ mpu6050Atualizar(dr_mpu_data_t* sensor)                    :int
+ mpu6050Init(dr_mpu_data_t * sensor);                      :int
+ mpuLigar(uint16_t tempo_ms);                              :void
+ mpuRead(uint8_t n_I2C, uint8_t slaveAddr, 
         uint8_t memAddr, uint8_t * data)                   :int 
+ mpuReadraw(uint8_t n_I2C, uint8_t slaveAddr,               :int
            uint8_t memAddr, uint8_t byteCount, uint8_t * data)
            giroscopeCalibrate(dr_mpu_data_t * sensor,       :void
            uint16_t calibration_size);
+ magnetometerCalibrate(dr_mpu_data_t * sensor);            :int8_t
+ mpu9250Init(dr_mpu_data_t * sensor);                      :int
+ mpu9250Atualizar(dr_mpu_data_t * sensor);                 :int
+ magSignalUpdate(dr_mpu_data_t * sensor);                  :void
+ accSignalUpdate(dr_mpu_data_t * sensor);                  :void
+ girSignalUpdate(dr_mpu_data_t * sensor);                  :void