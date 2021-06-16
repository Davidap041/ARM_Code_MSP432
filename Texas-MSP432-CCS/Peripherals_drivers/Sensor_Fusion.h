#ifndef _Sensor_Fusion_h_
#define _Sensor_Fusion_h_
#include <math.h>
#include <arm_math.h>
#include <arm_const_structs.h>
/***********************************************************
 * Luenberger Filter for Sensor Fusion
 ***********************************************************
 */typedef struct Luenberger_Struct
{
    float angle; // The angle calculated by the luenberger filter - part of the 2x1 state vector
    float angle_prev;

    float bias; // The gyro bias calculated by the luenberger filter - part of the 2x1 state vector
    float bias_prev;

    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    float residue;

    float pole1;
    float pole2;

    float L1_;
    float L2_; // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds


} Luenberger_data;
float getAngle_Luenberger(Luenberger_data *luenberger, float newAngle,
                          float newRate, float dt);
void setPoles_Luenberger(Luenberger_data *luenberger, float dt);
/***********************************************************
 * Kalman Filter for Sensor Fusion
 ***********************************************************
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
typedef struct Kalman_Struct
{
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
} Kalman_data;
float getAngle_Kalman(Kalman_data *kalman, float newAngle, float newRate,
                      float dt);

/********************************
 * FFT and SSF for Sensor Fusion
 * *****************************
 * */
//  Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192.
//#define BUFFER_SIGNAL_LENGTH 64
#define BUFFER_SIGNAL_LENGTH 64
#define FFT_SIZE 64
float32_t buffered_signal[BUFFER_SIGNAL_LENGTH];
float32_t fft_data_input[BUFFER_SIGNAL_LENGTH * 2];
float32_t fft_data_output[FFT_SIZE];
float32_t fft_filtered[FFT_SIZE];
float32_t eq_sigmoid[FFT_SIZE];
float32_t ifft_data_output[FFT_SIZE];


float calculateSSF_in_realTime(float32_t data_real_time, int sigmoide_a,
                               int sigmoide_c);
#endif

float32_t buffered_signal[BUFFER_SIGNAL_LENGTH];
float32_t fft_data_input[BUFFER_SIGNAL_LENGTH * 2];
float32_t fft_data_output[FFT_SIZE];
float32_t fft_filtered[FFT_SIZE];
float32_t eq_sigmoid[FFT_SIZE];
float32_t ifft_data_output[FFT_SIZE];


float calculateSSF_in_realTime(float32_t data_real_time, int sigmoide_a,
    int sigmoide_c);