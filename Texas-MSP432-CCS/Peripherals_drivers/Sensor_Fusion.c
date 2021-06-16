#include "Sensor_Fusion.h"
/*
 * Function to Filter SSF in real Time
 * */
float calculateSSF_in_realTime(float data_real_time, int sigmoide_a,
                               int sigmoide_c)
{
    int k;
    static float32_t buffered_signal[BUFFER_SIGNAL_LENGTH];
    static float32_t fft_data_input[BUFFER_SIGNAL_LENGTH * 2];
    static float32_t fft_data_output[FFT_SIZE];
    static float32_t fft_filtered[FFT_SIZE];
//    static float32_t eq_sigmoid[FFT_SIZE];
    static float32_t ifft_data_output[FFT_SIZE];
    static uint32_t buffer_counter = 0; // counter for buffering signal

    arm_rfft_fast_instance_f32 instance;

    if (buffer_counter > BUFFER_SIGNAL_LENGTH - 1)
    {
        buffer_counter = 0;
        buffered_signal[buffer_counter] = data_real_time; // Magnetometer
        buffer_counter++;
    }
    else
    {
        buffered_signal[buffer_counter] = data_real_time;
        buffer_counter++;
    }

    /* Computer real FFT using the completed data buffer */
    for (k = 0; k < BUFFER_SIGNAL_LENGTH; k++)
    {
        if (k + buffer_counter >= BUFFER_SIGNAL_LENGTH - 1)
        {
            fft_data_input[k] = buffered_signal[k + buffer_counter - 1
                    - BUFFER_SIGNAL_LENGTH];
        }
        else
        {
            fft_data_input[k] = buffered_signal[k + buffer_counter - 1];
        }
    }
//    for (k = BUFFER_SIGNAL_LENGTH / 2; k < BUFFER_SIGNAL_LENGTH - 1; k++)
//    {
//        fft_data_input[k] = 0;
//    }

    volatile arm_status status = arm_rfft_fast_init_f32(&instance, FFT_SIZE);
    arm_rfft_fast_f32(&instance, fft_data_input, fft_data_output, 0);

    /*Multiplicate for sigmoide*/
    for (k = 0; k < FFT_SIZE; k++) // o certo era 0
    {
        if (k < 2)
        {
            fft_filtered[k] = fft_data_output[k];
        }
        else
        {
            fft_filtered[k] = fft_data_output[k]
                    * (1 / (1 + expf(-sigmoide_a * (k - sigmoide_c))));
//            eq_sigmoid[k] = 1 / (1 + expf(-sigmoide_a * (k - sigmoide_c)));
        }

    }

    /* Calculate Inverse FFT*/
    arm_rfft_fast_f32(&instance, fft_filtered, ifft_data_output, 1);
    /* last data value in time */
    return ifft_data_output[FFT_SIZE - 1];
}
/***********************************************************
 * Luenberger Filter for Sensor Fusion
 **********************************************************/
float getAngle_Luenberger(Luenberger_data *luenberger, float newAngle,
                          float newRate, float dt)
{
    // Store previous values
    luenberger->angle_prev = luenberger->angle;
    luenberger->bias_prev = luenberger->bias;
    // Calculate residues in velocity and position
    luenberger->rate = newRate - luenberger->bias;
    luenberger->residue = newAngle - luenberger->angle;
    // Estimate Angle and Velocity for next iteration
    luenberger->angle = luenberger->angle_prev
            + luenberger->L1_ * luenberger->residue + dt * luenberger->rate;
    luenberger->bias += luenberger->L2_ * luenberger->residue;

    return luenberger->angle_prev;
}

void setPoles_Luenberger(Luenberger_data *luenberger, float dt)
{
    luenberger->L1_ = 2 - luenberger->pole1 - luenberger->pole2;
    float fs = 1 / dt;
    luenberger->L2_ = (1 - luenberger->L1_
            - luenberger->pole2 * luenberger->pole1) * fs;
}

/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

/* We will set the variables like so, these can also be tuned by the user */
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle_Kalman(Kalman_data *kalman, float newAngle, float newRate,
                      float dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    kalman->P[0][0] += dt
            * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0]
                    + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = kalman->P[0][0] + kalman->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - kalman->angle; // Angle difference
    /* Step 6 */
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}
