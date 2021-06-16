/*
 * Aquisição funcionando,
 * falta ajeitar a trajetória e conferir a centralização
 * */
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
/*Library files include*/
#include "mpu_sensor.h"
#include "Sensor_Fusion.h"
#include "Drivers_Control.h"
#include "Control_Law.h"
#include "function.h"

/* Misc. definitions. */
#define TEMPO_ESTABILIZACAO_MOTOR 1
#define SIZE_VECTOR_SINAIS_MOTOR 3001 // Sin Wave 0.2Hz
//#define SIZE_VECTOR_SINAIS_MOTOR 4001 // Square Wave 0.2Hz
extern float sin_wave_trajeto[SIZE_VECTOR_SINAIS_MOTOR];

/* Instance of Variables */
// Variables for Luenberger 
Luenberger_data luenberger_Rho = { .pole1 = 0.98f, .pole2 = 0.95f };
// Instance for Kalman Filter
Kalman_data kalman_Rho = { .Q_angle = 0.01f, .Q_bias = 0.001f, .R_measure =
                                   0.03f,
                           .angle = 0.0f, .bias = 0.0f };
// Sensor definition
dr_mpu_data_t sensor_rho = { .identification = 0, .address = 0x68, .I2C = 0,
                             .calibration_offset_mag = 0, .Axis_Magnetometer = {
                                     2, 1 },
                             .Axis_Gyroscope = 3 };
//PWM definitions
dr_pwm_parameters pwm_Rho = { .identification = 0, .timer = TIMER_A0_BASE,
                              .fast_mode = true, .timer_Prescaler = 64,
                              .timer_Prescaler = 4,
                              .true_Sawtooth_not_triangular = true,
                              .period_count = 60000, //60360 = 50 Hz(fechado, osciloscÃ³pio)
                              .pwm_channel = 1, .outputmode = //Config TA0.1
                                      TIMER_A_OUTPUTMODE_RESET_SET };
Control_Law_data controller_Rho = { .identification = 0,
                                    .PI_Kp = 50.0f, //10.0f
        .PI_Ki = 0.1f, .upper_limit = 5.0f, .lower_limit = -5.0f };
// upper = 5.0f and lower  -5.0f for sin wave

/* For Debug graph*/

/*Sigmoide Filter*/
float a = -1;      //-0.1
float c = 6; //6
double processing_time = 0;

// Print aquision variables
uint32_t time_acquisition = 0;
bool acquisition_Data_Start = 0;
int_fast16_t aq_data1[4];
int_fast16_t aq_data2[5];
int_fast16_t aq_data3[6];
int_fast16_t aq_data4[7];
bool acquisition_Data_Stop_Receive = 0;

/* Global Variables for access Routine calibration and centralize*/
uint8_t routine_calibration = 0;
uint8_t centralize_engine = 0;
uint8_t routine_unwinding = 0;

/* *******************************************
 * CENTRALIZE ENGINE ROUTINE
 * *******************************************
 * */

int8_t DR_centralize_Engine(uint32_t stabilization_time)
{
    static uint32_t contador_time = 0;
    if (contador_time > stabilization_time)
    {
        contador_time = 0;
        Dr_clc_led;
        return 0;
    }
    else
    {
        Dr_set_led;
        float Control_Signal = PI_Control_Law(&controller_Rho, 0);
        setPosition_ServoMotor(&pwm_Rho, Control_Signal);
        contador_time++;
        return 1;
    }

}
/* *******************************************
 * UNWINDING CABLE ENGINE ROUTINE
 * *******************************************
 * */
int8_t DR_unwinding_cable(uint32_t unwinding_time, float Signal_motor)
{
    Dr_clc_led;
    uint32_t unwinding_counter = 0;
    if (unwinding_counter < unwinding_time)
    {
        setPosition_ServoMotor(&pwm_Rho, Signal_motor);
        unwinding_counter++;
        Dr_set_led;
        return 1;
    }
    else
    {
        setPosition_ServoMotor(&pwm_Rho, 0);
        return 0;
    }

}
void DR_moving_Avarege(dr_mpu_data_t *sensor, uint32_t size_horizon)
{ // function to set calibration_offset_mag
    uint32_t avarege_counter = 0;
    float current_value = 0;
    float preview_value = 0;
    float average = 0;

    while (avarege_counter < size_horizon)
    {
        preview_value = average;

        DR_mpu9250_atualizar(sensor);  // read sensors
        Dr_mag_signal_update(sensor); // update new magnetometer signal

        current_value = sensor->mag_Signal;

        average = (current_value + preview_value) / 2;

        avarege_counter++;
        Dr_clc_led;
    }
    Dr_set_led;
//    sensor->calibration_offset_mag = average;
}

// retorna o sinal de controle
void DR_acquisition_data(uint16_t n_ensaio)
{
    time_acquisition++;
    if (n_ensaio == 0)
    {

    }
    if (n_ensaio == 1)
    { /* Ensaio 1 :: Kalman {Magnetometer + Giroscope}
     * 1 - Sinal Magnetometer
     * 2 - Sinal Giroscope
     * 3 - Sinal da Fusão  (Kalman Angle)
     * */
        aq_data1[0] = 1000 * sensor_rho.mag_Signal;
        aq_data1[1] = 1000 * sensor_rho.gyro_Signal;
        aq_data1[2] = 1000 * sensor_rho.ang_Kalman;
        printf("\n\r%d %d %d %d", time_acquisition, aq_data1[0], aq_data1[1],
               aq_data1[2]);
    }
    if (n_ensaio == 2)
    {
        /*Ensaio 2 :: Kalman {Magnetometer (w/FSE) + Giroscope}
         * 1 - Sinal Magnetometer (antes FSE)   -- sensor_rho.ang_pitch
         * 2 - Sinal Magnetometer (depois FSE)  -- ang_pitch
         * 3 - Sinal Giroscope                  -- ang_gyro
         * 4 - Sinal da Fusão (Kalman Angle)    -- ang_Kalman
         * */
        aq_data2[0] = 1000 * sensor_rho.mag_Signal;
        aq_data2[1] = 1000 * sensor_rho.mag_Signal_Filtered;
        aq_data2[2] = 1000 * sensor_rho.gyro_Signal;
        aq_data2[3] = 1000 * sensor_rho.ang_Kalman;

        printf("\n\r%d %d %d %d %d", time_acquisition, aq_data2[0],
               aq_data2[1], aq_data2[2], aq_data2[3]);
    }
    if (n_ensaio == 3)
    {
        /* Ensaio 3 :: Luenberger {Magnetometer + Giroscope}
         * 1 - Sinal Magnetometer                   -- sensor_rho.ang_pitch
         * 2 - Sinal Giroscope                      -- sensor_rho.ang_gyro
         * 3 - Residuo Observador (Magnetometer)    -- luenberger_Rho.residue
         * 4 - Rate Observador (Giroscope)          -- luenberger_Rho.rate
         * 5 - Sinal da Fusão (Luenberger Angle)    -- sensor_rho.ang_Luenberger
         ****/
        aq_data3[0] = 1000 * sensor_rho.mag_Signal;
        aq_data3[1] = 1000 * sensor_rho.gyro_Signal;
        aq_data3[2] = 1000 * luenberger_Rho.residue;
        aq_data3[3] = 1000 * luenberger_Rho.rate;
        aq_data3[4] = 1000 * sensor_rho.ang_Luenberger;

        printf("\n\r%d %d %d %d %d %d", time_acquisition, aq_data3[0],
               aq_data3[1], aq_data3[2], aq_data3[3], aq_data3[4]);
    }
    if (n_ensaio == 4)
    {
        /*
         * Ensaio 4 :: Luenberger {Magnetometer(w/FSE) + Giroscope}
         * 1 - Sinal Magnetometer (antes FSE)      -- sensor_rho.ang_pitch
         * 2 - Sinal Magnetometer (depois FSE)     -- sensor_rho.ang_pitch
         * 3 - Sinal Giroscope                     -- sensor_rho.ang_gyro
         * 4 - Residuo Observador (Magnetometer)   -- luenberger_Rho.residue
         * 5 - Rate Observador (Giroscope)         -- luenberger_Rho.rate
         * 6 - Sinal da Fusão (Luenberger Angle)   -- sensor_rho.ang_Luenberger
         *
         * */
        aq_data4[0] = 1000 * sensor_rho.mag_Signal;
        aq_data4[1] = 1000 * sensor_rho.mag_Signal_Filtered;
        aq_data4[2] = 1000 * sensor_rho.gyro_Signal;
        aq_data4[3] = 1000 * luenberger_Rho.residue;
        aq_data4[4] = 1000 * luenberger_Rho.rate;
        aq_data4[5] = 1000 * sensor_rho.ang_Luenberger;

        printf("\n\r%d %d %d %d %d %d %d", time_acquisition, aq_data4[0],
               aq_data4[1], aq_data4[2], aq_data4[3], aq_data4[4], aq_data4[5]);
    }
    if (n_ensaio == 5)
    {

    }
}

void Update_Servo_Motor(uint16_t position_table, uint8_t prbs_on)
{
    float Rerencia_Motor, Duty_Table_Value;
    if (prbs_on == 0)
    {
        Rerencia_Motor = sin_wave_trajeto[position_table]
                - sensor_rho.calibration_offset_mag;
        Duty_Table_Value = PI_Control_Law(&controller_Rho, Rerencia_Motor);
    }

    setPosition_ServoMotor(&pwm_Rho, Duty_Table_Value);
}

int DR_Sensor_Fusion(uint16_t n_ensaio)
{
    if (n_ensaio == 0)
    { /* Classical Filtering x Luenberger with SSF */
        /*Luenberger {Magnetometer Filtered + Gyroscope}*/
        sensor_rho.ang_Luenberger = getAngle_Luenberger(
                &luenberger_Rho, sensor_rho.mag_Signal_Filtered,
                sensor_rho.gyro_Signal, Ts);
        /*Kalman {Magnetometer + Gyroscope}*/
        sensor_rho.ang_Kalman = getAngle_Kalman(&kalman_Rho,
                                                sensor_rho.mag_Signal,
                                                sensor_rho.gyro_Signal,
                                                Ts);

        return 1;
    }
    if (n_ensaio == 1)
    { /* Classical Filtering Kalman */
        sensor_rho.ang_Kalman = getAngle_Kalman(&kalman_Rho,
                                                sensor_rho.mag_Signal,
                                                sensor_rho.gyro_Signal, Ts);

        return 1;
    }
    if (n_ensaio == 2)
    { /* Kalman Fusion with Magnetometer filtered with SSF */
        sensor_rho.ang_Kalman = getAngle_Kalman(&kalman_Rho,
                                                sensor_rho.mag_Signal_Filtered,
                                                sensor_rho.gyro_Signal, Ts);

        return 1;
    }
    if (n_ensaio == 3)
    { /* Luenberger Classical Fusion */
        sensor_rho.ang_Luenberger = getAngle_Luenberger(&luenberger_Rho,
                                                        sensor_rho.mag_Signal,
                                                        sensor_rho.gyro_Signal,
                                                        Ts);

        return 1;
    }
    if (n_ensaio == 4)
    { /* Luenberger Fusion with Magnetometer filtered by SSF*/
        sensor_rho.ang_Luenberger = getAngle_Luenberger(
                &luenberger_Rho, sensor_rho.mag_Signal_Filtered,
                sensor_rho.gyro_Signal,
                Ts);

        return 1;
    }
    if (n_ensaio == 5)
    {/* Kalman Fusion with SSF x Luenberger Fusion with SSF */
        /*Luenberger {Magnetometer Filtered + Gyroscope}*/
        sensor_rho.ang_Luenberger = getAngle_Luenberger(
                &luenberger_Rho, sensor_rho.mag_Signal_Filtered,
                sensor_rho.gyro_Signal, Ts);
        /*Kalman {Magnetometer + Gyroscope}*/
        sensor_rho.ang_Kalman = getAngle_Kalman(&kalman_Rho,
                                                sensor_rho.mag_Signal_Filtered,
                                                sensor_rho.gyro_Signal,
                                                Ts);
        return 1;
    }
    else
    {
        return 0;
    }
}

void main_interrupt()
{
// --------------------- Disable the flag and Start processing time -------------//
    Timer32_clearInterruptFlag(TIMER32_0_BASE);

//------------------  Data acquisition Routine ----------------------------------//
    if (acquisition_Data_Start)
    { // Start Acquisition Data
        static uint16_t update_position_servo_value = 0;
        static uint8_t teste_routine = 0;
        if (teste_routine == 0)
        { // Initialize data Acquisition Routine
            teste_routine++;
            time_acquisition = 0;
            update_position_servo_value = 0;
            DR_leds_alterar(1);
            Dr_clc_led;

        }
        if (update_position_servo_value >= SIZE_VECTOR_SINAIS_MOTOR)
        { // Finished reading the test signal vector
            if (teste_routine < 4)
            { // end of test routine
//              teste_routine = 0;
                teste_routine++;        // next routine
                time_acquisition = 0;   // reset acquisition time
                update_position_servo_value = 0; // reset the vector scan position
            }
            else
            { // end of all test routines
                Dr_set_led;
                teste_routine = 0;           // Restart routines
                acquisition_Data_Start = 0;  // Finish Data_Acquisition
//                DR_acquisition_data(5); // do the data acquisition
                centralize_engine = 2; //  Centralize and unwind the motor cables
                DR_leds_alterar(7);
                Dr_clc_led;

            }
        }
        else
        { // Update new engine position, read and store new data
            DR_mpu9250_atualizar(&sensor_rho);  // read sensors
            Dr_mag_signal_update(&sensor_rho); // update new magnetometer signal
            Dr_gyro_signal_update(&sensor_rho); // update new gyroscope signal
            sensor_rho.mag_Signal_Filtered = calculateSSF_in_realTime(
                    sensor_rho.mag_Signal, a, c); // Filter the magnetometer signal
            DR_Sensor_Fusion(teste_routine);    // do the sensory fusion
            DR_acquisition_data(teste_routine); // do the data acquisition
            Update_Servo_Motor(update_position_servo_value++, 0); // Update new engine position
        }
    }

//------------------- Routines outside of Data acquisition ----------------------//
    else
    { // Engine calibration and centralization routines
        /* ***************************************************************
         * MAGNETOMETER CALIBRATION ROUTINE
         * ***************************************************************
         * */
        if (routine_calibration == 1)
        { // do calibration routine
            int8_t calibrating = DR_magnetometer_calibrate(&sensor_rho);
            setPosition_ServoMotor(&pwm_Rho, 3); // rotate the engine counterclockwise

            if (!calibrating)
            { // calibration completed
                setPosition_ServoMotor(&pwm_Rho, 0); // stop the engine
                centralize_engine = 3; // center the engine
                routine_calibration = 0; // stop calibration routine
                Dr_clc_led;
                DR_leds_alterar(4);
            }

        }
        else
        { // update magnetometer signal
            DR_mpu9250_atualizar(&sensor_rho); // update the sensor signal
            Dr_mag_signal_update(&sensor_rho); // update magnetometer signal
        }
        if (routine_unwinding == 1)
        {
            int8_t unwinding = DR_unwinding_cable(SIZE_MAGNETOMETER_CALIBRATION,
                                                  -3);
            Dr_clc_led;
            if (!unwinding)
            { // unwinding completed
                setPosition_ServoMotor(&pwm_Rho, 0); // stop the engine
                centralize_engine = 4; // center the engine
                routine_unwinding = 0; // stop calibration routine
                Dr_set_led;
            }

        }
        /* *************************************************************
         * ROUTINE TO CENTRALIZE THE ENGINE POSITION
         ***************************************************************
         * */
        if (centralize_engine >= 1)
        { // centralize the engine
            int8_t stabilizing = DR_centralize_Engine(500);
            if (!stabilizing)
            { // stabilized engine
                switch (centralize_engine)
                {
                case 1: // center the engine before performing the calibration
                    Dr_set_led;
                    routine_calibration = 1;
                    Dr_clc_led;
                    DR_leds_alterar(3);

                    break;
                case 2: // center the engine after all tests and calibration
//                    routine_unwinding = 1;
                    Dr_set_led;
                    DR_leds_alterar(3);
                    while (acquisition_Data_Stop_Receive == 0)
                    {

                        if (UART_receiveData(EUSCI_A0_BASE) == 50)
                        {
                            acquisition_Data_Stop_Receive = 1;
                        }
                        else
                        {
                            Dr_toogle_led
                            ;
                            printf("\n\r%d %d %d", 0, 0, 0);
                            DR_delay_k(10);
                        }

                    }
                    acquisition_Data_Stop_Receive = 0;
                    DR_leds_alterar(2);
                    break;
                case 3: // Center the engine and calculate mean
                    Dr_set_led;
                    DR_moving_Avarege(&sensor_rho, 500);
                    Dr_clc_led;
                    DR_leds_alterar(5);
                    break;
                default:
                    break;
                }
                centralize_engine = 0; // motor centering finished
            }
        }
//------------------- End of Main Routine ---------------------------------------//
//        processing_time = DR_tick_stop(false);
    }
}

int main(void)
{
    WDT_A_holdTimer();

    /* Pin Config */
    DR_leds_sw_pin();
    DR_uart_pin();
    DR_i2c_pin();
    DR_pwm_pin();
    DR_adc_pin();

    /* Peripherals Config */
    DR_uart_config(true);
    DR_i2c_config(0);
    DR_t32_config_Hz(0, 100);
    DR_pwm_config(&pwm_Rho);
    DR_adc_config();

    /* Initialize Programs*/
    DR_leds_init(true);
    DR_uart_init();
    DR_pwm_init(&pwm_Rho, 4450);
    DR_i2c_init(0);
    DR_t32_init(0);
    DR_adc_init();

    DR_mpu9250_init(&sensor_rho);

    /* Gyroscope Calibration*/
    Dr_clc_led;
    DR_leds_alterar(1);
    DR_gyroscope_calibrate(&sensor_rho, 1000); //era 1000
    Dr_set_led;

    /* Poles Luenberger */
    setPoles_Luenberger(&luenberger_Rho, Ts);

    /* Interrupt Config */
    DR_uart_interrupt_receive();
    DR_t32_interrupt_init(0, main_interrupt);
    /* Enabling interrupts */
    DR_adc_interrupt();
    DR_interrupt_on();
    /*Centralize Engine in 100Hz with PI Control*/
    Dr_clc_led;
    DR_leds_alterar(2);
    centralize_engine = 1;
    while (1)
    {
    }
}

void EUSCIA0_IRQHandler(void)
{
    UART_clearInterruptFlag(EUSCI_A0_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    uint8_t count_RX_Buffer = UART_receiveData(EUSCI_A0_BASE) - 48;
    printf("\n\n\r--ReceiveRX(%d)", count_RX_Buffer);
    if (count_RX_Buffer == 1)
    {
        acquisition_Data_Start = 1;
    }
}
/*
 * ADC Interrupt Handler. This handler is called whenever there is a conversion
 * that is finished for ADC_MEM0.
 */
void ADC14_IRQHandler(void)
{
    uint64_t status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);
    if (ADC_INT0 & status)
    {
        uint_fast16_t ADC_result = ADC14_getResult(ADC_MEM0);
        controller_Rho.y = getPosition_ServoMotor(&pwm_Rho, ADC_result);
        ADC14_toggleConversionTrigger();
    }
}
