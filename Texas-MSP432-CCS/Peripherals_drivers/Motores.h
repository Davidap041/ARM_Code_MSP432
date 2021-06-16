#ifndef MOTORES_H_
#define MOTORES_H_
#include <math.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <mpu_sensor.h>

#define PI 3.14159265358979f
#define pi_2 1.570796f
#define Ts 0.01

// Calibration Parameters for Servo Motor
// Limits of the PWM signal and corresponding signal control for values for continuous serve control
#define rho_pwm_max 4650.0f // Maximum PWM for the Rho motor
#define rho_pwm_min 4250.f  // Minimum PWM for the Rho motor
#define rho_signal_max 10.0f      // Define Maximum Control Signal
#define rho_signal_min -10.0f     // Define Minimum Control Signal
#define adc_signal_max 16384
#define adc_signal_min 0
// Limits of the PWM signal and corresponding angle values for link Theta 1
#define theta1_pwm_max 2000.0f
#define theta1_pwm_min 4800.0f
#define theta1_angle_max 1.972f
#define theta1_angle_min 0.562f
// Limits of the PWM signal and corresponding angle values for link Theta 2
#define theta2_pwm_max 5400.0f
#define theta2_pwm_min 1100.0f
#define theta2_angle_max 0.961796f
#define theta2_angle_min 3.142796f
// Limits of the PWM signal and corresponding angle values for link Theta 3
#define theta3_pwm_max 1200.0f
#define theta3_pwm_min 5500.0f
#define theta3_angle_max 2.510996f
#define theta3_angle_min 0.11979f
// Converter PWM(uC) in control Signal for link Rho
#define motor0_a ((rho_pwm_max - rho_pwm_min) / (rho_signal_max - rho_signal_min))
#define motor0_b ((rho_signal_max*rho_pwm_min - rho_signal_min*rho_pwm_max)/(rho_signal_max - rho_signal_min))
// Converter PWM(uC) in Angle(rad) for link Theta 1
#define motor1_a ((theta1_pwm_max - theta1_pwm_min) / (theta1_angle_max - theta1_angle_min))
#define motor1_b ((theta1_angle_max*theta1_pwm_min - theta1_angle_min*theta1_pwm_max)/(theta1_angle_max - theta1_angle_min))
// Converter PWM(uC) in Angle(rad) for link Theta 2
#define motor2_a ((theta2_pwm_max - theta2_pwm_min) / (theta2_angle_max - theta2_angle_min))
#define motor2_b ((theta2_angle_max*theta2_pwm_min - theta2_angle_min*theta2_pwm_max)/(theta2_angle_max - theta2_angle_min))
// Converter PWM(uC) in Angle(rad) for link Theta 3
#define motor3_a ((theta3_pwm_max - theta3_pwm_min) / (theta3_angle_max - theta3_angle_min))
#define motor3_b ((theta3_angle_max*theta3_pwm_min - theta3_angle_min*theta3_pwm_max)/(theta3_angle_max - theta3_angle_min))
// Converter Voltage(Potentiometer) in Angle (radians) for link Rho
#define ang_0_a (2*PI / (adc_signal_max-adc_signal_min))
#define ang_0_b (-PI)
// Converter Angle(rad) in PWM(uC) for link Theta 1
#define ang_1_a ((theta1_angle_max - theta1_angle_min) / (theta1_pwm_max - theta1_pwm_min))
#define ang_1_b ((theta1_pwm_max*theta1_angle_min - theta1_pwm_min*theta1_angle_max)/(theta1_pwm_max - theta1_pwm_min))
// Converter Angle(rad) in PWM(uC) for link Theta 2
#define ang_2_a ((theta2_angle_max - theta2_angle_min) / (theta2_pwm_max - theta2_pwm_min))
#define ang_2_b ((theta2_pwm_max*theta2_angle_min - theta2_pwm_min*theta2_angle_max)/(theta2_pwm_max - theta2_pwm_min))
// Converter Angle(rad) in PWM(uC) for link Theta 3
#define ang_3_a ((theta3_angle_max - theta3_angle_min) / (theta3_pwm_max - theta3_pwm_min))
#define ang_3_b ((theta3_pwm_max*theta3_angle_min - theta3_pwm_min*theta3_angle_max)/(theta3_pwm_max - theta3_pwm_min))

// Parameter to estimate position for motor rho
#define rho_a0 1.0f
#define rho_a1 -3.635f
#define rho_a2 5.683f
#define rho_a3 -4.721f
#define rho_a4 2.068f
#define rho_a5 -0.3792f

#define rho_b0 0.0f
#define rho_b1 0.01931f

typedef struct LocalServoControl
{
    int identification;	// link identification

    // PD Serve Motor control Variables
    float PD_P;
    float PD_D;
    float PD_Kp;
    float PD_Kd;

    // Control Law Variables
    float error;   // Current error
    float derivate_error; // Derived error
    float previous_error; // Previous errors

    float u;    // Control Signal
    float previous_u;    // Previous Control Signal
    float y; // Output Measure
    float previous_y; // Previous Output measure

    float dy; 	// Derived Output
    float dref; // Derived Reference
    float ref; // Reference for controller
    float previous_ref; // Previous reference for controller

    // Saturator Variables
    float upper_limit;
    float lower_limit;

} Servo_Control_data;

typedef struct Motor {
	dr_pwm_parameters PWM;
	
} Motor_Data;
/*Transform Angle in Pwm and set Servo Position*/
void setPosition_ServoMotor(dr_pwm_parameters* PWM_link, float ControlSignal);
/*Transform Actual Postion of PWM Servo Motor (directly PWM registers) in Angle (rad)*/
float getPosition_ServoMotor(dr_pwm_parameters* PWM_link, uint_fast16_t ADC_value);
#endif /* MOTORES_H_ */
