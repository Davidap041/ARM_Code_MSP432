#include "Motores.h"
/*Private Functions*/
/*Transform Angle in Pwm and set Servo Position*/
void setPosition_ServoMotor(dr_pwm_parameters *PWM_link, float ControlSignal)
{
    /*Transform PWM for Angle OpenLoop*/
    float static Duty_Value;
    switch (PWM_link->identification)
    {
    case 0:
        /*Update Rho_ServoMotor */
        Duty_Value = motor0_a * ControlSignal + motor0_b;
        break;
    case 1:
        /*Update Theta1_ServoMotor*/
        Duty_Value = motor1_a * ControlSignal + motor1_b;
        break;
    case 2:
        /*Update Theta2_ServoMotor*/
        Duty_Value = motor2_a * ControlSignal + motor2_b;
        break;
    case 3:
        /*Update Theta3_ServoMotor*/
        Duty_Value = motor3_a * ControlSignal + motor3_b;
        break;
    default:
        break;
    }
    DR_PWM_setDuty(PWM_link, Duty_Value);
}
/*Transform Actual Postion of PWM Servo Motor (directly PWM registers) in Angle (rad)*/
float getPosition_ServoMotor(dr_pwm_parameters *PWM_link,
                             uint_fast16_t ADC_value)
{
    float Angle_Position; // Servo Motor Angle Corresponding Position in rad
    switch (PWM_link->identification)
    {
    case 0:
        /*Update Rho_ServoMotor */
        Angle_Position = ang_0_a * ADC_value + ang_0_b;
        break;
    case 1:
        /*Update Theta1_ServoMotor*/
        Angle_Position = ang_1_a * ADC_value + ang_1_b;
        break;
    case 2:
        /*Update Theta2_ServoMotor*/
        Angle_Position = ang_2_a * ADC_value + ang_2_b;
        break;
    case 3:
        /*Update Theta3_ServoMotor*/
        Angle_Position = ang_3_a * ADC_value + ang_3_b;
        break;
    default:
        break;
    }
    return Angle_Position;
}
/*Centralizer Servo Motor*/
int8_t DR_centralize_Engine(uint32_t stabilization_time)
{ /*using in real time update*/
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
        return 1;   /*Finish Centralize_Engine*/
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
float PD_Local_ServoControl(Servo_Control_data* link, float reference)
{   link->ref = reference; //  Update Reference Value
    link->previous_error = link->error; //  Update Preview Error
    link->previous_u = link->u; // Update Preview Control Signal
    /* Calculate error*/
    link->error = (link->ref - link->y); // Calculate Current error
    /* Control PD - Davi*/
    link->u = link->PD_Kp * link->error
        + link->PD_Kd * (link->error - link->previous_error) / Ts;

    if (link->u > link->upper_limit)
    {
        link->u = link->upper_limit;
    }
    if (link->u < link->lower_limit)
    {
        link->u = link->lower_limit;
    }
    return link->u;
}
