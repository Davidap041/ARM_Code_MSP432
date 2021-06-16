#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_
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
// Converter Tension(Potentiometer) in Angle (radians) for link Rho
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

// Variáveis da Estimação de posição do Elo 1 Ângulo Rho
// valores Passados de Entrada e Saída atualidos na frequência de 100Hz
float y_estimado[5], u_estimado[2];

typedef struct Coordinates_Struct
{
    float ref_time; /* time */

    float x;
    float y;
    float z;

    float rho;
    float theta1;
    float theta2;
    float theta3;

} Coordinates_data;

/*Calculate Circle Reference Trajectory */
void References_Circle(Coordinates_data *reference)
{
    reference->ref_time = reference->ref_time + Ts;
    float x, y, z;
    if (reference->ref_time < 5 * pi_2)
    {
        x = 2.0f * cosf(reference->ref_time);
        y = 8.0f;
        z = 2.0f * sinf(reference->ref_time);
        z = 25.0f + z;

    }
    else
    {
//		x = 8 + 3 * cos(5 * pi_2);
//		y = 3;
//		z = 12 + 3 * sin(5 * pi_2);
        reference->ref_time = pi_2;
    }
    reference->x = x;
    reference->y = y;
    reference->z = z;
}

/*Transform x,y,z from reference in rho,theta1,theta2 e theta3 for reference*/
void Inverse_Kinematic(Coordinates_data *reference)
{
#define L3 8.0f
#define L2 13.0f
#define L1 13.0f
#define L1L1 169.0f
#define L2L2  169.0f

    float r, phi, beta, qsi, qsi_p1, qsi_p2, qsi_p3, qsi_p4, rho, theta1,
            theta2, theta2_p1, theta2_p2, theta2_p3, theta2_p4, theta3, beta_1,
            beta_2;

    float x, y, z;
    x = reference->x;
    y = reference->y;
    z = reference->z;

//	r(i) = sqrt(x(i)^2 + z_(i)^2);
    r = sqrtf(x * x + z * z);

//	phi(i) = -atan2(y(i),r(i));
    phi = -atan2f(y, r);

//	beta(i) = atan2(y(i)-l3*sin(phi(i)),r(i) - l3*cos(phi(i)));
    beta_1 = y - L3 * sinf(phi);
    beta_2 = r - L3 * cosf(phi);

    beta = atan2f(beta_1, beta_2);

//	qsi(i) = acos(((r(i) - l3*cos(phi(i)))^2 + (y(i) - l3*sin(phi(i)))^2 + l1^2 - l2^2)
//				/(2*l1*sqrt((r(i) - l3*cos((phi(i))))^2 + (y(i) - l3*sin(phi(i)))^2)));
    qsi_p1 = ((r - L3 * cosf(phi))) * (r - L3 * cosf(phi));
    qsi_p2 = (y - L3 * sinf(phi)) * (y - L3 * sinf(phi)) + L1L1 - L2L2;

    qsi_p3 = (r - L3 * cosf(phi)) * (r - L3 * cosf(phi));
    qsi_p4 = (y - L3 * sinf(phi)) * (y - L3 * sinf(phi));
    qsi_p3 = 2.0f * L1 * sqrtf(qsi_p3 + qsi_p4);

    qsi = (qsi_p1 + qsi_p2) / (qsi_p3);

    qsi = acosf(qsi);

//	rho(i) = atan2(x(i),z_(i));
    rho = atan2f(x, z);

//	theta1(i) = (beta(i) + qsi(i)) - pi/2;
    theta1 = (beta + qsi) - pi_2;

//	theta2(i) = -acos(((r(i) - l3*cos(phi(i)))^2 +
//			(y(i) - l3*sin(phi(i)))^2 - l1^2 - l2^2)/(2*l1*l2));
    theta2_p1 = (r - L3 * cosf(phi)) * (r - L3 * cosf(phi));
    theta2_p2 = (y - L3 * sinf(phi)) * (y - L3 * sinf(phi));
    theta2_p3 = L1L1 + L2L2;
    theta2_p4 = 2.0f * L1 * L2;
    theta2 = -acosf((theta2_p1 + theta2_p2 - theta2_p3) / theta2_p4);

//	theta3(i) = (phi(i) - theta1(i) - pi/2 - theta2(i));
    theta3 = phi - theta1 - pi_2 - theta2;

//	Posicionamento dos Eixos
    rho = -rho;
    theta1 = -theta1;
    theta2 = -theta2;
    theta3 = -theta3;

    reference->rho = rho;
    if (theta1 > 0 && theta1 < 5)
    {
        reference->theta1 = theta1;
    }
    if (theta2 > 0 && theta2 < 5)
    {
        reference->theta2 = theta2;
    }
    reference->theta3 = theta3;

}
// Estimar Posição recebendo a entrada atual de angulo in 100 Hz!!
float estimar_posicao_rho(float entrada)
{
//    0.01931 z^-1
//-------------------------------------------------------------------
//1 - 3.635 z^-1 + 5.683 z^-2 - 4.721 z^-3 + 2.068 z^-4 - 0.3792 z^-5

//	Ordem do Polinômio de Estimação
    int k = 6;

    y_estimado[k - 5] = y_estimado[k - 4];
    y_estimado[k - 4] = y_estimado[k - 3];
    y_estimado[k - 3] = y_estimado[k - 2];
    y_estimado[k - 2] = y_estimado[k - 1];
    y_estimado[k - 1] = y_estimado[k];

    u_estimado[k - 1] = u_estimado[k];
    u_estimado[k] = entrada;

    y_estimado[k] = rho_b0 * u_estimado[k] + rho_b1 * u_estimado[k - 1]
            - rho_a1 * y_estimado[k - 1] - rho_a2 * y_estimado[k - 2]
            - rho_a3 * y_estimado[k - 3] - rho_a4 * y_estimado[k - 4]
            - rho_a5 * y_estimado[k - 5];

    return y_estimado[k];
}
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

//}
#endif /* FUNCTIONS_H_ */
