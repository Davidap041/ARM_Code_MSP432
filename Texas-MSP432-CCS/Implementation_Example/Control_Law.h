/*
 * Control_Law.h
 *
 *  Created on: 5 de mar de 2020
 *      Auyor: davia
 */

#ifndef CONTROL_LAW_H_
#define CONTROL_LAW_H_

typedef struct ControlStruct
{
    int identification;	// link identification

    //  PD Mechanical Impedance control Variables
    float PD_Kp;
    float PD_Kv;
    float m;    // mass of model
    float b;	// damper constant of model
    float k;	// spring constant of model
    float means; // average of identification test

    float feedforward_linear_controller;
    float primary_controller;

    // PI Serve Motor control Variables
    float PI_P;
    float PI_I;
    float PI_Kp;
    float PI_Ki;

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

} Control_Law_data;

extern float PI_Control_Law(Control_Law_data *link, float reference);
/*Calculate Control Signal "u" by PD Control (look if ye order is right!)*/
extern float Pd_Control_Law(Control_Law_data *link, float reference);
/* Calculate Control Signal "u" by Open Loop Control (look if ye order is right!)*/
extern float Open_Loop_Control_Law(Control_Law_data *link, float reference);

#endif /* CONTROL_LAW_H_ */
