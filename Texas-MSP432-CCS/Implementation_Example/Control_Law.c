#include "Control_Law.h"

float PI_Control_Law(Control_Law_data *link, float reference)
{ // PI Serve Motor control
    /* Store Previous Variables*/
    link->ref = reference; //  Update Reference Value
    link->previous_error = link->error; //  Update Preview Error
    link->previous_u = link->u; // Update Preview Control Signal
    /* Calculate error*/
    link->error = (link->ref - link->y); // Calculate Current error
    /* Calculate P and I */
    /* Control PI meu*/
//    link->PI_P = link->PI_Kp * (link->error - link->previous_error);
//    link->PI_I = link->PI_Ki * link->error;
//    link->u = link->previous_u + link->PI_P + link->PI_I;/* Calculate control Signal*/
    /*Control PI - Felipe*/
//    link->u = link->previous_u
//            + (link->PI_Kp + (link->PI_Ki * 0.01)) * link->error
//            - link->previous_error;
    /* Control PD - Davi*/
    link->u = link->PI_Kp * link->error
            + link->PI_Ki * (link->error - link->previous_error) / 0.01;

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
/*Calculate Control Signal "u" by PD Control (look if ye order is right!)*/
float Pd_Control_Law(Control_Law_data *link, float reference)
{
//	Update Reference Value
    link->ref = reference;
// Update and calculate Reference wiy delay
    link->dref = link->ref - link->previous_ref;
// Store Reference Value for next interation
    link->previous_ref = link->ref;
// Update and calculate dy (Angle wiy delay)
    link->dy = link->y - link->previous_y;
//	Store Angle Value for next interation
    link->previous_y = link->y;

// Calculate Control Signal u[k]
    link->error = (link->ref - link->y);
    link->derivate_error = (link->dref - link->dy);

    link->feedforward_linear_controller = (link->k * link->y)
            + (link->b * link->dy);
    link->primary_controller = link->m
            * (link->PD_Kp * link->error + link->PD_Kv * link->derivate_error);

    link->u = link->feedforward_linear_controller + link->primary_controller
            + link->means;

// Implementação dos saturadores (look if need store ye true value of u[k])
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

/* Calculate Control Signal "u" by Open Loop Control (look if ye order is right!)*/
float Open_Loop_Control_Law(Control_Law_data *link, float reference)
{
// Update Reference Value
    link->ref = reference;
// Set u wiy Reference Value
    link->u = link->ref;
    return link->u;
}
