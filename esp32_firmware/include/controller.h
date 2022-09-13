// controller.h

#define ROBOT_LENGTH 1

/* lineal controller variables */
namespace controller_lineal
{
    float kp = 1;
    float error = 0.0;
    float u_p = 0.0;
    float velocity_right = 0.0;
    float velocity_left = 0.0;
}

/* Y controller variables */
namespace controller_theta
{
    float kp = 1;
    float u_p = 0.0;
    float error = 0.0;
}
