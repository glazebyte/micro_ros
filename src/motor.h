#include <Adafruit_PWMServoDriver.h>
#include <omniwheels_interfaces/msg/wheels_velocity3.h>

struct motor_pin
{
    int lpwm_pin,rpwm_pin;
};


class motor
{
private:
    Adafruit_PWMServoDriver *motor_dev = NULL;
    motor_pin l_motor = {.lpwm_pin = 0, .rpwm_pin =1};
    motor_pin r_motor = {.lpwm_pin = 4, .rpwm_pin =5};
    motor_pin b_motor = {.lpwm_pin = 8, .rpwm_pin =9};
    // wheel_velocity wheels_vel;
public:
    bool begin();
    void setWheelsSpeed(omniwheels_interfaces__msg__WheelsVelocity3 wheels_vel);
    void setMotorRPM(motor_pin motor,int rpm);
};
