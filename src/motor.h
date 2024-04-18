#include <Adafruit_PWMServoDriver.h>
#include <geometry_msgs/msg/twist.h>

struct motor_pin
{
    int lpwm_pin,rpwm_pin;
};


class motor
{
private:
    Adafruit_PWMServoDriver *motor_dev = NULL;
    motor_pin motor_1 = {.lpwm_pin = 0, .rpwm_pin =1};
    motor_pin motor_2 = {.lpwm_pin = 4, .rpwm_pin =5};
    motor_pin motor_3 = {.lpwm_pin = 8, .rpwm_pin =9};
    // wheel_velocity wheels_vel;
public:
    bool begin();
    void setWheelsSpeed(geometry_msgs__msg__Twist cmd_vel);
    void setMotorRPM(motor_pin motor,float rpm);
};
