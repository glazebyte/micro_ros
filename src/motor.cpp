#include "motor.h"
#include "mat.h"

bool motor::begin(){
    if (motor_dev)
        delete motor_dev;
    motor_dev = new Adafruit_PWMServoDriver();
    motor_dev->begin();
    motor_dev->setPWMFreq(1600);

    return true;
}

void motor::setWheelsSpeed(geometry_msgs__msg__Twist cmd_vel){
    double v = cmd_vel.linear.x;
    double vn = cmd_vel.linear.y;
    double w = cmd_vel.angular.z; 
    // center to wheel radius 16 cm = 0.16m
    // gamma 30 degree or 0.5235987756 rads
    double L =0.16;
    float gamma = 0.5235987756;
    float wheel_1 = ( w * L - vn ); 
    float wheel_2 = ( w * L) + (v*cos(gamma)+vn*sin(gamma));
    float wheel_3 = ( w * L) + (-v*cos(gamma)+vn*sin(gamma));
    Serial2.printf("wheel_1 speed = %f\n",wheel_1);
    Serial2.printf("wheel_2 speed = %f\n",wheel_2);
    Serial2.printf("wheel_3 speed = %f\n",wheel_3);
    setMotorRPM(this->motor_1,wheel_1);
    setMotorRPM(this->motor_2,wheel_2);
    setMotorRPM(this->motor_3,wheel_3);
}

void motor::setMotorRPM(motor_pin motor, float rpm){
    rpm = rpm * 4096 / 3.46;
    Serial2.printf("convert rpm to pwm = %f\n",rpm);
    int pwm = abs(rpm);
    Serial2.printf("pwm power = %d\n",pwm);
    if(rpm<0){
        motor_dev->setPin(motor.rpwm_pin,0);
        motor_dev->setPin(motor.lpwm_pin,pwm);
    }else{
        motor_dev->setPin(motor.lpwm_pin,0);
        motor_dev->setPin(motor.rpwm_pin,pwm);
    }
}