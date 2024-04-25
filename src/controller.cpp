#include "controller.h"
#include "cmath"

bool controller::begin(){
    if (controller_dev)
        delete controller_dev;
    controller_dev = new Adafruit_PWMServoDriver();
    controller_dev->begin();
    controller_dev->setPWMFreq(1600);

    addMotorEncodeer(motor_1);
    addMotorEncodeer(motor_2);
    addMotorEncodeer(motor_3);

    xTaskCreate(&read_encoder_task, "read_encoder_motor1", 2048, &motor_1, 1, NULL);
    xTaskCreate(&read_encoder_task, "read_encoder_motor2", 2048, &motor_2, 1, NULL);
    xTaskCreate(&read_encoder_task, "read_encoder_motor3", 2048, &motor_3, 1, NULL);

    return true;
}

void controller::setWheelsSpeed(geometry_msgs__msg__Twist cmd_vel){
    double v = cmd_vel.linear.x;
    double vn = cmd_vel.linear.y;
    double w = cmd_vel.angular.z;
    // center to wheel radius 16 cm = 0.16m
    // gamma 30 degree or 0.5235987756 rads
    double L = 0.16;
    float gamma = 30 * (180 / M_PI);
    float wheel_1 = (w * L - vn);
    float wheel_2 = (w * L) + (v * cos(gamma) + vn * sin(gamma));
    float wheel_3 = (w * L) + (-v * cos(gamma) + vn * sin(gamma));
    Serial2.printf("wheel_1 speed = %f\n", wheel_1);
    Serial2.printf("wheel_2 speed = %f\n", wheel_2);
    Serial2.printf("wheel_3 speed = %f\n", wheel_3);
    setMotorRPM(this->motor_1, wheel_1);
    setMotorRPM(this->motor_2, wheel_2);
    setMotorRPM(this->motor_3, wheel_3);
}

void controller::setMotorRPM(motor motor_dev, float rpm){
    rpm = rpm * 4096 / 3.46;
    Serial2.printf("convert rpm to pwm = %f\n", rpm);
    int pwm = abs(rpm);
    Serial2.printf("pwm power = %d\n", pwm);
    if (rpm < 0){
        controller_dev->setPin(motor_dev.rpwm_pin, 0);
        controller_dev->setPin(motor_dev.lpwm_pin, pwm);
    }
    else{
        controller_dev->setPin(motor_dev.lpwm_pin, 0);
        controller_dev->setPin(motor_dev.rpwm_pin, pwm);
    }
}

void controller::addMotorEncodeer(motor motor_dev){
    pinMode(motor_dev.encodera_pin, INPUT_PULLUP);
    pinMode(motor_dev.encoderb_pin, INPUT_PULLUP);
    motor_dev.prev_milis = millis();
    attachInterruptArg(motor_dev.encodera_pin, motor_interupt_isr, &motor_dev, RISING);
}
void controller::kick(){
    controller_dev->setPin(solenoid.kick_pin,4096,true);
    vTaskDelay(500/portTICK_RATE_MS);
    controller_dev->setPin(solenoid.kick_pin,0,true);
}
void controller::charge(bool charge_in){
    controller_dev->setPin(solenoid.charge_pin,4096,charge_in);
}


void motor_interupt_isr(void *motot_arg){
    motor *motor_dev = (motor *)motot_arg;
    int enc_b = digitalRead(motor_dev->encoderb_pin);
    if (enc_b > 0){
        if (motor_dev->forward){
            motor_dev->pulse += 1;
        }
        else{
            motor_dev->forward = true;
            motor_dev->pulse = 1;
            motor_dev->prev_milis = millis();
        }
    }
    else{
        if (motor_dev->forward){
            motor_dev->forward = false;
            motor_dev->pulse = -1;
            motor_dev->prev_milis = millis();
        }
        else{
            motor_dev->pulse -= 1;
        }
    }
}

void read_encoder_task(void *pvParameter)
{
    motor *motor_dev = (motor *)pvParameter;
    motor_dev->cur_milis = millis();
    long interval = motor_dev->cur_milis - motor_dev->prev_milis;
    int current_rpm = (float)(motor_dev->pulse / 7) * (1000000 / interval) * 60;
    motor_dev->prev_milis = millis();
    motor_dev->encoder_rpm = current_rpm;
    vTaskDelay(1000 / portTICK_RATE_MS);
}