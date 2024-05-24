#include "controller.h"
#include "cmath"

bool controller::begin(){
    if (controller_dev)
        delete controller_dev;
    controller_dev = new Adafruit_PWMServoDriver(0x40,Wire1);
    controller_dev->begin();
    controller_dev->setPWMFreq(1600);
    
    controller_dev->setPin(solenoid.charge_pin,0);
    controller_dev->setPin(solenoid.kick_pin,0);

    return true;
}

void controller::encoder_begin(){
    addMotorEncodeer(&motor_1);
    addMotorEncodeer(&motor_2);
    addMotorEncodeer(&motor_3);
    prev_millis=0;

    if(pid_1)
        delete pid_1;
    if(pid_2)
        delete pid_2;
    if(pid_3)
        delete pid_3;
    
    pid_1 = new QuickPID(
        &motor_1.input, &motor_1.output, &motor_1.setpoint,
        motor_1.Kp,motor_1.Ki,motor_1.Kd,
        QuickPID::Action::direct);
    pid_2 = new QuickPID(
        &motor_2.input, &motor_2.output, &motor_2.setpoint,
        motor_2.Kp,motor_2.Ki,motor_2.Kd,
        QuickPID::Action::direct);
    pid_3 = new QuickPID(
        &motor_3.input, &motor_3.output, &motor_3.setpoint,
        motor_3.Kp,motor_3.Ki,motor_3.Kd,
        QuickPID::Action::direct);

    // xTaskCreatePinnedToCore(this->pid_task,"pid_task",10000,this,9,NULL,0);
}

void controller::setWheelsSpeed(geometry_msgs__msg__Twist cmd_vel){
    // cmd vel in m/s
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y ;
    double w = cmd_vel.angular.z;
    // center to wheel radius 16 cm = 0.16m
    // gamma 30 degree or 0.5235987756 rads
    double L = 0.16;
    float wheel_1 = ((sqrtf(3)/3)*vx) - (vy/3) + (w/3);
    float wheel_2 = -((sqrtf(3)/3)*vx) - (vy/3) + (w/3);
    float wheel_3 = (vy*2/3) + (w/3);

    Serial2.printf("wheel_1 speed = %f\n", wheel_1);
    Serial2.printf("wheel_2 speed = %f\n", wheel_2);
    Serial2.printf("wheel_3 speed = %f\n", wheel_3);
    
    // setpoint in tickperframe or pps (pulse per second) 
    // ppr is pulse per rotate 
    // wheel diameter is 82mm, so circumference is 0.02576106 Meter
    // so vel(m/s) to pps(hz) is 
    // motor_1.setpoint = ( wheel_1 / 0.02576106 ) * motor_1.ppr;
    // motor_2.setpoint = ( wheel_2 / 0.02576106 ) * motor_2.ppr;
    // motor_3.setpoint = ( wheel_3 / 0.02576106 ) * motor_3.ppr;

    motor_1.output = wheel_1;
    motor_2.output = wheel_2;
    motor_3.output = wheel_3;
    
    driveMotor(motor_1);
    driveMotor(motor_2);
    driveMotor(motor_3);
}

void controller::update_msg(omniwheels_interfaces__msg__EncoderPulseStamped *msg_out){
    msg_out->encoder.wheel1 = motor_1.pulse;
    msg_out->encoder.wheel2 = motor_2.pulse;
    msg_out->encoder.wheel3 = motor_3.pulse;
    uint32_t current_time_ms = millis();
    msg_out->header.stamp.sec = current_time_ms/1000;
    msg_out->header.stamp.nanosec = (current_time_ms % 1000) * 1000000;
}

void controller::driveMotor(motor motor_dev){
    float rpm = motor_dev.output * 4096;
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

void controller::addMotorEncodeer(motor *motor_dev){
    pinMode(motor_dev->encodera_pin, INPUT_PULLUP);
    pinMode(motor_dev->encoderb_pin, INPUT_PULLUP);
    attachInterruptArg(motor_dev->encodera_pin, motor_interupt_isr, motor_dev, CHANGE);
    attachInterruptArg(motor_dev->encoderb_pin, motor_interupt_isr, motor_dev, CHANGE);
}
void controller::kick(){
    controller_dev->setPin(solenoid.kick_pin,4096);
    vTaskDelay(500/portTICK_RATE_MS);
    controller_dev->setPin(solenoid.kick_pin,0);
}
void controller::charge(bool charge_in){
    if(charge_in)
        controller_dev->setPin(solenoid.charge_pin,4096);
    else
        controller_dev->setPin(solenoid.charge_pin,0);
}

void controller::reset(){
    // set motor to off
    motor_1.setpoint =0;
    motor_2.setpoint =0;
    motor_3.setpoint =0;

    // set charger and  kicker to off
    // true is invert
    controller_dev->setPin(solenoid.kick_pin,0);
    controller_dev->setPin(solenoid.charge_pin,0);
}


void IRAM_ATTR controller::motor_interupt_isr(void *motot_arg){
    motor *motor_dev = (motor *)motot_arg;
    motor_dev->currentStateCLK = digitalRead(motor_dev->encodera_pin);
    if(motor_dev->currentStateCLK != motor_dev->lastStateCLK && motor_dev->currentStateCLK ==1 ){
      if (digitalRead(motor_dev->encoderb_pin) != motor_dev->currentStateCLK)
        motor_dev->pulse -=1;
      else
        motor_dev->pulse +=1;
    }
    motor_dev->lastStateCLK = motor_dev->currentStateCLK;
}
void controller::pid_task(void *pvParameter){
    controller *myController = (controller *) pvParameter;
    for(;;){
        motor *motor_1= &myController->motor_1;
        motor *motor_2= &myController->motor_2;
        motor *motor_3= &myController->motor_3;

        myController->cur_millis=millis();
        long delta_t = myController->cur_millis - myController->prev_millis;
        Serial2.printf("delta_t %ld",delta_t);
        motor_1->input=(motor_1->pulse - motor_1->prev_pulse)*(1000/delta_t);
        motor_2->input=(motor_2->pulse - motor_2->prev_pulse)*(1000/delta_t);
        motor_3->input=(motor_3->pulse - motor_3->prev_pulse)*(1000/delta_t);
        myController->pid_1->Compute();
        myController->pid_2->Compute();
        myController->pid_3->Compute();
        myController->prev_millis = myController->cur_millis;

        myController->driveMotor(*motor_1);
        myController->driveMotor(*motor_2);
        myController->driveMotor(*motor_3);

        Serial2.printf(">setpoint_1:%f\n",motor_1->setpoint);
        Serial2.printf(">current_1:%f\n",motor_1->input);
        Serial2.printf(">error_1:%f\n",motor_1->setpoint - motor_1->input);
        Serial2.printf(">output:%f\n",motor_1->output);

        Serial2.printf(">setpoint_2:%f\n",motor_2->setpoint);
        Serial2.printf(">current_2:%f\n",motor_2->input);
        Serial2.printf(">error_2:%f\n",motor_2->setpoint - motor_2->input);
        Serial2.printf(">output:%f\n",motor_2->output);

        Serial2.printf(">setpoint_3:%f\n",motor_3->setpoint);
        Serial2.printf(">current_3:%f\n",motor_3->input);
        Serial2.printf(">error_3:%f\n",motor_3->setpoint - motor_3->input);
        Serial2.printf(">output:%f\n",motor_3->output);

        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}