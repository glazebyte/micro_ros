#include <Adafruit_PWMServoDriver.h>
#include <geometry_msgs/msg/twist.h>
#include <omniwheels_interfaces/msg/encoder_pulse_stamped.h>
#include <freertos/task.h>
#include <QuickPID.h>

struct motor
{
    const int lpwm_pin, rpwm_pin, encodera_pin, encoderb_pin;
    int encoder_rpm, lastStateCLK, currentStateCLK;
    long pulse, prev_pulse;
    float input,output,setpoint,Kp,Ki,Kd,ppr;
};
struct kicker
{
    const int kick_pin, charge_pin;
    int capacity;
};



class controller
{
private:
    Adafruit_PWMServoDriver * controller_dev;
    long cur_millis, prev_millis;
    motor motor_1={
        .lpwm_pin=0,
        .rpwm_pin=1,
        .encodera_pin=33,
        .encoderb_pin=32,
        .pulse = 0,
        .prev_pulse = 0,
        .Kp=1,
        .Ki=0,
        .Kd=0,
        .ppr=134.4
    };
    motor motor_2={
        .lpwm_pin=2,
        .rpwm_pin=3,
        .encodera_pin=35,
        .encoderb_pin=34,
        .pulse = 0,
        .prev_pulse = 0,
        .Kp=1,
        .Ki=0,
        .Kd=0,
        .ppr=134.4
    };
    motor motor_3={
        .lpwm_pin=4,
        .rpwm_pin=5,
        .encodera_pin=39,
        .encoderb_pin=36,
        .pulse = 0,
        .prev_pulse = 0,
        .Kp=1,
        .Ki=0,
        .Kd=0,
        .ppr=134.4
    };
    kicker solenoid = {.kick_pin = 6, .charge_pin = 7};
    QuickPID *pid_1;
    QuickPID *pid_2;
    QuickPID *pid_3;
public:
    bool begin();
    void encoder_begin();
    void update_msg(omniwheels_interfaces__msg__EncoderPulseStamped *msg_out);
    void setWheelsSpeed(geometry_msgs__msg__Twist cmd_vel);
    void driveMotor(motor motor_dev);
    void addMotorEncodeer(motor *motor_dev);
    void readMotorEncoder(motor motor_dev);
    void kick();
    void charge(bool charge_in);
    void reset();
    static void motor_interupt_isr(void *motor_arg);
    static void pid_task(void *pvParameter);
};