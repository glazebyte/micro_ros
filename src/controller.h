#include <Adafruit_PWMServoDriver.h>
#include <geometry_msgs/msg/twist.h>
struct motor
{
    const int lpwm_pin, rpwm_pin, ppr, encodera_pin, encoderb_pin;
    int pulse, encoder_rpm;
    long cur_milis, prev_milis;
    bool forward;
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
    motor motor_1={
        .lpwm_pin=0,
        .rpwm_pin=1,
        .ppr=7,
        .encodera_pin=NULL,
        .encoderb_pin=NULL
        };
    motor motor_2={
        .lpwm_pin=2,
        .rpwm_pin=3,
        .ppr=7,
        .encodera_pin=NULL,
        .encoderb_pin=NULL
        };
    motor motor_3={
        .lpwm_pin=4,
        .rpwm_pin=5,
        .ppr=7,
        .encodera_pin=NULL,
        .encoderb_pin=NULL
        };
    kicker solenoid = {.kick_pin = 6, .charge_pin = 7};
public:
    bool begin();
    void setWheelsSpeed(geometry_msgs__msg__Twist cmd_vel);
    void setMotorRPM(motor motor_dev,float rpm);
    void addMotorEncodeer(motor motor_dev);
    void readMotorEncoder(motor motor_dev);
    void kick();
    void charge(bool charge_in);
};

void motor_interupt_isr(void *motor_arg);
void read_encoder_task(void *pvParameter);
