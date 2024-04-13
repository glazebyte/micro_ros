#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "motor.h"

// msg type
#include <geometry_msgs/msg/twist.h>
#include <omniwheels_interfaces/msg/wheels_velocity3.h>

#define RCCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK)){Serial2.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK)){Serial2.printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);} }


// define ros2 executor and support structure
rclc_executor_t executor;
rclc_support_t support;

// define ros2 memory alocator
rcl_allocator_t allocator;

// define ros2 node, subscriber, and publisher
rcl_node_t node;
rcl_subscription_t susbcriber;

// define motor device
motor myMotor;


void wheel_vel_callback(const void *msg_in)
{
    const omniwheels_interfaces__msg__WheelsVelocity3 *wheel_vel = (const omniwheels_interfaces__msg__WheelsVelocity3*) msg_in; 
    Serial2.printf("wheels velocity = l_motor : %f, r_motor: %f, b_motor: %f \n",wheel_vel->l_wheel,wheel_vel->r_wheel,wheel_vel->b_wheel);
    myMotor.setWheelsSpeed(*wheel_vel);
}

void kick_callback()
{
}


void setup()
{
    // Serial for microros transport
    // Serial2 for esp32 telemetry
    Serial.begin(115200);
    Serial2.begin(115200);

    myMotor.begin();

    // define subscribe message type
    omniwheels_interfaces__msg__WheelsVelocity3 wheels_vel;

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "krsbi_microros", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &susbcriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(omniwheels_interfaces,msg,WheelsVelocity3),
        "cmd_vel"));

    // init executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    

    rclc_executor_add_subscription(&executor, &susbcriber, &wheels_vel, &wheel_vel_callback, ON_NEW_DATA);
}

void loop()
{
    rclc_executor_spin_some(&executor,RCL_MS_TO_NS(100));
}
