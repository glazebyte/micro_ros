#include <Arduino.h>

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

#define RCCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK)){Serial2.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK))   {  Serial2.printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);} }

// define ros2 executor and support structure
rclc_executor_t executor;
rclc_support_t support;

// define ros2 memory alocator
rcl_allocator_t allocator;

// define ros2 node, subscriber, and publisher
rcl_node_t node;
rcl_subscription_t susbcriber;

// define subscribe message type
geometry_msgs__msg__Twist cmd_vel;

void print_hello(void *pvParameter)
{
    Serial.println("Hello World");
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void twist_callback(const void *cmd_vel)
{
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
    xTaskCreate(&print_hello, "print_hello", 2048, NULL, 1, NULL);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "krsbi_microros", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &susbcriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // init executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rclc_executor_add_subscription(&executor, &susbcriber, &cmd_vel, &twist_callback, ON_NEW_DATA);
}

void loop()
{
    rclc_executor_spin_some(&executor,RCL_MS_TO_NS(100));
}
