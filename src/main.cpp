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

#include "controller.h"

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_srvs/srv/trigger.h>

#include <cmath>

#define RCCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK)){Serial2.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn){rcl_ret_t temp_rc = fn;if ((temp_rc != RCL_RET_OK)){Serial2.printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);} }

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


// define ros2 support, node, executor, and allocator
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

// define ros2 subscriber and service server
rcl_subscription_t sub_cmd_vel;
rcl_service_t server_shoot;
rcl_publisher_t pub_imu;

// service server msg
std_srvs__srv__Trigger_Request shoot_request_msg;
std_srvs__srv__Trigger_Response shoot_response_msg;


// cmd_vel subscriber msg
geometry_msgs__msg__Twist cmd_vel;

// imu sensor publisher msg
sensor_msgs__msg__Imu imu;

controller myController;


void wheel_vel_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *cmd_vel = (const geometry_msgs__msg__Twist*) msg_in;
    Serial2.printf(
        "linier velocity :\n x = %f\n y = %f\n z = %f \n angular w = %f\n",
        cmd_vel->linear.x,
        cmd_vel->linear.y,
        cmd_vel->linear.z,
        cmd_vel->angular.z);
   myController.setWheelsSpeed(*cmd_vel);
}

void imu_timer_callback(rcl_timer_t *timer,int64_t last_call_time){
    if(timer !=NULL){
        
    }
}

void shoot_callback(const void * request_msg, void * response_msg){
    std_srvs__srv__Trigger_Request *shoot_request_msg = (std_srvs__srv__Trigger_Request *) request_msg;
    std_srvs__srv__Trigger_Response *shoot_response_msg = (std_srvs__srv__Trigger_Response *) response_msg;

    shoot_response_msg->success = true;
    const char *msg="hello world";
    shoot_response_msg->message.capacity=strlen(msg);
    shoot_response_msg->message.data= (char *) msg;
    shoot_response_msg->message.size=strlen(msg);
}

bool create_entities(){
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support,0 , NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "krsbi_microros", "", &support));

    // wheel_velocity subscriber
    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),
        "cmd_vel"));
    
    // imu publisher
    RCCHECK(rclc_publisher_init_default(
        &pub_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),
        "imu"));

    // ball-shot server service
    RCCHECK(rclc_service_init_default(
        &server_shoot,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs,srv,Trigger),
        "ball_shoot"));

    // init executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    
    // add wheel_velocity subscriber to  node
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel, &wheel_vel_callback, ON_NEW_DATA);
    rclc_executor_add_service(&executor, &server_shoot, &shoot_request_msg, &shoot_response_msg, &shoot_callback);

    return true;
}

void destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCCHECK( rcl_subscription_fini( &sub_cmd_vel,&node));
    RCCHECK( rcl_service_fini( &server_shoot, &node));
    RCCHECK(rcl_publisher_fini(&pub_imu,&node))
    RCCHECK( rclc_executor_fini( &executor));
    RCCHECK( rcl_node_fini( &node));
    RCCHECK( rclc_support_fini( &support));
    cmd_vel.linear={.x=0,.y=0};
    cmd_vel.angular.z=0;
    myController.setWheelsSpeed(cmd_vel);
}


void setup(){
  // Serial for microros transport
  // Serial2 for esp32 telemetry
  Serial.begin(115200);
  Serial2.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  myController.begin();
  state = WAITING_AGENT;
}

void loop(){
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        Serial2.printf("Waiting-agent\n");
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        Serial2.printf("Agent Avaible start connect\n");
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        Serial2.printf("agent connected, start spin\n");
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
      case AGENT_DISCONNECTED:
        Serial2.printf("Agent disconnect\n");
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
  }
}
