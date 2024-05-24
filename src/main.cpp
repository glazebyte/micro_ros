#include <Arduino.h>
#include <QuickPID.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "controller.h"
#include "sensor.h"

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_srvs/srv/trigger.h>
#include <std_msgs/msg/bool.h>
#include <omniwheels_interfaces/msg/encoder_pulse_stamped.h>
#include <micro_ros_utilities/string_utilities.h>

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


// define ros2 support, node, executor, allocator and timer
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_timer_t timer;

// define ros2 subscriber and service server
rcl_subscription_t sub_cmd_vel;
rcl_service_t server_shoot;
rcl_publisher_t pub_imu;
rcl_publisher_t pub_mag;
rcl_publisher_t pub_enc;
rcl_subscription_t sub_charge;

// service server msg
std_srvs__srv__Trigger_Request shoot_request_msg;
std_srvs__srv__Trigger_Response shoot_response_msg;


// cmd_vel subscriber msg
geometry_msgs__msg__Twist cmd_vel;

// imu sensor publisher msg
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
omniwheels_interfaces__msg__EncoderPulseStamped enc_msg;

//  charge subscbriber;
std_msgs__msg__Bool charge;

controller myController;
sensor mySensor;

// task handler for sensor loop
TaskHandle_t Task2;

SPIClass *spi = NULL;



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

void charge_callback(const void *msg_in){
  const std_msgs__msg__Bool *charge_bool = (const std_msgs__msg__Bool *) msg_in;
  myController.charge(charge_bool->data);
}

void publish_timer_callback(rcl_timer_t *timer,int64_t last_call_time){
    if(timer !=NULL){
        myController.update_msg(&enc_msg);
        RCCHECK(rcl_publish(&pub_imu,&imu_msg,NULL));
        RCCHECK(rcl_publish(&pub_mag,&mag_msg,NULL));
        RCCHECK(rcl_publish(&pub_enc,&enc_msg,NULL));
        Serial2.printf("callback\n");
    }
}

void shoot_callback(const void * request_msg, void * response_msg){
    std_srvs__srv__Trigger_Request *shoot_request_msg = (std_srvs__srv__Trigger_Request *) request_msg;
    std_srvs__srv__Trigger_Response *shoot_response_msg = (std_srvs__srv__Trigger_Response *) response_msg;

    myController.kick();
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
    
    // charging subscriber
    RCCHECK(rclc_subscription_init_default(
        &sub_charge,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),
        "charging"));
    
    // imu publisher
    RCCHECK(rclc_publisher_init_default(
        &pub_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),
        "imu/data_raw"));
      
    RCCHECK(rclc_publisher_init_default(
        &pub_mag,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,MagneticField),
        "imu/mag"));
    // encoder position publisher
    RCCHECK(rclc_publisher_init_default(
        &pub_enc,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(omniwheels_interfaces,msg,EncoderPulseStamped),
        "encoder_pulse"));
    
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(10),
      publish_timer_callback
    ));

    // ball-shot server service
    RCCHECK(rclc_service_init_default(
        &server_shoot,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs,srv,Trigger),
        "ball_shoot"));

    // init executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    // add wheel_velocity subscriber to  node
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel, &wheel_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor,&sub_charge,&charge,&charge_callback,ON_NEW_DATA);
    rclc_executor_add_timer(&executor,&timer);
    rclc_executor_add_service(&executor, &server_shoot, &shoot_request_msg, &shoot_response_msg, &shoot_callback);

    return true;
}

void destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCCHECK( rcl_subscription_fini( &sub_cmd_vel,&node));
    RCCHECK( rcl_service_fini( &server_shoot, &node));
    RCCHECK( rcl_subscription_fini(&sub_charge,&node));
    RCCHECK( rcl_timer_fini(&timer));
    RCCHECK( rcl_publisher_fini(&pub_imu,&node));
    RCCHECK( rcl_publisher_fini(&pub_mag,&node));
    RCCHECK( rcl_publisher_fini(&pub_enc,&node));
    RCCHECK( rclc_executor_fini( &executor));
    RCCHECK( rcl_node_fini( &node));
    RCCHECK( rclc_support_fini( &support));
    myController.reset();
}

void task_loop_2( void * pvParameters ){
  // setup
  // myController.encoder_begin();
  Wire.begin(static_cast<int> (SDA),static_cast<int> (SCL),400000);
  mySensor.begin();



  char *frame_str = "imu";
  rosidl_runtime_c__String frame;
  frame.capacity = 4;
  frame.data = frame_str;
  frame.size = strlen(frame_str);


  imu_msg.header.frame_id=frame;
  mag_msg.header.frame_id=frame;
  
  
  // loop
  for (;;)
  {
    mySensor.imu_loop(&imu_msg,&mag_msg);
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}


void setup(){
  // Serial for microros transport
  // Serial2 for esp32 telemetry
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.setDebugOutput(false);
  Serial2.setDebugOutput(true);

  Wire1.begin(25,26,400000);
  set_microros_serial_transports(Serial);

  delay(2000);

  myController.begin();
  
  xTaskCreatePinnedToCore(task_loop_2,"task_loop2",10000,NULL,7,&Task2,0);


  char *enc_frame_str = "encoder";
  rosidl_runtime_c__String enc_frame;
  enc_frame.capacity = 7;
  enc_frame.data = enc_frame_str;
  enc_frame.size = strlen(enc_frame_str);

  enc_msg.header.frame_id=enc_frame;

  myController.encoder_begin();
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
