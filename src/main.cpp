#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    mpu.getEvent(&a, &g, &temp);

    msg.angular_velocity.x=g.gyro.x;
    msg.angular_velocity.y=g.gyro.y;
    msg.angular_velocity.z=g.gyro.z;

    msg.linear_acceleration.x=a.acceleration.x;
    msg.linear_acceleration.y=a.acceleration.y;
    msg.linear_acceleration.z=a.acceleration.z;

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    //msg.data++;
  }
}

void setup() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  IPAddress agent_ip(192, 168, 1, 18);
  size_t agent_port = 8888;

  char ssid[] = "ssid";
  char psk[]= "haslo";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "hal_imu", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "hal_imu"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.angular_velocity.x=0;
    msg.angular_velocity.y=0;
    msg.angular_velocity.z=0;

    for (int i = 0; i < 9; i++)
    {
        msg.angular_velocity_covariance[i]=0;
    }

    msg.linear_acceleration.x=0;
    msg.linear_acceleration.y=0;
    msg.linear_acceleration.z=0;

    for (int i = 0; i < 9; i++)
    {
        msg.linear_acceleration_covariance[i]=0;
    }
    
    msg.orientation.w=0;
    msg.orientation.x=0;
    msg.orientation.y=0;
    msg.orientation.z=0;
    msg.orientation_covariance[0]=-1;

    for (int i = 1; i < 9; i++)
    {
        msg.orientation_covariance[i]=0;
    }
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}