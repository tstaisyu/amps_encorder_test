/* Copyright 2024 Taisyu Shibata
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROS_COMMUNICATIONS_H
#define ROS_COMMUNICATIONS_H

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
geometry_msgs__msg__Twist msg_pub;
rcl_publisher_t vel_publisher;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
//rcl_init_options_t init_options; // Humble
//size_t domain_id = 117;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Error in " #fn); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void logReceivedData(const geometry_msgs__msg__Twist *msg);
void updateDisplay(const geometry_msgs__msg__Twist *msg);
void initMotor(HardwareSerial& serial, byte motorID);

#endif