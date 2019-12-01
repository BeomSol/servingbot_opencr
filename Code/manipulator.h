/*******************************************************************************
  Copyright 2018 SEOULTECH CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Donghyun Ko */

#ifndef MANIPULATOR_CONFIG_H_
#define MANIPULATOR_CONFIG_H_

// include msgs
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>


#include <AccelStepper.h>
#include <MultiStepper.h>

HardwareTimer Timer(TIMER_CH1);
#define CONTROL_TIME 10
#define DOF 4  //4

#define STEPPER1 0
#define STEPPER2 1
#define STEPPER3 2
#define STEPPER4 3

#define STEPPER1_STEP_PIN 21   
#define STEPPER1_DIR_PIN 20    
#define STEPPER2_STEP_PIN 14   
#define STEPPER2_DIR_PIN 15    
#define STEPPER3_STEP_PIN 19  
#define STEPPER3_DIR_PIN 18   
#define STEPPER4_STEP_PIN 17  
#define STEPPER4_DIR_PIN 16   

#define GRIPPER_PWM 5
#define GRIPPER_DIR 6

#define OFF 0
#define ON 1

/*******************************************************************************
* Callback function prototypes
*******************************************************************************/
void goalJointPositionCallback(const sensor_msgs::JointState& goal_joint_position_msg);
void gripper_onCallback( const std_msgs::Int32& gripper_on_msg);
void gripper_offCallback( const std_msgs::Int32& gripper_off_msg);

void motor1_first_postureCallback( const std_msgs::Int32& motor1_first_posture_msg);
void all_motor_first_postureCallback( const std_msgs::Int32& all_motor_first_posture_msg);

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void timer_setup();
void stepper_setup();
void photo_setup();
void gripper_setup();
void first_posture_process();
void control_stepper();
void control_gripper();
void set_profile();

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/


/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<sensor_msgs::JointState> joint_position_sub("move_group/fake_controller_joint_states", goalJointPositionCallback);
ros::Subscriber<std_msgs::Int32> gripper_on_sub("gripper_on", gripper_onCallback );
ros::Subscriber<std_msgs::Int32> gripper_off_sub("gripper_off", gripper_offCallback );

ros::Subscriber<std_msgs::Int32> motor1_first_posture_sub("motor1_first_posture", motor1_first_postureCallback );
ros::Subscriber<std_msgs::Int32> all_motor_first_posture_sub("all_motor_first_posture", all_motor_first_postureCallback );

/*******************************************************************************
* Publisher
*******************************************************************************/
std_msgs::Int32 reached_msg;
ros::Publisher reached_pub("reached", &reached_msg);

/*******************************************************************************
* For debugging
*******************************************************************************/ 
std_msgs::Int32 position_stepper1_msg;
ros::Publisher position_stepper1_pub("position_stepper1", &position_stepper1_msg);
std_msgs::Int32 position_stepper2_msg;
ros::Publisher position_stepper2_pub("position_stepper2", &position_stepper2_msg);
std_msgs::Int32 position_stepper3_msg;
ros::Publisher position_stepper3_pub("position_stepper3", &position_stepper3_msg);
std_msgs::Int32 position_stepper4_msg;
ros::Publisher position_stepper4_pub("position_stepper4", &position_stepper4_msg);


/*******************************************************************************
* Declaration for manipulation
*******************************************************************************/

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER3_STEP_PIN, STEPPER3_DIR_PIN);
AccelStepper stepper4(AccelStepper::DRIVER, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);

MultiStepper steppers;

//Motor variable
float max_speed[DOF] = {400, 800, 800, 800}; //400 800 800 800
float acceleration[DOF] = {0, 0, 0, 0};
float speed[DOF] = {0, 0, 0, 0};
int resolution[DOF] = {2, 1, 1, 1};
int gear_ratio[DOF] = {4, 48, 50, 38};

long goal_position[DOF] = {0, 0, 0, 0};
long prev_goal_position[DOF] = {0, 0, 0, 0};
long step_goal_position[DOF] = {0, 0, 0, 0};

float start_speed[DOF] = {-100, -400, 600, 800};

long finish_position[DOF] = {500, 7000, 5000, 4000};
float finish_max_speed[DOF] = {400, 800, 800, 800};
float finish_acceleration[DOF] = {400, 800, 800, 800};

unsigned long prev_time = 0;
unsigned long time_now  = 0;
float step_time = 0; // dimension = [msec]
float step_time_weight = 0.0014; //8 * 0.0002616667 = 0.0021 -> 현재 1.5배 정도 빠르게 0.0014

//start_flag
int start_flag = 0;
int finish_flag = 0;
int goal_flag = 0;

//photo sensor
int photo_pin1 = BDPIN_GPIO_11;
int photo_pin2 = BDPIN_GPIO_13;
int photo_pin3 = BDPIN_GPIO_15;
int photo_pin4 = BDPIN_GPIO_17;

int photo_val1;
int photo_val2;
int photo_val3;
int photo_val4;

//first posture flag
bool first_posture1 = false;
bool first_posture2 = false;
bool first_posture3 = false;
bool first_posture4 = false;

//gripper
int gripper_flag = 0;
int gripper_onoff;

int motor1_error = 0;
int all_motor_error = 0;


#endif //MANIPULATOR_CONFIG_H_
