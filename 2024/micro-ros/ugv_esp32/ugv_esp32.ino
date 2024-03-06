/*
UGV teleop motor controller using the ESP32 platform and Flysky remote controller

Dependencies:
- PID library

Last edited by Brendan O'Malley 13/08/23
Changelog 13/08/23:
- Changed filename
- Refactoring variables for readability
- Changing previous hardcoded values to pre-defined macros
- Formatting
- Commenting
- Extracted functions for setting reverse and break pins for readability

14/08/23:
- RPM is calculated during the main loop
- encoder ISRs only increment the pulse counters and detect if motor is in reverse
*/

#include <Arduino.h>

#include <Wire.h>
#include <TFMPI2C.h> // TFmini plus api

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Input Pin Definitions
const int FLYSKY_LEFT_PIN = 23;  // Left Flysky Stick (CH3)
const int FLYSKY_RIGHT_PIN = 22; // Right Flysky Stick (CH2)
const int FLYSKY_SWB_PIN = 16;   // Left three-way switch (CH7)

const int I2C_SDA_PIN = 21; // I2C Data bus pin
const int I2C_SCL_PIN = 19; // I2C Clock bus pin

const int ENCODER_LEFT_A_PIN = 5;   // Left Rotary Encoder phase A pin
const int ENCODER_LEFT_B_PIN = 17;  // Left Rotary Encoder phase B pin
const int ENCODER_RIGHT_A_PIN = 2;  // Right Rotary Encoder Phase A pin
const int ENCODER_RIGHT_B_PIN = 15; // Right Rotary Encoder Phase B pin

// Output Pin Definitions
const int MOTOR_LEFT_DAC_PIN = 25;    // Left motor controller DAC output pin
const int MOTOR_RIGHT_DAC_PIN = 26;   // Right motor controller DAC output pin
const int MOTOR_LEFT_REV_PIN = 32;    // Left motor reverse pin
const int MOTOR_RIGHT_REV_PIN = 33;   // Right motor reverse pin
const int MOTOR_LEFT_BRAKE_PIN = 27;  // Left motor brake pin
const int MOTOR_RIGHT_BRAKE_PIN = 14; // Right motor brake pin

const double DEADZONE_VALUE = 2.0;     // Flysky stick deadzone value
const int PID_CONS_AGG_THRESHOLD = 10; // Use aggressive tuning parameters if above this value, otherwise use conservative parameters
const double PULSES_PER_REV = 2560.0;  // Pulses per revolution. Actual PPR for the encoder is 1024.
                                       // A gear step up of 0.4 gives us 1024 / 0.4 = 2560
const double RPM_MAX = 360.0;          // Maximum achievable speed of the motors
const double RPM_LIMIT_LOW = 20.0;
const double RPM_LIMIT_MEDIUM = 60.0;
const double RPM_LIMIT_HIGH = 150.0;

const int TIMER_PRESCALER = 80;             // Prescaler to set period of timer using the default clock of 80Mhz (APB_CLK)
const int TIMER_TICKS_PER_MS = 1000;        // Timer ticks for every millisecond
const double MOTOR_SPEED_SAMPLE_RATE = 5.0; // RPM calculation frequency (Hz)

const bool PROXIMITY_ENABLE = true;
const int TFMP_FRONT_ADDR = 16;
const int TFMP_REAR_ADDR = 17;
const int PROXIMITY_LIMIT = 30; // Centimeters. Stop UGV if sensors detect when distance is less than this value

// Integers to represent values from flysky sticks and pots
double flysky_value_left;
double flysky_value_right;
int flysky_speed_limit_switch; // Value is 1, 2 or 3 for low, medium and high max speeds

// Encoder pulse counters
volatile unsigned int pulse_counter_left = 0;
volatile unsigned int pulse_counter_right = 0;

// Flag when motor is turning in reverse
volatile bool is_motor_left_reversing = false;
volatile bool is_motor_right_reversing = false;

bool motor_left_enable_brake = false;
bool motor_right_enable_brake = false;

// Flags for when the UGV is in proximity with an object
volatile bool proximity_limit_front_flag = 0;
volatile bool proximity_limit_rear_flag = 0;

double rpm_limit; // Rpm limit set by the speed switch value

// rpm used to calculate rpm as the function in the loop
double rpm_left;
double rpm_right;

double rpm_target_L;
double rpm_target_R;

// DAC values. Range from 0-255
int dac_left;
int dac_right;

// variable to store the encoder readings
int encoder_left_B_value;
int encoder_right_B_value;

// Initialise timer
hw_timer_t *timer = NULL;

// Declare TFMPI2C API instance
TFMPI2C tfmp;

int16_t tf_front_dist = 300;
int16_t tf_rear_dist = 300;
bool prox_limit_front_flag = false;
bool prox_limit_rear_flag = false;

// Declare microros variables
rcl_publisher_t range_f_pub;
rcl_publisher_t range_r_pub;
rcl_subscription_t cmd_vel_left_sub;
rcl_subscription_t cmd_vel_right_sub;
sensor_msgs__msg__Range range_f_msg;
sensor_msgs__msg__Range range_r_msg;
geometry_msgs__msg__Twist cmd_vel_left_msg;
geometry_msgs__msg__Twist cmd_vel_right_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Read the number of a specified channel and convert to the range provided.
// The output is a throttle percentage, negative is reverse.
double readFlyskyPWM(int pin)
{
  int ch = pulseIn(pin, HIGH, 30000); // Read PWM duty cycle which is between 1000 - 2000 for the flysky remote
  if (ch < 700)                       // If remote or receiver is turned off, return 1500 (idle position)
    return 1500.0;
  return (double)map(ch, 1000, 2000, -100, 100); // Scale duty cycle
}

double setRPMLimit()
{
  int channel_duty = pulseIn(FLYSKY_SWB_PIN, HIGH, 30000);
  if (channel_duty > 750 && channel_duty < 1250)
  {
    return RPM_LIMIT_LOW;
  }
  else if (channel_duty < 1750)
  {
    return RPM_LIMIT_MEDIUM;
  }
  else if (channel_duty < 2250)
  {
    return RPM_LIMIT_HIGH;
  }
  else
  {
    return RPM_LIMIT_LOW; // If channel is disconnected, default to low speed.
  }
}

// Increment counter and determine motor direction using Phase B.
// Triggered at each rising edge of Phase A.
void encoderLeftISR()
{
  pulse_counter_left++;
  encoder_left_B_value = digitalRead(ENCODER_LEFT_B_PIN); // Read B phase encoder value on left side
  is_motor_left_reversing = encoder_left_B_value;         // If B phase is HIGH when A phase is HIGH, then motor is rotating in reverse
}

void encoderRightISR()
{
  pulse_counter_right++;
  encoder_right_B_value = digitalRead(ENCODER_RIGHT_B_PIN);
  is_motor_right_reversing = !encoder_right_B_value; // If B phase is LOW when A phase is HIGH, then motor is rotating in reverse
}

// Timer callback function to calculate RPM
void IRAM_ATTR getRPMISR()
{
  // Calculate rpm
  rpm_left = (MOTOR_SPEED_SAMPLE_RATE * pulse_counter_left * 60.0) / PULSES_PER_REV;
  rpm_right = (MOTOR_SPEED_SAMPLE_RATE * pulse_counter_right * 60.0) / PULSES_PER_REV;

  // Set RPS value negative if rotating in reverse
  if (is_motor_left_reversing)
  {
    rpm_left *= -1.0;
  }
  if (is_motor_right_reversing)
  {
    rpm_right *= -1.0;
  }

  // Reset encoder pulse counter
  pulse_counter_left = 0;
  pulse_counter_right = 0; 
}

void cmd_vel_left_callback(const void *msgin)
{
  // const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // rpm_target_L = msg->linear.x * rpm_limit;
}

void cmd_vel_right_callback(const void *msgin)
{
  // const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // rpm_target_R = msg->linear.x * rpm_limit;
}

void initROS()
{
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &range_f_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "tof/front"));

  RCCHECK(rclc_publisher_init_default(
    &range_r_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "tof/rear"));

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_left_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/left"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_right_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel/right"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_left_sub, &cmd_vel_left_msg, &cmd_vel_left_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_right_sub, &cmd_vel_right_msg, &cmd_vel_right_callback, ON_NEW_DATA));
}

void setup()
{
  // Set up serial monitor and has to be 115200 for the esp32
  Serial.begin(115200);
  delay(20);

  initROS();

  // Initialise input and output pins
  pinMode(FLYSKY_LEFT_PIN, INPUT);
  pinMode(FLYSKY_RIGHT_PIN, INPUT);
  pinMode(FLYSKY_SWB_PIN, INPUT);
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  pinMode(ENCODER_LEFT_B_PIN, INPUT);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT);
  pinMode(ENCODER_RIGHT_B_PIN, INPUT);
  pinMode(MOTOR_LEFT_REV_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_REV_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_BRAKE_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_BRAKE_PIN, OUTPUT);

  // Attach inturrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderLeftISR, RISING);   // inturrupt the pulse at rising edge
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderRightISR, RISING); // and trigger the pulse functions

  // Initialise 1 second delay timer interrupts
  timer = timerBegin(0, TIMER_PRESCALER, true);
  timerAttachInterrupt(timer, &getRPMISR, true);                                     // Set timer to execute ISR on edge
  timerAlarmWrite(timer, 1000 * TIMER_TICKS_PER_MS / MOTOR_SPEED_SAMPLE_RATE, true); // ISR is executed at a frequency defined by RPM_SAMPLE_RATE_MS
  timerAlarmEnable(timer);

  if (PROXIMITY_ENABLE)
  {
    tfmp.recoverI2CBus(I2C_SDA_PIN, I2C_SCL_PIN);
  }

  delay(500); // Wait half a second
}

void loop()
{
  // Read flysky values
  flysky_value_left = readFlyskyPWM(FLYSKY_LEFT_PIN);
  flysky_value_right = readFlyskyPWM(FLYSKY_RIGHT_PIN);
  rpm_limit = setRPMLimit();

  if (PROXIMITY_ENABLE)
  {
    tfmp.getData(tf_front_dist, TFMP_FRONT_ADDR);
    tfmp.getData(tf_rear_dist, TFMP_REAR_ADDR);
    if (tfmp.status != TFMP_READY && tfmp.status == TFMP_I2CWRITE)
    {
      tfmp.recoverI2CBus(I2C_SDA_PIN, I2C_SCL_PIN);
    }
  }

  prox_limit_front_flag = tf_front_dist < PROXIMITY_LIMIT;
  prox_limit_rear_flag = tf_rear_dist < PROXIMITY_LIMIT;

  if (abs(flysky_value_left) > DEADZONE_VALUE)
  {
    rpm_target_L = flysky_value_left / 100.0 * rpm_limit;
  }
  else
  {
    rpm_target_L = 0;
  }

  if (abs(flysky_value_right) > DEADZONE_VALUE)
  {
    rpm_target_R = flysky_value_right / 100.0 * rpm_limit;
  }
  else
  {
    rpm_target_R = 0;
  }

  // Set reverse pins to LOW
  digitalWrite(MOTOR_LEFT_REV_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_REV_PIN, LOW);
  // If flysky input is negative, set reverse to HIGH
  if (rpm_target_L < 0)
  {
    digitalWrite(MOTOR_LEFT_REV_PIN, HIGH);
  }
  if (rpm_target_R < 0)
  {
    digitalWrite(MOTOR_RIGHT_REV_PIN, HIGH);
  }

  // Calculate DAC outputs 0-255 from scaled rpm_target (-RPM_MAX to RPM_MAX)
  dac_left = abs(floor((rpm_target_L * 255.0) / RPM_MAX));
  dac_right = abs(floor((rpm_target_R * 255.0) / RPM_MAX));

  // IF absolute flysky value is greater than deadzone value AND not in proximity with an object in front or behind it
  // THEN disable brakes and set motor DAC output to flysky value
  // ELSE enable brakes and set motor DAC to zero
  if ((abs(flysky_value_left) > DEADZONE_VALUE) && !prox_limit_front_flag && !prox_limit_rear_flag)
  {
    digitalWrite(MOTOR_LEFT_BRAKE_PIN, LOW);
    dacWrite(MOTOR_LEFT_DAC_PIN, dac_left);
    motor_left_enable_brake = false;
  }
  else
  {
    digitalWrite(MOTOR_LEFT_BRAKE_PIN, HIGH);
    dacWrite(MOTOR_LEFT_DAC_PIN, 0);
    motor_left_enable_brake = true;
  }

  if ((abs(flysky_value_right) > DEADZONE_VALUE) && !prox_limit_front_flag && !prox_limit_rear_flag)
  {
    digitalWrite(MOTOR_RIGHT_BRAKE_PIN, LOW);
    dacWrite(MOTOR_RIGHT_DAC_PIN, dac_right);
    motor_right_enable_brake = false;
  }
  else
  {
    digitalWrite(MOTOR_RIGHT_BRAKE_PIN, HIGH);
    dacWrite(MOTOR_RIGHT_DAC_PIN, 0);
    motor_right_enable_brake = true;
  }

  // Debugging print statements
  // Serial.print("FlySky: ");
  // Serial.print(flysky_value_left, 3);
  // Serial.print(", ");
  // Serial.print(flysky_value_right, 3);
  // Serial.print(" | Setpoint: ");
  // Serial.print(rpm_target_L, 3);
  // Serial.print(", ");
  // Serial.print(rpm_target_R, 3);
  // Serial.print(" | RPM: ");
  // Serial.print(rpm_left, 3);
  // Serial.print(", ");
  // Serial.print(rpm_right, 3);
  // Serial.print(" | Prox: ");
  // Serial.print(prox_front.dist);
  // Serial.print(", ");
  // Serial.print(prox_rear.dist);
  // Serial.print(" | DAC: ");
  // Serial.print(dac_left);
  // Serial.print(", ");
  // Serial.println(dac_right);
  // Serial.print(" | Rev: ");
  // Serial.print(is_motor_left_reversing);
  // Serial.print(", ");
  // Serial.print(is_motor_right_reversing);
  // Serial.print(" | Brakes: ");
  // Serial.print(motor_left_enable_brake);
  // Serial.print(", ");
  // Serial.println(motor_right_enable_brake);

  range_f_msg.range = tf_front_dist;
  range_r_msg.range = tf_rear_dist;

  RCSOFTCHECK(rcl_publish(&range_f_pub, &range_f_msg, NULL));
  RCSOFTCHECK(rcl_publish(&range_r_pub, &range_r_msg, NULL));

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  delay(100); // Run at approx 50 Hz
}
