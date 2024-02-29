/*
ECU UGV teleoperation using the ESP32 platform, Flysky remote controller/receiver
and VESC 75100 motor controllers

Dependencies:
- TFmini Plus I2C library

Hardware:
- ESP32 DevkitV4
- TFmini Plus IR Sensor
- MG90s Servo
- VESC  motor conntroller
*/

#include <Wire.h>
#include <math.h>
#include <TFMPI2C.h> // TFmini plus api

#include <ESP32Servo.h>
#include <esp_log.h>

// Input Pin Definitions
const int FLYSKY_LEFT_PIN = 27;  // Left Flysky Stick (CH3)
const int FLYSKY_RIGHT_PIN = 14; // Right Flysky Stick (CH2)
const int FLYSKY_SWB_PIN = 12;   // Left three-way switch (CH7)
const int FLYSKY_SWA_PIN = 13;   // Left two-way switch (CH5)

const int I2C_SDA_PIN = 21; // I2C Data bus pin
const int I2C_SCL_PIN = 22; // I2C Clock bus pin

const int MOTOR_LEFT_PWM_PIN = 25;  // Servo output pin for left motor controller
const int MOTOR_RIGHT_PWM_PIN = 26; // Servo output pin for right motor controller

const int ENCODER_LEFT_A_PIN = 19;  // Left Rotary Encoder phase A pin
const int ENCODER_LEFT_B_PIN = 18;  // Left Rotary Encoder phase B pin
const int ENCODER_RIGHT_A_PIN = 5;  // Right Rotary Encoder Phase A pin
const int ENCODER_RIGHT_B_PIN = 17; // Right Rotary Encoder Phase B pin

const int SERVO_SWEEP_PIN = 15;

// Output Pin Definitions
// const int MOTOR_LEFT_DAC_PIN = 25;  // Left motor controller DAC output pin
// const int MOTOR_RIGHT_DAC_PIN = 26; // Right motor controller DAC output pin

const int DEADZONE_VALUE = 10;        // Flysky stick deadzone value
const double PULSES_PER_REV = 2560.0; // Pulses per revolution. Actual PPR for the encoder is 1024.
                                      // A gear step up of 0.4 gives us 1024 / 0.4 = 2560

// Motors controlled using centered PWM duty
// i.e. PWM=1000 -> Max reverse, DAC=1500 -> Brake/Stop, PWM=2000 -> Max forward
const int MOTOR_PWM_RANGE_LIMIT_LOW = 145;    // 145 for 1m/s
const int MOTOR_PWM_RANGE_LIMIT_MEDIUM = 215; // 215 for 2m/s
const int MOTOR_PWM_RANGE_LIMIT_HIGH = 290;   // 290 for 3m/s

const int FLYSKY_PWM_MIN_DUTY = 1000;
const int FLYSKY_PWM_MAX_DUTY = 2000;
const int FLYSKY_PWM_CENTER_DUTY = 1500;
const int FLYSKY_PWM_DEADZONE = 10;

const int TIMER_PRESCALER = 80;             // Prescaler to set period of timer using the default clock of 80Mhz (APB_CLK)
const int TIMER_TICKS_PER_MS = 1000;        // Timer ticks for every millisecond
const double MOTOR_SPEED_SAMPLE_RATE = 5.0; // RPM calculation frequency (Hz)

// TODO: Add a jumper pin to enable a debugging mode
const bool PROXIMITY_ENABLE = true;     // Enable proximity sensors and automatic braking
const int TFMP_FRONT_ADDR = 0x10;       // Front sensor I2C address
const int TFMP_REAR_ADDR = 0x11;        // Rear sensor I2C address
const int PROXIMITY_LIMIT_SLOW = 50;    // Limit for slow speeds (cm)
const int PROXIMITY_LIMIT_MEDIUM = 200; // Limit for medium speeds
const int PROXIMITY_LIMIT_FAST = 350;   // Limit for fast speeds
const int PROXIMITY_BRAKE_TIMEOUT = 1500;

const double SERVO_SWEEP_RATE = 2.0;

// Enable tasks
const int SERVO_SWEEP_TASK_ENABLE = true;

// Declare TFMPI2C handle
TFMPI2C tfmp;

// Initialise front and rear distance variables
int16_t tf_front = 500;
int16_t tf_rear = 500;

// Flags for when the UGV is in proximity with an object
int proximity_limit = PROXIMITY_LIMIT_SLOW;
volatile bool proximity_limit_front_flag = 0;
volatile bool proximity_limit_rear_flag = 0;

// Servo objects to handle pwm output
Servo ServoMotorLeft;
Servo ServoMotorRight;

// rpm used to calculate rpm as the function in the loop
double rpm_left;
double rpm_right;

// Encoder pulse counters
volatile unsigned int pulse_counter_left = 0;
volatile unsigned int pulse_counter_right = 0;

// Variable to store the encoder readings
int encoder_left_B_value;
int encoder_right_B_value;

// Variables to determine motor direction
bool is_motor_left_reversing;
bool is_motor_right_reversing;

Servo ServoSweep;
const int SERVO_SWEEP_RESOLUTION = 20;
int servo_sweep_counter = 0;
int servo_sweep_positions[SERVO_SWEEP_RESOLUTION];
int servo_sweep_pos = 0;
int servo_sweep_enable = false;

// Initialise timer for ISR
hw_timer_t *timer = NULL;

// Task handlers
TaskHandle_t ServoSweepTask;
TaskHandle_t ProximityReadTask;

// Return the duty cycle in microseconds of the given flysky analog stick channel
// Range: 1000us to 2000us
// Centre: 1500us
int readFlysky(int channel_pin)
{
  int duty = pulseIn(channel_pin, HIGH, 30000);
  if (duty < FLYSKY_PWM_MIN_DUTY - FLYSKY_PWM_DEADZONE || duty > FLYSKY_PWM_MAX_DUTY + FLYSKY_PWM_DEADZONE)
  {
    return FLYSKY_PWM_CENTER_DUTY; // Return center duty value if read value not in expected range
  }
  return constrain(duty, FLYSKY_PWM_MIN_DUTY, FLYSKY_PWM_MAX_DUTY); // Read PWM duty cycle which is between 1000-2000 for the flysky remote
}

// Return the state of a three-way switch from the given flysky channel. Default to 0.
// 1 == LOW == 1000us
// 2 == MEDIUM == 1500us
// 3 == HIGH == 2000us
int readFlyskyThreeSwitch(int channel_pin)
{
  int duty = pulseIn(channel_pin, HIGH, 30000);
  if (duty > FLYSKY_PWM_MIN_DUTY - FLYSKY_PWM_DEADZONE &&
      duty < FLYSKY_PWM_MIN_DUTY + FLYSKY_PWM_DEADZONE)
  {
    return 1;
  }
  else if (duty < FLYSKY_PWM_CENTER_DUTY + FLYSKY_PWM_DEADZONE)
  {
    return 2;
  }
  else if (duty < FLYSKY_PWM_MAX_DUTY + FLYSKY_PWM_DEADZONE)
  {
    return 3;
  }
  else
  {
    log_w("Switch position not detected...");
    return 0;
  }
}

// Return the state of the switch from the given flysky channel
// True == ON == 2000us
// False == OFF == 1000us
bool readFlyskySwitch(int channel_pin)
{
  int duty = pulseIn(channel_pin, HIGH, 30000);
  if (duty > FLYSKY_PWM_CENTER_DUTY)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Returns the motor pwm limit given the position of the three-way switch value
// i.e sets the motor speed depending on switch level
int updateOutputLimits(int switch_pos)
{
  log_d("Updating pwm output limits...");
  switch (switch_pos)
  {
  case 1:
    return MOTOR_PWM_RANGE_LIMIT_LOW;
  case 2:
    return MOTOR_PWM_RANGE_LIMIT_MEDIUM;
  case 3:
    return MOTOR_PWM_RANGE_LIMIT_HIGH;
  default:
    return MOTOR_PWM_RANGE_LIMIT_LOW;
  }
}

// Return the auto braking trigger distance given the state of the given three-way switch
int updateProximityLimits(int switch_pos)
{
  log_d("Updating proximity limits...");
  switch (switch_pos)
  {
  case 1:
    return PROXIMITY_LIMIT_SLOW;
  case 2:
    return PROXIMITY_LIMIT_MEDIUM;
  case 3:
    return PROXIMITY_LIMIT_FAST;
  default:
    return PROXIMITY_LIMIT_SLOW;
  }
}
// Interrupt routine to read and count encoder pulses
// Increment counter and determine motor direction using Phase B.
// Triggered at each rising edge of Phase A.
void encoderLeftISR()
{
  pulse_counter_left++;
  encoder_left_B_value = digitalRead(ENCODER_LEFT_B_PIN); // Read B phase encoder value on left side
  is_motor_left_reversing = encoder_left_B_value;         // If B phase is HIGH when A phase is HIGH, then motor is rotating in reverse
}

// Interrupt routine to read and count encoder pulses
// Same as above routine but for right side
void encoderRightISR()
{
  pulse_counter_right++;
  encoder_right_B_value = digitalRead(ENCODER_RIGHT_B_PIN);
  is_motor_right_reversing = !encoder_right_B_value; // If B phase is LOW when A phase is HIGH, then motor is rotating in reverse
}

// Timer callback function to calculate real-time RPM from encoder measurements
void IRAM_ATTR getRPMISR() // TODO: Experiment with using FreeRTOS task handlers for this ISR
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

// Return the input direction given the duty of the flysky analog stick
// 0 == Stop/Brake == 1500us
// 1 == Forwards == 2000us
// -1 == Reverse == 1500us
int getInputDirection(int flysky_value)
{
  if (flysky_value < FLYSKY_PWM_CENTER_DUTY + DEADZONE_VALUE &&
      flysky_value > FLYSKY_PWM_CENTER_DUTY - DEADZONE_VALUE)
  {
    return 0;
  }
  else if (flysky_value < FLYSKY_PWM_CENTER_DUTY)
  {
    return -1;
  }
  else if (flysky_value > FLYSKY_PWM_CENTER_DUTY)
  {
    return 1;
  }
  return 0;
}

// Task loop to set the positions of the sweeping servos
void servoSweepLoop(void *pvParameters)
{
  if (!SERVO_SWEEP_TASK_ENABLE)
  {
    vTaskDelete(ServoSweepTask);
  }
  log_i("Starting %s on core %i", pcTaskGetName(NULL), xPortGetCoreID());

  const TickType_t delay = (1000.0 / SERVO_SWEEP_RESOLUTION) / SERVO_SWEEP_RATE; // ms
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;)
  {
    if (servo_sweep_counter >= SERVO_SWEEP_RESOLUTION)
    {
      servo_sweep_counter = 0;
    }

    servo_sweep_pos = servo_sweep_positions[servo_sweep_counter];
    if (servo_sweep_enable)
    {
      ServoSweep.write(servo_sweep_pos);
    }
    else
    {
      ServoSweep.write(45);
    }

    servo_sweep_counter++;
    vTaskDelayUntil(&last_wake_time, delay);
  }
}

// Task loop to read the proximity sensors
void proximityReadLoop(void *pvParameters)
{
  if (!PROXIMITY_ENABLE)
  {
    vTaskDelete(ProximityReadTask);
  }

  log_i("Starting %s on core %i", pcTaskGetName(NULL), xPortGetCoreID());

  const TickType_t delay = 10;
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;)
  {
    tfmp.getData(tf_front, TFMP_FRONT_ADDR);
    tfmp.getData(tf_rear, TFMP_REAR_ADDR);
    log_d("F: %i R: %i", tf_front, tf_rear);
    if (tfmp.status != TFMP_READY && tfmp.status == TFMP_I2CWRITE)
    {
      // TODO: timeout counter
      log_e("Unable to read sensor(s)");
      tfmp.printFrame();

      log_e("Attempting soft reset...");

      tfmp.sendCommand(SOFT_RESET, 0, TFMP_FRONT_ADDR);
      tfmp.printReply();
      tfmp.sendCommand(SOFT_RESET, 0, TFMP_REAR_ADDR);
      tfmp.printReply();

      tfmp.recoverI2CBus(I2C_SDA_PIN, I2C_SCL_PIN);
    }

    proximity_limit_front_flag = tf_front < proximity_limit && tf_front != 0;
    proximity_limit_rear_flag = tf_rear < proximity_limit && tf_rear != 0;

    if (proximity_limit_front_flag)
    {
      log_v("Front proximity limit reached!");
    }
    if (proximity_limit_rear_flag)
    {
      log_v("Rear proximity limit reached!");
    }

    vTaskDelayUntil(&last_wake_time, delay);
  }
}

void setup()
{
  log_i("Begin setup...");
  // Set up serial monitor and has to be 115200 for the esp32
  int baud_rate = 115200;
  log_d("Initialising serial comms using baud rate: %i...", baud_rate);
  Serial.begin(baud_rate);
  delay(100);
  log_d("Successful.");

  // Initialise input and output pins
  log_d("Initialising GPIO pins...");
  pinMode(FLYSKY_LEFT_PIN, INPUT);
  pinMode(FLYSKY_RIGHT_PIN, INPUT);
  pinMode(FLYSKY_SWB_PIN, INPUT);
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  pinMode(ENCODER_LEFT_B_PIN, INPUT);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT);
  pinMode(ENCODER_RIGHT_B_PIN, INPUT);
  log_d("Successful.");
  delay(100);
  // Attach inturrupt
  log_d("Attaching interrupt pins...");
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderLeftISR, RISING);   // inturrupt the pulse at rising edge
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), encoderRightISR, RISING); // and trigger the pulse functions
  log_d("Successful.");
  delay(100);
  // Initialise 1 second delay timer interrupts
  log_d("Initialising timer...");
  timer = timerBegin(0, TIMER_PRESCALER, true);
  timerAttachInterrupt(timer, &getRPMISR, true);                                     // Set timer to execute ISR on edge
  timerAlarmWrite(timer, 1000 * TIMER_TICKS_PER_MS / MOTOR_SPEED_SAMPLE_RATE, true); // ISR is executed at a frequency defined by RPM_SAMPLE_RATE_MS
  timerAlarmEnable(timer);
  log_d("Successful.");
  delay(100);

  if (PROXIMITY_ENABLE)
  {
    log_d("Recovering I2C bus...");
    tfmp.recoverI2CBus(I2C_SDA_PIN, I2C_SCL_PIN);
    log_d("Successful.");
    delay(100);
  }

  xTaskCreatePinnedToCore(
      servoSweepLoop,
      "ServoSweepTask",
      10000,
      NULL,
      2,
      &ServoSweepTask,
      0);
  delay(100);

  xTaskCreatePinnedToCore(
      proximityReadLoop,
      "ProxReadTask",
      10000,
      NULL,
      3,
      &ProximityReadTask,
      0);
  delay(100);

  log_d("Initialising PWM motor control outputs...");
  ServoMotorLeft.setPeriodHertz(50);
  ServoMotorRight.setPeriodHertz(50);
  ServoMotorLeft.attach(MOTOR_LEFT_PWM_PIN, 1000, 2000);
  ServoMotorRight.attach(MOTOR_RIGHT_PWM_PIN, 1000, 2000);
  ServoSweep.setPeriodHertz(50);
  ServoSweep.attach(SERVO_SWEEP_PIN, 400, 2400);
  log_d("Successful.");
  delay(100);

  // Generate servo positions
  log_d("Generating servo sweep positions...");
  for (int t = 0; t < SERVO_SWEEP_RESOLUTION; t++)
  {
    if (t < (SERVO_SWEEP_RESOLUTION / 2) + 1)
    {
      // servo_sweep_positions[t] = round(45.0 - 45.0 * cos(2.0 * M_PI * t / (double)SERVO_SWEEP_RESOLUTION));
      servo_sweep_positions[t] = round((180.0 * t) / SERVO_SWEEP_RESOLUTION);
    }
    else
    {
      servo_sweep_positions[t] = round(180.0 - ((180.0 * t) / SERVO_SWEEP_RESOLUTION));
    }
  }
  log_d("Successful.");

  delay(500); // Wait half a second
  log_i("Initial Setup Done.");
}

void loop()
{
  // Read flysky values
  int flysky_value_left = readFlysky(FLYSKY_LEFT_PIN);
  int flysky_value_right = readFlysky(FLYSKY_RIGHT_PIN);

  // Determine input direction
  int input_direction_left = getInputDirection(flysky_value_left);
  int input_direction_right = getInputDirection(flysky_value_right);

  int switch_pos = readFlyskyThreeSwitch(FLYSKY_SWB_PIN);
  int pwm_duty_limit = updateOutputLimits(switch_pos);
  proximity_limit = updateProximityLimits(switch_pos);

  servo_sweep_enable = readFlyskySwitch(FLYSKY_SWA_PIN);

  int servo_duty_left;
  int servo_duty_right;

  // Brake if front prox detected AND trying to move forwards
  if (proximity_limit_front_flag && (input_direction_left == 1 || input_direction_right == 1))
  {
    // Apply brakes for a set time
    log_i("Applying brakes...");
    ServoMotorLeft.writeMicroseconds(FLYSKY_PWM_CENTER_DUTY);
    ServoMotorRight.writeMicroseconds(FLYSKY_PWM_CENTER_DUTY);
    delay(PROXIMITY_BRAKE_TIMEOUT); // FIXME: Needs to be changed so that that the whole code isn't interrupted by this delay
  }
  else if (proximity_limit_rear_flag && (input_direction_left == -1 || input_direction_right == -1))
  {
    // Apply brakes for a set time
    log_i("Applying brakes...");
    ServoMotorLeft.writeMicroseconds(FLYSKY_PWM_CENTER_DUTY);
    ServoMotorRight.writeMicroseconds(FLYSKY_PWM_CENTER_DUTY);
    delay(PROXIMITY_BRAKE_TIMEOUT);
  }
  else
  {
    // Map flysky pwm range to motor pwm range
    log_d("Driving...");
    int servo_duty_min = FLYSKY_PWM_CENTER_DUTY - pwm_duty_limit;
    int servo_duty_max = FLYSKY_PWM_CENTER_DUTY + pwm_duty_limit;
    servo_duty_left = map(flysky_value_left, FLYSKY_PWM_MIN_DUTY, FLYSKY_PWM_MAX_DUTY, servo_duty_min, servo_duty_max);
    servo_duty_right = map(flysky_value_right, FLYSKY_PWM_MIN_DUTY, FLYSKY_PWM_MAX_DUTY, servo_duty_min, servo_duty_max);

    // Write the DAC values to the DAC pins
    ServoMotorLeft.writeMicroseconds(servo_duty_left);
    ServoMotorRight.writeMicroseconds(servo_duty_right);
  }

  double mps_left = (rpm_left * 0.34 * M_PI) / 60.0;
  double mps_right = (rpm_right * 0.34 * M_PI) / 60.0;
  double rps_left = rpm_left / 60.0;
  double rps_right = rpm_right / 60.0;

  log_i("IN: (%4i, %4i) RPM: (%3.2f, %3.2f) RPS: (%.2f, %.2f) MPS: (%.2f, %.2f)", servo_duty_left, servo_duty_right, rpm_left, rpm_right, rps_left, rps_right, mps_left, mps_right);

  delay(20); // Run at approx 50 Hz
}
