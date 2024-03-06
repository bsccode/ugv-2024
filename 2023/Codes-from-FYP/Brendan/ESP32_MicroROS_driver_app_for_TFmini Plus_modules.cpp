/*
ESP32 MicroROS program for ECU-UGV Mark II
Author: Brendan O’Malley
Current Version:
This version of the code can read the front and rear time-of-flight infrared sensors.
It uses two UARTs to read the range and publishes the data onto topics that can be read
by other ROS nodes. The sensors will be used to detect close proximity objects in order
to avoid them.
*/
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <driver/gpio.h>
#include “driver/uart.h”
#include “freertos/FreeRTOS.h”
#include “freertos/task.h”
#include “esp_system.h”
#include “esp_log.h”
// Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf(”Failed status on line %d: %d.
Aborting.\n”,__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf(”Failed status on line %d: %d.
Continuing.\n”,__LINE__,(int)temp_rc);}}
// Constants
#define NODE_NAME “range_reader”
#define FRONT_TOPIC “microROS/tof/front”
#define REAR_TOPIC “microROS/tof/rear”
#define BAUD_RATE 115200
#define TX_PIN_UART1 (GPIO_NUM_4)
#define RX_PIN_UART1 (GPIO_NUM_5)
#define TX_PIN_UART2 (GPIO_NUM_17)
#define RX_PIN_UART2 (GPIO_NUM_16)
#define UART_HEADER 0x59
static const int TIMEOUT_MS = 1000;
static const int RX_BUFFER_SIZE = 128;
// Initialise publisher variables
rcl_publisher_t front_publisher;
rcl_publisher_t rear_publisher;
// Initialise msg variables
sensor_msgs__msg__Range front_msg;
sensor_msgs__msg__Range rear_msg;
// TFmini Plus properties
float _radiation_type = sensor_msgs__msg__Range__INFRARED; // Infrared = 1
float _field_of_view = 3.6 * (3.14/180); // Field of view (rads)
float _min_range = 100; // Minimum range (mm)
float _max_range = 12000; // Maximum range (mm)
// Function declarations
void setup_uart();
void setup_ROS();
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void fill_header_stamp(sensor_msgs__msg__Range *msg);
// Function to get the current time in seconds and nanoseconds and save them into the message header
void fill_header_stamp(sensor_msgs__msg__Range *msg) {
struct timespec ts;
clock_gettime(CLOCK_REALTIME, &ts);
msg->header.stamp.sec = ts.tv_sec;
msg->header.stamp.nanosec = ts.tv_nsec;
}
void setup_uart() {
// Setup UART1
const uart_config_t uart_config1 = {
.baud_rate = BAUD_RATE,
.data_bits = UART_DATA_8_BITS,
.parity = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
.source_clk = UART_SCLK_APB,
};
uart_driver_install(UART_NUM_1, RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
uart_param_config(UART_NUM_1, &uart_config1);
uart_set_pin(UART_NUM_1, TX_PIN_UART1, RX_PIN_UART1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// Setup UART2
const uart_config_t uart_config2 = {
.baud_rate = BAUD_RATE,
.data_bits = UART_DATA_8_BITS,
.parity = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
.source_clk = UART_SCLK_APB,
};
uart_driver_install(UART_NUM_2, RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
uart_param_config(UART_NUM_2, &uart_config2);
uart_set_pin(UART_NUM_2, TX_PIN_UART2, RX_PIN_UART2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
45
void setup_ros() {
rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
// Set initial range msg values
front_msg.radiation_type = _radiation_type;
rear_msg.radiation_type = _radiation_type;
front_msg.field_of_view = _field_of_view;
rear_msg.field_of_view = _field_of_view;
front_msg.min_range = _min_range;
rear_msg.min_range = _min_range;
front_msg.max_range = _max_range;
rear_msg.max_range = _max_range;
// create init_options
RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
// create node
rcl_node_t node;
RCCHECK(rclc_node_init_default(&node, NODE_NAME, ”, &support));
// create range1 publisher
RCCHECK(rclc_publisher_init_default(&front_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), FRONT_TOPIC));
// create range2 publisher
RCCHECK(rclc_publisher_init_default(&rear_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), REAR_TOPIC));
// create timer
rcl_timer_t timer;
const unsigned int timer_timeout = 10;
RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
// create executor
rclc_executor_t executor;
RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
RCCHECK(rclc_executor_add_timer(&executor, &timer));
while(1){
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(TIMEOUT_MS));
usleep(100000);
}
// free resources
RCCHECK(rcl_publisher_fini(&front_publisher, &node));
RCCHECK(rcl_publisher_fini(&rear_publisher, &node));
RCCHECK(rcl_node_fini(&node));
vTaskDelete(NULL);
}
void read_uart(int uart_num, sensor_msgs__msg__Range *msg)
{
uint8_t* data = (uint8_t*) malloc(RX_BUFFER_SIZE+1);
const int rxBytes = uart_read_bytes(uart_num, data, 9, 100 / portTICK_PERIOD_MS);
// Check if any data exists in rx buffer
if (rxBytes <= 0) {
msg->range = msg->min_range;
}
// Check data header
if (data[0] != UART_HEADER || data[1] != UART_HEADER)
{
// Invalid header
return;
}
// Verify checksum
int check = data[0]+data[1]+data[2]+data[3]+data[4]+data[5]+data[6]+data[7];
if (data[8] == (check&0xff))
{
// Calculate values from the received bytes
msg->range = 10 * (data[3]*256 + data[2]); // Little-endian hex to dec conversion + cm -> mm conversion
}
// Set minimum range if less than specifications
if (msg->range < msg->min_range)
{
msg->range = msg->min_range;
}
// Fill the message timestamp
fill_header_stamp(&msg);
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
RCLC_UNUSED(last_call_time);
if (timer == NULL)
{
return;
}
// Read UARTs
read_uart(UART_NUM_1, &front_msg);
read_uart(UART_NUM_2, &rear_msg);
// Publish msgs
RCSOFTCHECK(rcl_publish(&front_publisher, &front_msg, NULL));
RCSOFTCHECK(rcl_publish(&rear_publisher, &rear_msg, NULL));
// Clear UART input buffers
uart_flush_input(UART_NUM_1);
uart_flush_input(UART_NUM_2);
}
// Main function
void appMain(void *arg) {
setup_uart();
setup_ros();
}