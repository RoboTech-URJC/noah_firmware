#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#include "esp_attr.h"


#include "driver/gpio.h"
#include "driver/timer.h"

#include "sensor.h"
#include "motor.h"


#define LOG_INFO_MSG_SIZE 200


rcl_subscription_t subscriber_wheel_power_left;
rcl_subscription_t subscriber_wheel_power_right;

rcl_publisher_t log_info_publisher;
rcl_publisher_t wheel_radps_left_publisher;
rcl_publisher_t wheel_radps_right_publisher;

std_msgs__msg__Float32 wheel_power_left_msg;
std_msgs__msg__Float32 wheel_power_right_msg;
std_msgs__msg__Float32 wheel_radps_left_msg;
std_msgs__msg__Float32 wheel_radps_right_msg;
std_msgs__msg__String log_info_msg;


void log_info_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);

	if (timer != NULL) {
		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		//sprintf(log_info_string, "Hadabot heartbeat %d_%d", ts.tv_sec, ts.tv_nsec);

		//sprintf(log_info_msg.data.data, "Hadabot heartbeat %d_%d", ts.tv_sec, ts.tv_nsec);
		sprintf(log_info_msg.data.data, "Hadabot heartbeat");
		log_info_msg.data.size = strlen(log_info_msg.data.data);
		
		
		// Reset the pong count and publish the ping message

		rcl_publish(&log_info_publisher, (const void*)&log_info_msg, NULL);
		printf("Sent Hadabot heartbeat\n");
	}
}


void left_motor_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Received message /hadabot/wheel_power_left.");
	if (msg != NULL) {
		printf("Data: %f\n", msg->data);
		
		updateMotorState(MCPWM_UNIT_0, MCPWM_TIMER_0, &left_wheel_state, &left_stoping_counter, msg->data);
	}
}

void right_motor_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Received message /hadabot/wheel_power_right.");
	if (msg != NULL) {
		printf("Data: %f\n", msg->data);
		
		updateMotorState(MCPWM_UNIT_0, MCPWM_TIMER_1, &right_wheel_state, &right_stoping_counter, msg->data);
	}
}



float wheel_radps_left_prev = 0;
float wheel_radps_right_prev = 0;

// Не правильно отслеживать последнее совпадение здесь

void wheel_radsp_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);

	
	if (timer != NULL) {
		if (left_wheel_state != STOPED) {
			
			if (left_wheel_state == STOPING_FORWARD || left_wheel_state == STOPING_BACKWARD) {
				updateStopingSubstate(&left_wheel_state, &left_stoping_counter);
			
				//if (wheel_radps_left_msg.data == wheel_radps_left_prev && abs(wheel_radps_left_msg.data) < 6) {
				if (abs(wheel_radps_left_msg.data) < 6) {
					wheel_radps_left_msg.data = 0;
					left_stoping_counter = 0;
					left_wheel_state = STOPED;
				}
			}
			
			rcl_publish(&wheel_radps_left_publisher, (const void*)&wheel_radps_left_msg, NULL);		
			wheel_radps_left_prev = wheel_radps_left_msg.data;

		}			
		if (right_wheel_state != STOPED) {
			if (right_wheel_state == STOPING_FORWARD || right_wheel_state == STOPING_BACKWARD) {
				updateStopingSubstate(&right_wheel_state, &right_stoping_counter);
				
				//if (wheel_radps_right_prev == wheel_radps_right_msg.data && abs(wheel_radps_right_msg.data) < 6) {
				if (abs(wheel_radps_right_msg.data) < 6) {
					wheel_radps_right_msg.data = 0;
					right_stoping_counter = 0;
					right_wheel_state = STOPED;
				}

			}
				
			rcl_publish(&wheel_radps_right_publisher, (const void*)&wheel_radps_right_msg, NULL);
			wheel_radps_right_prev = wheel_radps_right_msg.data;
		}			
		


	}

}

void appMain(void * arg)
{
	mcpwm_brushed_motor_control_init(arg);

	wheel_sensors_init();
	
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "hadabot_esp32", "", &support));

	RCCHECK(rclc_publisher_init_default(&log_info_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "hadabot/log/info"));

	rcl_timer_t log_info_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&log_info_timer, &support, RCL_MS_TO_NS(5000), log_info_timer_callback));
	
	rcl_timer_t wheel_radsp_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&wheel_radsp_timer, &support, RCL_MS_TO_NS(15), wheel_radsp_timer_callback));
	
	
	RCCHECK(rclc_publisher_init_default(&wheel_radps_left_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/wheel_radps_left"));

	RCCHECK(rclc_publisher_init_default(&wheel_radps_right_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "hadabot/wheel_radps_right"));
		
		
	// create subscriber for wheel power left message
	
	RCCHECK(rclc_subscription_init_default(
		&subscriber_wheel_power_left,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/hadabot/wheel_power_left"));


// create subscriber for wheel power right message

	RCCHECK(rclc_subscription_init_default(
		&subscriber_wheel_power_right,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"/hadabot/wheel_power_right"));
		
	
	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

	unsigned int rcl_wait_timeout = 100;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_timer(&executor, &wheel_radsp_timer));		
	RCCHECK(rclc_executor_add_timer(&executor, &log_info_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_left, &wheel_power_left_msg, &left_motor_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_power_right, &wheel_power_right_msg, &right_motor_callback, ON_NEW_DATA));

	log_info_msg.data.data = (char * ) malloc(LOG_INFO_MSG_SIZE * sizeof(char));
	log_info_msg.data.size = 0;
	log_info_msg.data.capacity = LOG_INFO_MSG_SIZE;

	
	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&log_info_publisher, &node));
	RCCHECK(rcl_publisher_fini(&wheel_radps_left_publisher, &node));
	RCCHECK(rcl_publisher_fini(&wheel_radps_right_publisher, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_left, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_wheel_power_right, &node));
	RCCHECK(rcl_node_fini(&node));
	vTaskDelete(NULL);
}
