#include "sdkconfig.h"

#include "uxr/client/config.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

#include "Wire.h"

#define CMD                 16                        // Values of 0 eing sent using write have to be cast as a byte to stop them being misinterperted as NULL
                                                        // This is a but with arduino 1
#define MD25ADDRESS         88                        // Address of the MD25
#define SOFTWAREREG         13                        // Byte to read the software version
#define SPEED1              0                        // Byte to send speed to left motor
#define SPEED2              1                        // Byte to send speed to right motor
#define ENCODERONE          2                        // Byte to read motor encoder 1
#define ENCODERTWO          6                        // Byte to read motor encoder 2
#define VOLTREAD            10                        // Byte to read battery volts
#define RESETENCODERS       32
#define LED 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
int x = 0;

rcl_publisher_t encoder_publisher;
rcl_subscription_t cmd_vel_subscriber;

geometry_msgs__msg__Vector3 msg;

long encoder(int encoder_dir){                                            // Function to read and display velue of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);           
  Wire.write(encoder_dir);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to become available
  long poss2 = Wire.read();
  poss2 <<= 8;
  poss2 += Wire.read();                
  poss2 <<= 8;
  poss2 += Wire.read();                
  poss2 <<= 8;
  poss2  +=Wire.read();               
                                  
  delay(5);
  return(poss2);
}

void encodeReset(){                            // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(RESETENCODERS);                   // Putting the value 0x20 to reset encoders
  Wire.endTransmission();
	delay(50);
}

void encoder_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		msg.x = encoder(ENCODERONE);
		msg.y = encoder(ENCODERTWO);
		RCSOFTCHECK(rcl_publish(&encoder_publisher, &msg, NULL));

		//digitalWrite(LED,HIGH);
 		//delay(50);
 		//digitalWrite(LED,LOW);
 		//delay(50);
		

 		x = 127;                                                // Put a value of 127 in x, this will dive motors forward at full speed
		Wire.beginTransmission(MD25ADDRESS);                    
    Wire.write(SPEED2);                                     // Drive motor 2 at speed value stored in x
    Wire.write(x);                                           
    Wire.endTransmission();
	}
}

void appMain(void * arg)
{
	Wire.begin();
  //Serial.begin(9600);                                       // Begin serial for LCD03
  delay(100);                                               // Wait for everything to power up
  //byte softVer = getSoft();                                 // Gets the software version of MD25
  //Serial.println(softVer, DEC);                             // Print software version to the screen
	encodeReset();
  pinMode(LED,OUTPUT);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "noah_firmware", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&encoder_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"noah/encoder"));
	
	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"noah/cmd_vel"));
	// 0.89ms vel maxima
	// vel to motor -- x/0.007 + 128
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 50;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		encoder_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.x = 0.0;
	msg.y = 0.0;
	msg.z = 0.0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&encoder_publisher, &node))
	RCCHECK(rcl_node_fini(&node))

  vTaskDelete(NULL);
}

extern "C"  void microros_interface_init() {

	// start microROS task
  xTaskCreate(appMain, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);	
}