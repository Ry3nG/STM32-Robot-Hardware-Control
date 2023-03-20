# STM32-Robot-Hardware-Control
This repository showcases the hardware engineering work for the MDP project at NTU. The project is designed to control the movement and turning of a robot using an STM32 microcontroller and a Raspberry Pi. The code includes a PID controller for precise turning and movement, as well as support for input from external devices like an Android tablet. The Raspberry Pi also integrates a camera to capture images and communicate with a computer vision server.

### Features
* Precise control of robot movement using PID controllers
* Turning and movement commands through UART communication
* Support for different movement directions:
 * Forward
 * Backward
 * Forward Left
 * Forward Right
 * Backward Left
 * Backward Right
*  Center calibration and open-loop movement support
* Gyroscope integration for accurate angle measurements
* Raspberry Pi integration for camera and ultrasonic sensor support
* Communication with an Android tablet for starting the robot and monitoring its progress

### Key Functions
STM32:
 * robotTurnPID(float *targetAngle, int direction): Turns the robot using a PID controller to achieve the desired angle based on gyroscope readings. The function takes a pointer to the target angle and a direction flag (0 for left, 1 for right).
chassisTask(void *argument): Main control task for the robot's movement. It reads commands from a message queue, decodes the commands, and controls the motors and servos accordingly.
 * HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart): UART receive complete callback function, triggered when a command is received through UART. It processes the received command and puts it into the message queue for the chassisTask function to handle.
and other helper functions.
Raspberry Pi:
 * capture_and_send(): Captures an image using the PiCamera and sends it to a computer vision server for processing.
 * get_distance(): Measures the distance between the robot and an obstacle using an ultrasonic sensor.
 * movement_task(): Main function for controlling the robot's movement based on the CV server's response and ultrasonic sensor readings.
 * approach_obstacle_and_advance(target_distance, dash_distance, back_conpensate): Approaches an obstacle and advances the robot based on the provided parameters.
 * Task-specific functions for different movement scenarios.
 * Communication functions for sending commands to the STM32 and Android tablet.
