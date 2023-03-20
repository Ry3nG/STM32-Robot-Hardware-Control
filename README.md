# STM32-Robot-Hardware-Control
This repository showcases my hardware engineering work for the MDP project at NTU. 
It contains the source code for controlling a robot using an STM32 microcontroller. The project is designed to control the movement and turning of the robot through PWM signals sent to its motors. The code includes a PID controller for precise turning and movement, as well as support for input from an external device like a Raspberry Pi.

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
  * Center calibration and open-loop movement support
* Gyroscope integration for accurate angle measurements

### Key Functions
* robotTurnPID(float *targetAngle, int direction): Turns the robot using a PID controller to achieve the desired angle based on gyroscope readings. The function takes a pointer to the target angle and a direction flag (0 for left, 1 for right).
* chassisTask(void *argument): Main control task for the robot's movement. It reads commands from a message queue, decodes the commands, and controls the motors and servos accordingly.
* HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart): UART receive complete callback function, triggered when a command is received through UART. It processes the received command and puts it into the message queue for the chassisTask function to handle.

and other helper functions. 
