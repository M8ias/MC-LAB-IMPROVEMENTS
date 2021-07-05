// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>     
#include <fcntl.h>
#include <syslog.h>		
#include <inttypes.h>
#include <errno.h>
#include <math.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "csad_actuator_driver/csad_actuator.h"

using namespace dynamixel;

// Control table address
#define SERVO_ADDR_CW_ANGLE_LIMIT   6
#define SERVO_ADDR_CCW_ANGLE_LIMIT  8
#define SERVO_ADDR_TORQUE_ENABLE    24
#define SERVO_ADDR_GOAL_POSITION    30
#define SERVO_ADDR_PRESENT_POSITION 36
// Protocol version
#define SERVO_PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL MX series.
// Default setting
#define SERVOBAUDRATE         57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

#define NUMBER_OF_SERVOS            6

PortHandler * portHandler;
PacketHandler * packetHandler;
int fd_;
float servoOffsets[] = {-0.52f, 1.475f, -1.568f, -1.319f, 0.2f, 1.206f};
int servoIds[] = {1, 2, 3, 4, 5, 6};


/**
 * @brief Constructor
 */
CSAD_Actuator::CSAD_Actuator()
{
}

/**
 * @brief get current Position of specific servo
 * @param id uint8_t id of the servo with the position you want to read
 * @retval position -4096 is negative 180deg + 4095 is positive 180deg
 */
float CSAD_Actuator::getServoPresentPosition(uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int16_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read2ByteTxRx(
    portHandler, id, SERVO_ADDR_PRESENT_POSITION, (uint16_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", id, + servoOffsets[servoIds[id- 1] - 1] + ((float)position * (2.0f * 3.1415f) / 4096.0f));
    return (- servoOffsets[servoIds[id- 1] - 1] + ((float)position * (2.0f * 3.1415f) / 4096.0f));
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
  }
}

void CSAD_Actuator::getServoPresentPositions(uint8_t ids[], float positions[])
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int16_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  for (int i = 0; i < NUMBER_OF_SERVOS; i++)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, servoIds[ids[i]- 1], SERVO_ADDR_PRESENT_POSITION, (uint16_t *)&position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("getPosition : [ID:%d] -> [POSITION:%f]", servoIds[ids[i] - 1], + servoOffsets[servoIds[ids[i]- 1] - 1] + ((float)position * (2.0f * 3.1415f) / 4096.0f));
      positions[i] = (- servoOffsets[servoIds[ids[i]- 1] - 1] + ((float)position * (2.0f * 3.1415f) / 4096.0f));
    } else {
      ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    }
  }
}

/**
 * @brief set position of single servo
 * @param position position you want the servo set to
 * @param id id of the servo you want to set the position of
 */
void CSAD_Actuator::setServoPosition(float position, uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  //uint16_t position = (unsigned int16_t)position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, SERVO_ADDR_GOAL_POSITION, (int16_t)((position + servoOffsets[servoIds[id - 1] - 1]) * 4096.0f / (2.0f * 3.1415f) + 0.5f), &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", id, (int16_t)((position + servoOffsets[servoIds[id - 1] - 1 ]) * 4096.0f / (2.0f * 3.1415f) + 0.5f));
  } else {
    ROS_ERROR("Failed to set position! Result: %d,", dxl_comm_result);
  }
}

/**
 * @brief set position of single servo
 * @param positions positions you want the servo set to
 * @param ids ids of the servo you want to set the position of
 */
void CSAD_Actuator::setServoPositions(float positions[], uint8_t ids[])
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  //uint16_t position = (unsigned int16_t)position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  for (int i = 0; i < NUMBER_OF_SERVOS; i++)
  {
    ROS_INFO("%d, %d , %f", ids[i], servoIds[ids[i] - 1], positions[i]);
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, servoIds[ids[i] - 1], SERVO_ADDR_GOAL_POSITION, ((int16_t)((positions[i] + servoOffsets[servoIds[ids[i] - 1] - 1]) * 4096.0f / (2.0f * 3.1415f) + 0.5f)), &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", servoIds[ids[i] - 1], ((int16_t)((positions[i] + servoOffsets[servoIds[ids[i] - 1] - 1]) * 4096.0f / (2.0f * 3.1415f) + 0.5f)));
    } else {
      ROS_ERROR("Failed to set position! Result: %d, %d, %d", dxl_comm_result, servoIds[ids[i] - 1], (int16_t)(positions[i] - servoOffsets[servoIds[ids[i] - 1] - 1] * 4096.0f / (2.0f * 3.1415f) + 0.5f));
    }
  }
}

/**
 * @brief initializes all dynamixel mx106 servos
 * @retval 0 Success
 * @retval -1 Failure
 */
int CSAD_Actuator::initServos(){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(SERVO_PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(SERVOBAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  for (int i = 0; i < NUMBER_OF_SERVOS; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, servoIds[i], SERVO_ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", servoIds[i]);
      return -1;
    }
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, servoIds[i], SERVO_ADDR_CW_ANGLE_LIMIT, 4095, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to set clockwise limit for Dynamixel ID %d", servoIds[i]);
      return -1;
    }else {
      ROS_INFO("successfully set clockwise limit for Dynamixel ID %d", servoIds[i]);
    }
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, servoIds[i], SERVO_ADDR_CCW_ANGLE_LIMIT, 4095, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to set counterclockwise limit for Dynamixel ID %d", servoIds[i]);
      return -1;
    }else {
      ROS_INFO("successfully set counterclockwise limit for Dynamixel ID %d", servoIds[i]);
    }
  }
}

/**
 * @brief Open device
 * @param device Device file name (/dev/i2c-1)
 * @param freq PWM frequency
 * @retval 0 Success
 * @retval -1 Failure
 */
int CSAD_Actuator::openI2CPort(const std::string device, int freq)
{
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    perror("openPort");
    return -1;
  }
  ioctl(fd_,I2C_SLAVE, 0x40);

  setPWMFreq(freq);

  return 0;
}

/**
 * @brief Closes port.. duh
 */
void CSAD_Actuator::closeI2CPort()
{
  close(fd_);
}

/**
 * @brief resets the PCA module
 */
void CSAD_Actuator::resetPCAModule() {

	unsigned char buff[2];
	buff[0] = MODE1;
	buff[1] = 0x00;
	write(fd_, buff,2); //Normal mode
	buff[0] = MODE2;
	buff[1] = 0x04;
	write(fd_,buff,2); //totem pole

}

/**
 * @brief sets the frequency of the PWM signal.
 * @param freq desired PWM frequency in Hz.
 */
void CSAD_Actuator::setPWMFreq(int freq) {
	uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
	unsigned char buff[2];
	buff[0] = MODE1;
	buff[1] = 0x10;
	write(fd_, buff, 2); //sleep
	buff[0] = PRE_SCALE;
	buff[1] = prescale_val;
	write(fd_, buff, 2); // multiplyer for PWM frequency
	buff[0] = MODE1;
	buff[1] = 0x80;
	write(fd_, buff, 2); //restart
	buff[0] = MODE2;
	buff[1] = 0x04;
	write(fd_, buff, 2); //totem pole (default)
}

/**
 @brief sets the PWM signal on time.
 @param led PWM channel to set
 @param value value to set the PWM signal to, 0 being 0% on and 4096 being 100% on.
 */
void CSAD_Actuator::setMotorPower(uint8_t motor, float power) {
	setPWM(motor, 0, power);
}

void CSAD_Actuator::setPWM(uint8_t motor, int on_value, float power) {
  //int off_value = (int)(power * 5*51.2f) + 307.2f; //round to nearest int and map from +- 1 to 6.25 and 8.75%
  int off_value = (int)((power *205.0f) + 614.0f);
  ROS_INFO("pwm signal = %d", off_value);
	unsigned char buff[2];
	buff[0] = LED0_ON_L + LED_MULTIPLYER * (motor - 1);
	buff[1] = on_value & 0xFF;
	write(fd_, buff, 2);
	buff[0] = LED0_ON_H + LED_MULTIPLYER * (motor - 1);
	buff[1] = on_value >> 8;
	write(fd_, buff, 2);
	buff[0] = LED0_OFF_L + LED_MULTIPLYER * (motor - 1);
	buff[1] = off_value & 0xFF;
	write(fd_, buff, 2);
	buff[0] = LED0_OFF_H + LED_MULTIPLYER * (motor - 1);
	buff[1] = off_value >> 8;
	write(fd_, buff, 2);
}

/**
 @brief Get current PWM value on specified pin.
 @param led specify pin 1-16
 */
int CSAD_Actuator::getPWM(uint8_t led){
	int ledval = 0;
	unsigned char buff[1];
	buff[0] = LED0_OFF_H + LED_MULTIPLYER * (led-1);
	read(fd_, buff, 1);
	ledval = buff[0];
	ledval = ledval & 0xf;
	ledval <<= 8;
	buff[0] = LED0_OFF_L + LED_MULTIPLYER * (led-1);
	ledval += buff[0];
	return ledval;
}
