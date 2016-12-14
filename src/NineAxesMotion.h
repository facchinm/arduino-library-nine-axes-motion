/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* NAxisMotion.h
* Date: 2015/02/10
* Revision: 3.0 $
*
* Usage:        Header file of the C++ Wrapper for the BNO055 Sensor API
*
****************************************************************************
*
* Added Arduino M0/M0 Pro support
*
* Date: 07/27/2015
*
* Modified by: Arduino.org development Team.
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/
#ifndef __NAXISMOTION_H__
#define __NAXISMOTION_H__

extern "C" {
#include <utility/BNO055.h>
}
#include <Wire.h>
#include "Arduino.h"

typedef enum{
		X_AXIS = 0,
		Y_AXIS,
		Z_AXIS
} Axis;

typedef enum{
		X_QUAT = 0,
		Y_QUAT,
		Z_QUAT,
		W_QUAT
} Quaternion;

//Custom Data structures
//Structure to hold the calibration status
struct bno055_calib_stat_t {
	uint8_t accel;	//Calibration Status of the accelerometer
	uint8_t mag;	//Calibration Status of the magnetometer
	uint8_t gyro;	//Calibration Status of the gyroscope
	uint8_t system;	//Calibration Status of the overall system
};

//Structure to hold the accelerometer configurations
struct bno055_accel_stat_t {
	uint8_t range;		//Range: 2G - 16G
	uint8_t bandwidth;	//Bandwidth: 7.81Hz - 1000Hz
	uint8_t powerMode;	//Power mode: Normal - Deep suspend
};

//GPIO pins used for controlling the Sensor
//GPIO to reset the BNO055 (RESET pin has to be HIGH for the BNO055 to operate)
int RESET_PIN = 4;

//GPIO to receive the Interrupt from the BNO055 (Interrupt is visible on the INT LED on the Shield)
#if defined(__AVR_ATmega32U4__) //Arduino Yun and Leonardo have I2C shared with pins 2 and 3
int INT_PIN	= 7;
#elif defined(ARDUINO_SAMD)
int INT_PIN	= 7;				// Older SAMD boards have NMI mapped on pin 2
#else
int INT_PIN	= 2;
#endif

#define ENABLE			1		//For use in function parameters
#define DISABLE			0		//For use in function parameters
#define NO_MOTION		1		//Enables the no motion interrupt
#define SLOW_MOTION		0		//Enables the slow motion interrupt
#define ANDROID			1		//To set the Output Data Format to Android style

#if defined(ARDUINO_SAM_DUE)
#define I2C_PORT			Wire1	//Define which I2C bus is used. Wire1 for the Arduino Due
#else
#define I2C_PORT            Wire    //Or Wire
#endif

#define INIT_PERIOD			600		//Initialization period set to 600ms
#define RESET_PERIOD		300		//Reset period set to 300ms
#define POST_INIT_PERIOD	50		//Post initialization delay of 50ms
#define MANUAL				1		//To manually call the update data functions
#define AUTO				0		//To automatically call the update data functions

class NineAxesMotion {
private:
	bool 	dataUpdateMode;								//Variable to store the mode of updating data
	struct 	bno055_t 			myBNO;					//Structure that stores the device information
	struct 	bno055_accel_float_t	accelData;				//Structure that holds the accelerometer data
	struct 	bno055_mag_float_t	magData;				//Structure that holds the magnetometer data
	struct 	bno055_gyro_float_t 	gyroData;				//Structure that holds the gyroscope data
	struct 	bno055_quaternion_t	quatData;				//Structure that holds the quaternion data
	struct 	bno055_euler_float_t	eulerData;				//Structure that holds the euler data
	struct 	bno055_linear_accel_float_t	linearAccelData;	//Structure that holds the linear acceleration data
	struct 	bno055_gravity_float_t	gravAccelData;			//Structure that holds the gravity acceleration data
	struct 	bno055_calib_stat_t	calibStatus;			//Structure to hold the calibration status
	struct 	bno055_accel_stat_t	accelStatus;			//Structure to hold the status of the accelerometer configurations
public:
	//Function Declarations
	/*******************************************************************************************
	*Description: Constructor of the class with the default initialization
	*Input Parameters: None
	*Return Parameter: None
	*******************************************************************************************/
	NineAxesMotion();

	/* CONTROL FUNCTIONS */
	void begin(unsigned int address = 0x28);
	void begin(unsigned int address, int int_pin, int reset_pin);
	void reset(unsigned int address);
	void end(void);

	void readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz);
	void readMotionSensorScaled(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);

	/* ACCELEROMETER */
	void readAccelerometer(float& x, float& y, float& z);
	float readAccelerometer(int axis);
	void readAccel(float& x, float& y, float& z);
	float readAccel(int axis);

	/* GRAV. ACCELERATION */
	void readGravAcceleration(float& x, float& y, float& z);
	float readGravAcceleration(int axis);
	void readGravAccel(float& x, float& y, float& z);
	float readGravAccel(int axis);

	/* LINEAR. ACCELERATION */
	void readLinearAcceleration(float& x, float& y, float& z);
	float readLinearAcceleration(int axis);
	void readLinearAccel(float& x, float& y, float& z);
	float readLinearAccel(int axis);

	/* GYRO */
	void readGyro(float& x, float& y, float& z);
	float readGyro(int axis);

	/* MAGNETOMETER */
	void readMagnetometer(float& x, float& y, float& z);
	float readMagnetometer(int axis);
	void readMag(float& x, float& y, float& z);
	float readMag(int axis);

	/* QUATERNION */
	void readQuaternion(int16_t& w, int16_t& x, int16_t& y, int16_t& z);
	int16_t readQuaternion(int axis);
	void readQuat(int16_t& w, int16_t& x, int16_t& y, int16_t& z);
	int16_t readQuat(int quaternion);

};

/******************** Bridge Functions for the Sensor API to control the Arduino Hardware******************************************/
signed char BNO055_I2C_bus_read(unsigned char,unsigned char, unsigned char*, unsigned char);
signed char BNO055_I2C_bus_write(unsigned char ,unsigned char , unsigned char* , unsigned char );
void _delay(u_32) __attribute__((always_inline));

#endif __NAXISMOTION_H__
