/*
 * thread_imu.h
 *
 *  Created on: May 21, 2017
 *      Author: Robin
 */

#ifndef APP_THREAD_IMU_H_
#define APP_THREAD_IMU_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

struct IMUMessage_t {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temperature;
};

extern QueueHandle_t qIMUData;

void IMUThread_Main(void * argument);
void IMUThread_IRQHandler(void);

#endif /* APP_THREAD_IMU_H_ */
