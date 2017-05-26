/*
 * thread_server.h
 *
 *  Created on: May 21, 2017
 *      Author: Robin
 */

#ifndef APP_THREAD_SERVER_H_
#define APP_THREAD_SERVER_H_

#include "FreeRTOS.h"
#include "app/thread_imu.h"

extern QueueHandle_t qIMUServerData;

void ServerThread_Main(void);
void ServerThread_SendIMUData(struct IMUMessage_t *data);

#endif /* APP_THREAD_SERVER_H_ */
