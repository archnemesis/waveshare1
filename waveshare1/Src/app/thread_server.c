/*
 * thread_server.c
 *
 *  Created on: May 21, 2017
 *      Author: Robin
 */

#include "app/thread_server.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "sockets.h"
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rtc.h"
#include "rtc.h"

QueueHandle_t qIMUServerData;

int clients[5] = { 0 };
char *buffer;

void ServerThread_Main(void)
{
	qIMUServerData = xQueueCreate(10, sizeof(struct IMUMessage_t *));
	struct IMUMessage_t *msg;
	int socket_fd;
	struct sockaddr_in sa, ra;
	struct fd_set master_set;
	struct fd_set working_set;
	struct fd_set writing_set;
	struct timeval timeout;
	int max_fd, new_fd, ready_fd;
	int on = 1;
	int rc = 0;
	int i;
	uint32_t opt;

	buffer = (char *)pvPortMalloc(512);

	socket_fd = socket(PF_INET, SOCK_STREAM, 0);

	if (socket_fd < 0) {
		__BKPT();
	}

	opt = fcntl(socket_fd, F_GETFL, 0);
	opt |= O_NONBLOCK;
	fcntl(socket_fd, F_SETFL, opt);

	memset(&sa, 0, sizeof(struct sockaddr_in));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	sa.sin_len = sizeof(sa);
	sa.sin_port = htons(445);

	if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(sa)) == -1) {
		close(socket_fd);
		__BKPT();
	}

	if (listen(socket_fd, 1) == -1) {
		close(socket_fd);
		__BKPT();
	}

	FD_ZERO(&master_set);
	max_fd = socket_fd;
	FD_SET(socket_fd, &master_set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	for(;;) {
		while (xQueueReceive(qIMUServerData, (void *)&msg, 0) == pdTRUE) {
			sprintf(buffer, "Gyro X: %d, Y: %d, Z: %d\r\n",
					msg->gyro_x,
					msg->gyro_y,
					msg->gyro_z);

			for (i = 0; i <= max_fd; i++) {
				if (FD_ISSET(i, &master_set)) {
					if (i != socket_fd) {
						send(i, buffer, strlen(buffer), 0);
					}
				}
			}

			vPortFree(msg);
		}

		RTC_DateTypeDef date;
		RTC_TimeTypeDef time;

		if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) == HAL_OK) {
			if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) == HAL_OK) {
				sprintf(buffer, "%02d/%02d/%02d %02d:%02d:%02d\r\n",
						date.Month,
						date.Date,
						date.Year,
						time.Hours,
						time.Minutes,
						time.Seconds);
			}
		}

		memcpy(&working_set, &master_set, sizeof(master_set));
		memcpy(&writing_set, &master_set, sizeof(master_set));
		rc = select(max_fd + 1, &working_set, &writing_set, NULL, &timeout);

		if (rc < 0) {
			__BKPT();
		}

		if (rc == 0) {
			continue;
		}

		ready_fd = max_fd;
		for (i = 0; i <= ready_fd; i++) {
			if (i == socket_fd) {
				if (FD_ISSET(i, &working_set)) {
					do {
						on = sizeof(ra);
						new_fd = accept(socket_fd, (struct sockaddr *)&ra, (socklen_t *)&on);
						if (new_fd < 0) {
							if (errno != EWOULDBLOCK) {
								__BKPT();
							}
							break;
						}

						/* add socket to master set */
						FD_SET(new_fd, &master_set);

						/* keep track of the max */
						if (new_fd > max_fd) {
							max_fd = new_fd;
						}
					} while (new_fd != -1);
				}
			}
			else {
				if (FD_ISSET(i, &working_set)) {
					rc = recv(i, buffer, sizeof(buffer), 0);

					if (rc <= 0) {
						if (errno != EWOULDBLOCK) {
							close(i);
							FD_CLR(i, &master_set);
							if (i == max_fd) {
								while (FD_ISSET(max_fd, &master_set) == false) {
									max_fd -= 1;
								}
							}
						}
					}
					else {
						/* TODO: process client commands */
					}
				}
				if (FD_ISSET(i, &writing_set)) {
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
					send(i, buffer, strlen(buffer), MSG_DONTWAIT);
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
				}
			}
		}
		osDelay(100);
	}
}
