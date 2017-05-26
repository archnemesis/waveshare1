/*
 * thread_imu.c
 *
 *  Created on: May 21, 2017
 *      Author: Robin
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "mpu6050.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "i2c.h"
#include "app/thread_imu.h"
#include "app/thread_server.h"

extern osThreadId imuTaskHandle;

#define EVENT_I2C	0x01
#define EVENT_IMU	0x02

void IMUThread_Main(void * argument)
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	uint8_t data[14];
	uint32_t notify_value;
	HAL_StatusTypeDef err;
	struct IMUMessage_t *msg;

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	for (;;) {
		xTaskNotifyWait(
				EVENT_IMU,
				0x00,
				&notify_value,
				portMAX_DELAY);

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);

		/*
		 * Read new values from IMU then wait for DMA...
		 */
		if (notify_value == EVENT_IMU) {
			err = HAL_I2C_Mem_Read(
				MPU6050_I2C,
				(uint16_t)MPU6050_DEFAULT_ADDRESS,
				(uint16_t)MPU6050_RA_ACCEL_XOUT_H,
				I2C_MEMADD_SIZE_8BIT,
				(uint8_t *)data,
				14, 1);

			if (err == HAL_BUSY) {
				(MPU6050_I2C)->Instance->CR1 |= I2C_CR1_SWRST;
				(MPU6050_I2C)->Instance->CR1 &= ~I2C_CR1_SWRST;
			}

//			err = HAL_I2C_Mem_Read(
//				MPU6050_I2C,
//				(uint16_t)MPU6050_DEFAULT_ADDRESS,
//				(uint16_t)MPU6050_RA_ACCEL_YOUT_H,
//				I2C_MEMADD_SIZE_8BIT,
//				(uint8_t *)&ay,
//				2, 1);
//
//			if (err == HAL_BUSY) {
//				(MPU6050_I2C)->Instance->CR1 |= I2C_CR1_SWRST;
//				(MPU6050_I2C)->Instance->CR1 &= ~I2C_CR1_SWRST;
//			}
//
//			err = HAL_I2C_Mem_Read(
//				MPU6050_I2C,
//				(uint16_t)MPU6050_DEFAULT_ADDRESS,
//				(uint16_t)MPU6050_RA_ACCEL_ZOUT_H,
//				I2C_MEMADD_SIZE_8BIT,
//				(uint8_t *)&az,
//				2, 1);
//
//			if (err == HAL_BUSY) {
//				(MPU6050_I2C)->Instance->CR1 |= I2C_CR1_SWRST;
//				(MPU6050_I2C)->Instance->CR1 &= ~I2C_CR1_SWRST;
//			}
		}

//		xTaskNotifyWait(
//				EVENT_I2C,
//				0x00,
//				&notify_value,
//				portMAX_DELAY);

		if (uxQueueSpacesAvailable(qIMUServerData)) {
			msg = pvPortMalloc(sizeof(struct IMUMessage_t));
			msg->accel_x = (int16_t)(((uint16_t)data[0] << 8) + data[1]);
			msg->accel_y = (int16_t)(((uint16_t)data[2] << 8) + data[3]);
			msg->accel_z = (int16_t)(((uint16_t)data[4] << 8) + data[5]);
			msg->gyro_x = (int16_t)(((uint16_t)data[8] << 8) + data[9]);
			msg->gyro_y = (int16_t)(((uint16_t)data[10] << 8) + data[11]);
			msg->gyro_z = (int16_t)(((uint16_t)data[12] << 8) + data[13]);
			msg->temperature = (int16_t)(((uint16_t)data[6] << 8) + data[7]);

			xQueueSend(qIMUServerData, (const void *)&msg, 0);
		}

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
	}
}

void IMUThread_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (imuTaskHandle != 0) {
		xTaskNotifyFromISR(imuTaskHandle, EVENT_IMU, eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (imuTaskHandle != 0) {
		xTaskNotifyFromISR(imuTaskHandle,
				EVENT_I2C,
				eSetBits,
				&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}
