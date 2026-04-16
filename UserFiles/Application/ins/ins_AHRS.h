#ifndef INS_AHRS_H
#define INS_AHRS_H

#include "bmi088.h"
#include "bsp_dwt.h"
#include "main.h"

#include <stdint.h>

/**
 * @brief 初始化INS传感器模块
 * @details 初始化BMI088加速度计和陀螺仪，IST8310磁力计，
 *          以及四元数、积分项和陀螺仪校准相关变量
 * @retval 无
 * @note 必须在系统启动时调用一次
 */
void INS_Init(void);

/**
 * @brief 获取欧拉角（偏航角、俯仰角、滚转角）
 * @param[out] angles 指向浮点数数组的指针，用于存储欧拉角
 *                    angles[0] = 偏航角(Yaw)，单位：度
 *                    angles[1] = 俯仰角(Pitch)，单位：度
 *                    angles[2] = 滚转角(Roll)，单位：度
 * @retval 无
 * @note 调用此函数会自动更新四元数和传感器数据
 */
void INS_getYawPitchRoll(float *angles);

/**
 * @brief 获取累计偏航角 (Total Yaw Angle)
 * @return 累计偏航角，单位：度，范围：无限制 (连续)
 */
float INS_getYawTotalAngle(void);

/**
 * @brief 获取陀螺仪角速度数据
 * @param[out] gyro 指向浮点数数组的指针，用于存储角速度
 *                  gyro[0] = X轴角速度(Roll方向)，单位：度/秒
 *                  gyro[1] = Y轴角速度(Pitch方向)，单位：度/秒
 *                  gyro[2] = Z轴角速度(Yaw方向)，单位：度/秒
 * @retval 无
 * @note 返回的是校准后的陀螺仪数据
 */
void INS_getGyroData(float *gyro);

/**
 * @brief 获取陀螺仪经过处理后的数据
 * @param[out] values 指向浮点数数组的指针
 * @retval 无
 * @note 返回的是校准后的陀螺仪数据
 */
void INS_getValues(float *values);

#endif