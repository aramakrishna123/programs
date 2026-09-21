#ifndef ISENSOR_H
#define ISENSOR_H

#include "main.h"
#include <stdint.h>

/* =========================================================
   INA219 ADDRESS
   ========================================================= */

#define INA219_ADDRESS              (0x40U << 1U)


/* =========================================================
   INA219 REGISTERS
   ========================================================= */

#define INA219_REG_CONFIG           0x00U
#define INA219_REG_SHUNT_VOLTAGE    0x01U
#define INA219_REG_BUS_VOLTAGE      0x02U
#define INA219_REG_POWER            0x03U
#define INA219_REG_CURRENT          0x04U
#define INA219_REG_CALIBRATION      0x05U


/* =========================================================
   INA219 SETTINGS
   ========================================================= */

#define INA219_SHUNT_RESISTOR_OHM   0.1f

#define INA219_CURRENT_LIMIT_A      0.300f


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

uint8_t INA219_IsConnected(void);

HAL_StatusTypeDef INA219_Init(void);

HAL_StatusTypeDef INA219_ReadShuntVoltage(float *voltage_V);

HAL_StatusTypeDef INA219_ReadBusVoltage(float *voltage_V);

HAL_StatusTypeDef INA219_ReadCurrent(float *current_A);

HAL_StatusTypeDef INA219_ReadPower(float *power_W);


#endif