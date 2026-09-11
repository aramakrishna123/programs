#ifndef INA219_H
#define INA219_H

#include "main.h"
#include <stdint.h>

/* =========================================================
   INA219 I2C ADDRESS
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
   SHUNT RESISTOR
   ========================================================= */

/*
 * 7Semi INA219 module
 *
 * Shunt resistor = 0.1 ohm
 */
#define INA219_SHUNT_RESISTOR_OHM   0.1f


/* =========================================================
   CURRENT LIMIT
   ========================================================= */

/*
 * Change ONLY this value when you want
 * to change the overcurrent limit.
 *
 * 0.300f = 300 mA
 * 0.400f = 400 mA
 * 0.500f = 500 mA
 *
 * This is the current measured in the
 * 24 V supply line going to the DM556.
 */
#define INA219_CURRENT_LIMIT_A      0.300f


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

/*
 * Check whether INA219 is connected.
 *
 * Return:
 * 1 = connected
 * 0 = not connected
 */
uint8_t INA219_IsConnected(I2C_HandleTypeDef *hi2c);


/*
 * Initialize INA219.
 *
 * Return:
 * 1 = success
 * 0 = failure
 */
uint8_t INA219_Init(I2C_HandleTypeDef *hi2c);


/*
 * Read current in amperes.
 *
 * Example:
 *
 * float current_A;
 *
 * INA219_ReadCurrent(&hi2c1, &current_A);
 */
uint8_t INA219_ReadCurrent(I2C_HandleTypeDef *hi2c,
                           float *current_A);


/*
 * Read bus voltage in volts.
 *
 * Example:
 *
 * float voltage_V;
 *
 * INA219_ReadBusVoltage(&hi2c1, &voltage_V);
 */
uint8_t INA219_ReadBusVoltage(I2C_HandleTypeDef *hi2c,
                              float *voltage_V);


/*
 * Read shunt voltage in millivolts.
 *
 * Example:
 *
 * float shunt_mV;
 *
 * INA219_ReadShuntVoltage(&hi2c1, &shunt_mV);
 */
uint8_t INA219_ReadShuntVoltage(I2C_HandleTypeDef *hi2c,
                                float *shunt_mV);


/*
 * Read calculated power in watts.
 *
 * Example:
 *
 * float power_W;
 *
 * INA219_ReadPower(&hi2c1, &power_W);
 */
uint8_t INA219_ReadPower(I2C_HandleTypeDef *hi2c,
                         float *power_W);

#endif /* INA219_H */