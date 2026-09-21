#include "isensor.h"


/* =========================================================
   I2C HANDLE
   ========================================================= */

extern I2C_HandleTypeDef hi2c1;


/* =========================================================
   WRITE REGISTER
   ========================================================= */

static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg,
                                              uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)(value & 0xFFU);

    return HAL_I2C_Mem_Write(&hi2c1,
                             INA219_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             2U,
                             100U);
}


/* =========================================================
   READ REGISTER
   ========================================================= */

static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg,
                                             uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c1,
                              INA219_ADDRESS,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              2U,
                              100U);

    if (status != HAL_OK)
    {
        return status;
    }

    *value = ((uint16_t)data[0] << 8U) |
             (uint16_t)data[1];

    return HAL_OK;
}


/* =========================================================
   CHECK INA219
   ========================================================= */

uint8_t INA219_IsConnected(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1,
                              INA219_ADDRESS,
                              3U,
                              100U) == HAL_OK)
    {
        return 1U;
    }

    return 0U;
}


/* =========================================================
   INA219 INITIALIZATION
   ========================================================= */

HAL_StatusTypeDef INA219_Init(void)
{
    HAL_StatusTypeDef status;

    /*
     * Configuration = 0x399F
     *
     * 32V bus range
     * PGA /8
     * 12-bit shunt ADC
     * 12-bit bus ADC
     * Continuous conversion
     */

    status = INA219_WriteRegister(INA219_REG_CONFIG,
                                  0x399FU);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10U);

    return HAL_OK;
}


/* =========================================================
   READ SHUNT VOLTAGE
   ========================================================= */

HAL_StatusTypeDef INA219_ReadShuntVoltage(float *voltage_V)
{
    uint16_t raw;
    int16_t signed_raw;

    if (voltage_V == NULL)
    {
        return HAL_ERROR;
    }

    if (INA219_ReadRegister(INA219_REG_SHUNT_VOLTAGE,
                            &raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    signed_raw = (int16_t)raw;

    /*
     * 1 LSB = 10 uV
     */

    *voltage_V =
        (float)signed_raw * 0.00001f;

    return HAL_OK;
}


/* =========================================================
   READ BUS VOLTAGE
   ========================================================= */

HAL_StatusTypeDef INA219_ReadBusVoltage(float *voltage_V)
{
    uint16_t raw;

    if (voltage_V == NULL)
    {
        return HAL_ERROR;
    }

    if (INA219_ReadRegister(INA219_REG_BUS_VOLTAGE,
                            &raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Bus voltage data = bits 15:3
     */

    raw >>= 3U;

    /*
     * 1 LSB = 4 mV
     */

    *voltage_V =
        (float)raw * 0.004f;

    return HAL_OK;
}


/* =========================================================
   READ CURRENT
   ========================================================= */

HAL_StatusTypeDef INA219_ReadCurrent(float *current_A)
{
    float shunt_voltage_V;

    if (current_A == NULL)
    {
        return HAL_ERROR;
    }

    if (INA219_ReadShuntVoltage(&shunt_voltage_V)
        != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Current = Shunt voltage / Shunt resistance
     *
     * R = 0.1 ohm
     */

    *current_A =
        shunt_voltage_V /
        INA219_SHUNT_RESISTOR_OHM;


    /*
     * Keep current positive for this application.
     */

    if (*current_A < 0.0f)
    {
        *current_A = -*current_A;
    }

    return HAL_OK;
}


/* =========================================================
   READ POWER
   ========================================================= */

HAL_StatusTypeDef INA219_ReadPower(float *power_W)
{
    float voltage_V;
    float current_A;

    if (power_W == NULL)
    {
        return HAL_ERROR;
    }

    if (INA219_ReadBusVoltage(&voltage_V)
        != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (INA219_ReadCurrent(&current_A)
        != HAL_OK)
    {
        return HAL_ERROR;
    }

    *power_W =
        voltage_V * current_A;

    return HAL_OK;
}