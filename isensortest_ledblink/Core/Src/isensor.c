#include "isensor.h"


/* =========================================================
   INTERNAL WRITE REGISTER FUNCTION
   ========================================================= */

static uint8_t INA219_WriteRegister(I2C_HandleTypeDef *hi2c,
                                    uint8_t reg,
                                    uint16_t value)
{
    uint8_t data[3];


    /*
     * Check I2C handle
     */
    if (hi2c == NULL)
    {
        return 0U;
    }


    /*
     * Register address
     */
    data[0] = reg;


    /*
     * Register MSB
     */
    data[1] =
        (uint8_t)(value >> 8U);


    /*
     * Register LSB
     */
    data[2] =
        (uint8_t)(value & 0xFFU);


    /*
     * Transmit register
     */
    if (HAL_I2C_Master_Transmit(
            hi2c,
            INA219_ADDRESS,
            data,
            3U,
            100U) != HAL_OK)
    {
        return 0U;
    }


    return 1U;
}


/* =========================================================
   INTERNAL READ REGISTER FUNCTION
   ========================================================= */

static uint8_t INA219_ReadRegister(I2C_HandleTypeDef *hi2c,
                                   uint8_t reg,
                                   uint8_t *data)
{
    /*
     * Check parameters
     */
    if ((hi2c == NULL) ||
        (data == NULL))
    {
        return 0U;
    }


    /*
     * Select register
     */
    if (HAL_I2C_Master_Transmit(
            hi2c,
            INA219_ADDRESS,
            &reg,
            1U,
            100U) != HAL_OK)
    {
        return 0U;
    }


    /*
     * Read two bytes
     */
    if (HAL_I2C_Master_Receive(
            hi2c,
            INA219_ADDRESS,
            data,
            2U,
            100U) != HAL_OK)
    {
        return 0U;
    }


    return 1U;
}


/* =========================================================
   CHECK INA219 CONNECTION
   ========================================================= */

uint8_t INA219_IsConnected(I2C_HandleTypeDef *hi2c)
{
    /*
     * Check I2C handle
     */
    if (hi2c == NULL)
    {
        return 0U;
    }


    /*
     * Check INA219 at address 0x40
     */
    if (HAL_I2C_IsDeviceReady(
            hi2c,
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

uint8_t INA219_Init(I2C_HandleTypeDef *hi2c)
{
    /*
     * First check whether INA219 exists.
     */
    if (!INA219_IsConnected(hi2c))
    {
        return 0U;
    }


    /*
     * Configuration register = 0x399F
     *
     * BRNG = 32 V
     * PGA  = /8
     * BADC = 12-bit
     * SADC = 12-bit
     * MODE = continuous shunt + bus
     */
    if (!INA219_WriteRegister(
            hi2c,
            INA219_REG_CONFIG,
            0x399FU))
    {
        return 0U;
    }


    /*
     * Allow first conversion.
     */
    HAL_Delay(10U);


    return 1U;
}


/* =========================================================
   READ SHUNT VOLTAGE
   ========================================================= */

uint8_t INA219_ReadShuntVoltage(I2C_HandleTypeDef *hi2c,
                                float *shunt_mV)
{
    uint8_t data[2];

    int16_t raw_shunt;


    /*
     * Parameter check
     */
    if ((hi2c == NULL) ||
        (shunt_mV == NULL))
    {
        return 0U;
    }


    /*
     * Read shunt voltage register
     */
    if (!INA219_ReadRegister(
            hi2c,
            INA219_REG_SHUNT_VOLTAGE,
            data))
    {
        return 0U;
    }


    /*
     * Convert bytes to signed 16-bit value
     */
    raw_shunt =
        (int16_t)(((uint16_t)data[0] << 8U) |
                  data[1]);


    /*
     * INA219 shunt voltage LSB:
     *
     * 10 uV
     *
     * 10 uV = 0.01 mV
     */
    *shunt_mV =
        (float)raw_shunt * 0.01f;


    return 1U;
}


/* =========================================================
   READ CURRENT
   ========================================================= */

uint8_t INA219_ReadCurrent(I2C_HandleTypeDef *hi2c,
                           float *current_A)
{
    float shunt_mV;


    /*
     * Parameter check
     */
    if ((hi2c == NULL) ||
        (current_A == NULL))
    {
        return 0U;
    }


    /*
     * Read shunt voltage
     */
    if (!INA219_ReadShuntVoltage(
            hi2c,
            &shunt_mV))
    {
        return 0U;
    }


    /*
     * Current = V / R
     *
     * shunt_mV -> volts
     */
    *current_A =
        (shunt_mV / 1000.0f) /
        INA219_SHUNT_RESISTOR_OHM;


    /*
     * Make current positive for the
     * unidirectional 24 V supply measurement.
     */
    if (*current_A < 0.0f)
    {
        *current_A = -*current_A;
    }


    return 1U;
}


/* =========================================================
   READ BUS VOLTAGE
   ========================================================= */

uint8_t INA219_ReadBusVoltage(I2C_HandleTypeDef *hi2c,
                              float *voltage_V)
{
    uint8_t data[2];

    uint16_t raw_bus;


    /*
     * Parameter check
     */
    if ((hi2c == NULL) ||
        (voltage_V == NULL))
    {
        return 0U;
    }


    /*
     * Read bus-voltage register
     */
    if (!INA219_ReadRegister(
            hi2c,
            INA219_REG_BUS_VOLTAGE,
            data))
    {
        return 0U;
    }


    /*
     * Convert to 16-bit value
     */
    raw_bus =
        (uint16_t)(((uint16_t)data[0] << 8U) |
                   data[1]);


    /*
     * Bus voltage is stored in bits 15:3.
     */
    raw_bus >>= 3U;


    /*
     * Bus-voltage LSB = 4 mV
     */
    *voltage_V =
        (float)raw_bus * 0.004f;


    return 1U;
}


/* =========================================================
   READ POWER
   ========================================================= */

uint8_t INA219_ReadPower(I2C_HandleTypeDef *hi2c,
                         float *power_W)
{
    float voltage_V;
    float current_A;


    /*
     * Parameter check
     */
    if ((hi2c == NULL) ||
        (power_W == NULL))
    {
        return 0U;
    }


    /*
     * Read bus voltage
     */
    if (!INA219_ReadBusVoltage(
            hi2c,
            &voltage_V))
    {
        return 0U;
    }


    /*
     * Read current
     */
    if (!INA219_ReadCurrent(
            hi2c,
            &current_A))
    {
        return 0U;
    }


    /*
     * Power = Voltage x Current
     */
    *power_W =
        voltage_V * current_A;


    return 1U;
}