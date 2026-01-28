#ifndef I2C_CODE_H
#define I2C_CODE_H

#include "em_gpio.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include <stdint.h>


#define BMA400_I2C_ADDRESS        0x14 // I2C address for BMA400
#define BMA400_CHIP_ID_REG        0x00 // Chip ID register
#define BMA400_ACC_X_LSB_REG      0x04 // X-Axis LSB acceleration register
#define BMA400_ACC_CONFIG_REG_0     0x19 // Acceleration config register
#define BMA400_ACC_CONFIG_REG_1     0x1A

#define I2C_FREQ_STANDARD_MAX     100000

// TMP117 I2C address and register
#define TMP117_ADDRESS    0x48  // TMP117 I2C address
#define TMP117_TEMP_REG   0x00  // Temperature register

// Step counter registers
#define BMA400_STEP_CNT_CONFIG_0     0x35    // Step counter config register 0
#define BMA400_STEP_CNT_CONFIG_1     0x36    // Step counter config register 1
#define BMA400_STEP_CNT_CONFIG_2     0x37    // Set step detect timeout
#define BMA400_STEP_CNT_CONFIG_3     0x38    // Set step precision
#define BMA400_STEP_CNT_CONFIG_4     0x39    // Set step delta time
#define BMA400_STEP_COUNTER_VALUE    0x3B    // Step counter value register

// Accelerometer configuration registers
#define BMA400_ACC_CONFIG0           0x19    // Accelerometer power mode
#define BMA400_ACC_CONFIG1           0x1A    // Accelerometer range and ODR
#define BMA400_ACC_CONFIG2           0x1B    // Additional accelerometer settings

// Define GPIO configuration
#define BMA400_GPIO_PORT             gpioPortC
#define BMA400_GPIO_PIN              3       // PC03

typedef enum {
    BMA400_INTERRUPT_PIN1 = 1,
    BMA400_INTERRUPT_PIN2 = 2
} BMA400_InterruptPin;

// Function prototypes
void I2C_init(void);
uint8_t I2C_readByte(uint8_t, uint8_t);
void I2C_writeByte(uint8_t, uint8_t, uint8_t);
uint8_t BMA400_init(void);
void BMA400_readAccel(float*, float*, float*);
void convertToFloat(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, float*, float*, float*);

// Read temperature from TMP117 sensor via I2C0
uint16_t i2c_read_temperature(void);

// Convert raw TMP117 temperature reading to Celsius
float convertToCelsius(uint16_t);

void BMA400_initStepCounter(void);
void BMA400_enableStepInterrupt(void);
bool BMA400_isStepDetected(void);
void BMA400_configureGPIOInterrupt(void);
uint32_t BMA400_readStepCount(void);

#endif // I2C_CODE_H
