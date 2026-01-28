#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "i2c_code.h"


// Configure SCL and SDA for I2C0 on PC08 (SCL) and PC09 (SDA)
void I2C_init(void) {
  // Enable clock for GPIO and I2C0
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_I2C0, true);

  // Configure PB01 (SCL) and PB05 (SDA) pins for I2C (open-drain with pull-up)
  GPIO_PinModeSet(gpioPortB, 5, gpioModeWiredAndPullUp, 1); // SDA
  GPIO_PinModeSet(gpioPortB, 1, gpioModeWiredAndPullUp, 1); // SCL

  // Initialize I2C0 with default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  I2C_Init(I2C0, &i2cInit);

  // Route I2C0 signals to PB05 (SDA) and PB01 (SCL)
  GPIO->I2CROUTE[0].SDAROUTE = (gpioPortB << _GPIO_I2C_SDAROUTE_PORT_SHIFT) | (5 << _GPIO_I2C_SDAROUTE_PIN_SHIFT);
  GPIO->I2CROUTE[0].SCLROUTE = (gpioPortB << _GPIO_I2C_SCLROUTE_PORT_SHIFT) | (1 << _GPIO_I2C_SCLROUTE_PIN_SHIFT);
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

  // Enable I2C0
  I2C_Enable(I2C0, true);
}

uint8_t I2C_readByte(uint8_t addr, uint8_t reg) {
    I2C_TransferSeq_TypeDef seq;
    uint8_t data[1];
    seq.addr = addr << 1;
    seq.flags = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = &reg;
    seq.buf[0].len = 1;
    seq.buf[1].data = data;
    seq.buf[1].len = 1;

    I2C_TransferInit(I2C0, &seq);
    while (I2C_Transfer(I2C0) == i2cTransferInProgress);

    return data[0];
}

void I2C_writeByte(uint8_t addr, uint8_t reg, uint8_t value) {
    I2C_TransferSeq_TypeDef seq;
    uint8_t data[2] = {reg, value};
    seq.addr = addr << 1;
    seq.flags = I2C_FLAG_WRITE;
    seq.buf[0].data = data;
    seq.buf[0].len = 2;

    I2C_TransferInit(I2C0, &seq);
    while (I2C_Transfer(I2C0) == i2cTransferInProgress);
}

uint8_t BMA400_init(void) {
    uint8_t chipId = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_CHIP_ID_REG);
    if (chipId != 0x90) {
        // Handle error
        return chipId;
    }

    // Set accelerometer configuration (e.g., range, bandwidth, etc.)
   // I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG_REG, 0x10);                  // Example configuration
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG_REG_0, 0x02); // Set to normal mode
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG_REG_1, 0x09); // 0x09 for ±2g
    return chipId;
}

void BMA400_readAccel(float *fx, float *fy, float *fz) {
    uint8_t xLsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_X_LSB_REG);
    uint8_t xMsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_X_LSB_REG + 1);
    uint8_t yLsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_X_LSB_REG + 2);
    uint8_t yMsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_X_LSB_REG + 3);
    uint8_t zLsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_X_LSB_REG + 4);
    uint8_t zMsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_X_LSB_REG + 5);

    // Convert to float directly
    convertToFloat(xLsb, xMsb, yLsb, yMsb, zLsb, zMsb, fx, fy, fz);
}

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH     (9.80665f)
#define g_range           (2)
#define bit_width         (12)

void convertToFloat(uint8_t xLsb, uint8_t xMsb, uint8_t yLsb, uint8_t yMsb, uint8_t zLsb, uint8_t zMsb, float *fx, float *fy, float *fz) {
    // Combine MSB and LSB for each axis
    int16_t x = (xMsb << 8) | xLsb; // X-axis
    int16_t y = (yMsb << 8) | yLsb; // Y-axis
    int16_t z = (zMsb << 8) | zLsb; // Z-axis

//    float conversionFactor = 0.061f;
    //*fx = x * conversionFactor; // Convert to g
    int16_t half_scale;

     half_scale = 1 << (bit_width - 1);

    *fx = (GRAVITY_EARTH * x * g_range) / half_scale;
    *fx -= (1.0);
    *fy = (GRAVITY_EARTH * y * g_range) / half_scale;; // Convert to g
    *fy -= (38.0);
    *fz = (GRAVITY_EARTH * z * g_range) / half_scale;; // Convert to g
}


void i2c0_init(void) {
    // Enable clock for GPIO and I2C0
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_I2C0, true);

    // Configure SCL (PC08) and SDA (PC09) pins for I2C (open-drain with pull-up)
    GPIO_PinModeSet(gpioPortC, 9, gpioModeWiredAndPullUp, 1); // SDA
    GPIO_PinModeSet(gpioPortC, 8, gpioModeWiredAndPullUp, 1); // SCL

    // Initialize I2C0 with default settings
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
    I2C_Init(I2C0, &i2cInit);

    // Route I2C0 signals to PC09 (SDA) and PC08 (SCL)
    GPIO->I2CROUTE[0].SDAROUTE = (gpioPortC << _GPIO_I2C_SDAROUTE_PORT_SHIFT) | (9 << _GPIO_I2C_SDAROUTE_PIN_SHIFT);
    GPIO->I2CROUTE[0].SCLROUTE = (gpioPortC << _GPIO_I2C_SCLROUTE_PORT_SHIFT) | (8 << _GPIO_I2C_SCLROUTE_PIN_SHIFT);
    GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

    // Enable I2C0
    I2C_Enable(I2C0, true);
}

// Function to write data to TMP117 (send register address)
I2C_TransferReturn_TypeDef i2c_write(uint8_t address, uint8_t reg) {
    I2C_TransferSeq_TypeDef seq;
    seq.addr = address << 1;  // TMP117 address with write bit (left-shifted by 1)
    seq.flags = I2C_FLAG_WRITE;

    // Buffer to hold register address to write to TMP117
    seq.buf[0].data = &reg;
    seq.buf[0].len = 1;

    // Perform the I2C transfer
    return I2C_TransferInit(I2C0, &seq);
}

// Function to read temperature from TMP117 sensor
uint16_t i2c_read_temperature(void) {
    uint8_t readData[2] = {0};

    // I2C Transfer sequence for read operation
    I2C_TransferSeq_TypeDef seq;
    seq.addr = TMP117_ADDRESS << 1;
    seq.flags = I2C_FLAG_WRITE_READ;

    // Write the register address to the sensor (temperature register)
    uint8_t tmp117RegAddr = TMP117_TEMP_REG;
    seq.buf[0].data = &tmp117RegAddr;
    seq.buf[0].len = 1;

    // Prepare to read 2 bytes from the sensor (temperature data)
    seq.buf[1].data = readData;
    seq.buf[1].len = 2;

    // Start the I2C transfer
    I2C_TransferReturn_TypeDef result = I2C_TransferInit(I2C0, &seq);
    while (result == i2cTransferInProgress) {
        result = I2C_Transfer(I2C0);
    }

    if (result != i2cTransferDone) {
        return 0xFFFF;  // Return error code if the transfer fails
    }

    // Combine MSB and LSB of temperature data
    uint16_t temperatureRaw = (readData[0] << 8) | readData[1];

    return temperatureRaw;
}

// Convert raw TMP117 temperature reading to Celsius
float convertToCelsius(uint16_t rawTemp) {
    return rawTemp * 0.0078125f;  // resolution of 0.0078125°C per bit
}

void BMA400_initStepCounter(void) {
    // First ensure accelerometer is in normal power mode with proper configuration
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG0, 0x02);  // Set normal power mode

    // Configure accelerometer: ±2g range, 100Hz sampling
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG1, 0x09);  // ODR = 100Hz, Range = ±2g

    // Short delay after power mode change
    for(volatile int i = 0; i < 1000; i++);

    // Configure step counter parameters
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_0, 0x02);  // Step counter algorithm enabled
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_1, 0x25);  // Adjusted step threshold
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_2, 0x14);  // Step detect timeout
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_3, 0x40);  // Step precision
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_4, 0x1C);  // Step delta time
}


uint32_t BMA400_readStepCount(void) {
    uint32_t steps = 0;

    // Step counter value is stored in 3 bytes (LSB, MID, MSB)
    uint8_t lsb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_STEP_COUNTER_VALUE);
    uint8_t mid = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_STEP_COUNTER_VALUE + 1);
    uint8_t msb = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_STEP_COUNTER_VALUE + 2);

    // Combine the bytes to get total step count
    steps = (msb << 16) | (mid << 8) | lsb;

    return steps;
}

void BMA400_resetStepCounter(void) {
    // Temporarily disable step counter
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_0, 0x00);

    // Small delay to ensure the counter is disabled
    for(volatile int i = 0; i < 1000; i++);

    // Re-enable step counter (this resets the count)
    I2C_writeByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_0, 0x03);
}


