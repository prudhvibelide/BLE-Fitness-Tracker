/***************************************************************************//**
 * @file main.c
 * @brief main() function.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#include "sl_system_process_action.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "i2c_code.h"
#include "em_gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Timer configuration
#define TIMER_FREQ  1000 // 1 kHz for 1 ms resolution

volatile bool delayComplete = false;

// Global variables for time tracking
volatile uint8_t current_hours = 12;   // Start at 16:05
volatile uint8_t current_minutes = 01;
volatile uint8_t current_seconds = 0;
volatile uint16_t ms_counter = 0;

#define SSD1306_ADDR 0x3C
#define SSD1306_CMD  0x00
#define SSD1306_DATA 0x40

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

// SSD1306 commands
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETMULTIPLEX       0xA8
#define SSD1306_SETDISPLAYOFFSET   0xD3
#define SSD1306_SETSTARTLINE       0x40
#define SSD1306_CHARGEPUMP         0x8D
#define SSD1306_MEMORYMODE         0x20
#define SSD1306_SEGREMAP           0xA0
#define SSD1306_COMSCANDEC         0xC8
#define SSD1306_SETCOMPINS         0xDA
#define SSD1306_SETCONTRAST        0x81
#define SSD1306_SETPRECHARGE       0xD9
#define SSD1306_SETVCOMDETECT      0xDB
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_NORMALDISPLAY      0xA6
#define SSD1306_DISPLAYON          0xAF

// Display buffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

const uint8_t font5x8[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // Space (20)
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x60, 0x60, 0x00, 0x00, // . (2E)
    0x00, 0x66, 0x66, 0x00, 0x00, // : (3A)
    0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x02, 0x0C, 0x02, 0x7F, // M
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x0C, 0x02, 0x7F, // N
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x7E, 0x09, 0x09, 0x09, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36  // B
};


// Modified symbol patterns with clearer and larger designs
const uint8_t custom_symbols[] = {
    // Temperature symbol (thermometer) - 8x8 pattern
    0x18,  // ...##...
    0x24,  // ..#..#..
    0x24,  // ..#..#..
    0x24,  // ..#..#..
    0x3C,  // ..####..
    0x3C,  // ..####..
    0x3C,  // ..####..
    0x18,  // ...##...

    // Steps symbol (shoe) - 8x8 pattern
       0x1E,  // ...####.
       0x3F,  // ..######
       0x3F,  // ..######
       0x7E,  // .######.
       0x7C,  // .#####..
       0x78,  // .####...
       0x70,  // .###....
       0x60,  // .##.....

    // Heart symbol - 8x8 pattern (unchanged)
    0x66,  // .##..##.
    0xFF,  // ########
    0xFF,  // ########
    0x7E,  // .######.
    0x3C,  // ..####..
    0x18,  // ...##...
    0x00,  // ........
    0x00   // ........
};

  void uart_init(void) {
      // Enable clock for GPIO and USART0
    CMU_ClockEnable(cmuClock_GPIO, true);
      CMU_ClockEnable(cmuClock_USART0, true);

      // Configure PB02 as TX (push-pull) and PB03 as RX (input)
      GPIO_PinModeSet(gpioPortB, 2, gpioModePushPull, 1); // TX
      GPIO_PinModeSet(gpioPortB, 3, gpioModeInput, 0);    // RX

      // Initialize USART with default settings
      USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
      init.baudrate = 115200;  // Set baud rate
      USART_InitAsync(USART0, &init);

      // Route USART0 TX and RX to PB02 and PB03
      GPIO->USARTROUTE[0].TXROUTE = (gpioPortB << _GPIO_USART_TXROUTE_PORT_SHIFT) | (2 << _GPIO_USART_TXROUTE_PIN_SHIFT);
      GPIO->USARTROUTE[0].RXROUTE = (gpioPortB << _GPIO_USART_RXROUTE_PORT_SHIFT) | (3 << _GPIO_USART_RXROUTE_PIN_SHIFT);
      GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_RXPEN;

      // Enable USART0
      USART_Enable(USART0, usartEnable);
}

// Function to send a single character over UART
void uart_send_char(char c) {
    // Wait until the transmit buffer is empty
    while (!(USART0->STATUS & USART_STATUS_TXBL));

    // Transmit the character
    USART_Tx(USART0, c);
}

// Function to send a string over UART
void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

char buffer[12];  // Buffer to hold the string representation of the number

// Function to send a number as a string over UART
void uart_send_number(int number) {
 //   char buffer[12];  // Buffer to hold the string representation of the number
    sprintf(buffer, "%d", number);
    uart_send_string(buffer);
}


void timer_init(void) {
    // Enable clock for TIMER0
    CMU_ClockEnable(cmuClock_TIMER0, true);

    // Initialize the timer
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
   // timerInit.prescale = timerPrescale256; // Adjust prescaler if necessary
    timerInit.prescale = timerPrescale16; // Set prescaler to 16 for slower counting
    TIMER_Init(TIMER0, &timerInit);

    // Set up the timer for 1 ms overflow
    uint32_t topValue = (CMU_ClockFreqGet(cmuClock_SYSCLK) / 16 / TIMER_FREQ) - 1;
    TIMER_TopSet(TIMER0, topValue);


    // Enable timer interrupt
    TIMER_IntClear(TIMER0, TIMER_IF_OF); // Clear overflow flag
    TIMER_IntEnable(TIMER0, TIMER_IF_OF); // Enable overflow interrupt
    NVIC_EnableIRQ(TIMER0_IRQn); // Enable TIMER0 interrupt in NVIC
}

// Modified timer interrupt handler
void TIMER0_IRQHandler(void) {
    // Clear the interrupt flag
    TIMER_IntClear(TIMER0, TIMER_IF_OF);
    delayComplete = true;

    // Since timer is set for 1ms intervals
    ms_counter++;

    // Check if a second has passed
    if (ms_counter >= 1000) {
        ms_counter = 0;
        current_seconds++;

        // Update minutes
        if (current_seconds >= 60) {
            current_seconds = 0;
            current_minutes++;

            // Update hours
            if (current_minutes >= 60) {
                current_minutes = 0;
                current_hours++;

                // Wrap hours at 24
                if (current_hours >= 24) {
                    current_hours = 0;
                }
            }
        }
    }
}

void blocking_delay_ms(uint32_t ms) {
    delayComplete = false; // Reset delay completion flag

    // Start the timer
    TIMER_CounterSet(TIMER0, 0); // Reset the timer counter
    TIMER_Enable(TIMER0, true);   // Enable the timer

    // Wait for the delay to complete
    for (uint32_t i = 0; i < ms; i++) {
        while (!delayComplete); // Wait for the timer to overflow
        delayComplete = false;  // Reset for the next iteration
    }

    TIMER_Enable(TIMER0, false); // Disable the timer after the delay
}

void reverse(char* str, int len) {
    int start = 0;
    int end = len - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

// Helper function to convert an integer to a string
int intToStr(int num, char* str, int minDigits) {
    int i = 0;
    int isNegative = 0;

    // Handle negative numbers
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // Convert each digit to character
    do {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num);

    // Add leading zeros if needed
    while (i < minDigits) {
        str[i++] = '0';
    }

    // Add negative sign if necessary
    if (isNegative) {
        str[i++] = '-';
    }

    // Reverse the string since we built it backward
    reverse(str, i);

    // Return the length of the string
    return i;
}

// Function to convert a floating-point number to a string
void floatToStr(float n, char* res, int precision) {
    // Extract integer part
    int intPart = (int)n;

    // Extract fractional part
    float fracPart = n - (float)intPart;

    // Convert integer part to string
    int i = intToStr(intPart, res, 0);

    // Add the decimal point
    res[i] = '.';
    i++;

    // Handle fractional part
    for (int j = 0; j < precision; j++) {
        fracPart *= 10;
    }

    int fracIntPart = (int)(fracPart);

    // Convert fractional part to string
    i+= intToStr(fracIntPart, res + i, precision);

    // Add \r\n at the end
//        res[i++] = '\r';
//        res[i++] = '\n';

        // Null-terminate the string
        res[i] = '\0';
}


// Display buffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Function to write a command to SSD1306
void SSD1306_WriteCommand(uint8_t command) {
    I2C_writeByte(SSD1306_ADDR, SSD1306_CMD, command);
}

// Function to write data to SSD1306
void SSD1306_WriteData(uint8_t data) {
    I2C_writeByte(SSD1306_ADDR, SSD1306_DATA, data);
}

// Initialize the display
void SSD1306_Init(void) {
    blocking_delay_ms(100);  // Wait for VDD to stabilize

    SSD1306_WriteCommand(SSD1306_DISPLAYOFF);
    SSD1306_WriteCommand(SSD1306_SETDISPLAYCLOCKDIV);
    SSD1306_WriteCommand(0x80);  // Suggested ratio

    SSD1306_WriteCommand(SSD1306_SETMULTIPLEX);
    SSD1306_WriteCommand(0x3F);  // 64 MUX for 128x64

    SSD1306_WriteCommand(SSD1306_SETDISPLAYOFFSET);
    SSD1306_WriteCommand(0x00);

    SSD1306_WriteCommand(SSD1306_SETSTARTLINE | 0x00);

    SSD1306_WriteCommand(SSD1306_CHARGEPUMP);
    SSD1306_WriteCommand(0x14);  // Enable charge pump

    SSD1306_WriteCommand(SSD1306_MEMORYMODE);
    SSD1306_WriteCommand(0x00);  // Horizontal addressing

    SSD1306_WriteCommand(SSD1306_SEGREMAP | 0x01);
    SSD1306_WriteCommand(SSD1306_COMSCANDEC);

    SSD1306_WriteCommand(SSD1306_SETCOMPINS);
    SSD1306_WriteCommand(0x12);

    SSD1306_WriteCommand(SSD1306_SETCONTRAST);
    SSD1306_WriteCommand(0xCF);

    SSD1306_WriteCommand(SSD1306_SETPRECHARGE);
    SSD1306_WriteCommand(0xF1);

    SSD1306_WriteCommand(SSD1306_SETVCOMDETECT);
    SSD1306_WriteCommand(0x40);

    SSD1306_WriteCommand(SSD1306_DISPLAYALLON_RESUME);
    SSD1306_WriteCommand(SSD1306_NORMALDISPLAY);
    SSD1306_WriteCommand(SSD1306_DISPLAYON);
}

// Clear the display buffer
void SSD1306_Clear(void) {
    memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));
}

// Update the display with the buffer contents
void SSD1306_UpdateDisplay(void) {
    SSD1306_WriteCommand(0x20); // Set Memory Addressing Mode
    SSD1306_WriteCommand(0x00); // Horizontal addressing mode

    SSD1306_WriteCommand(0x21); // Set Column Address
    SSD1306_WriteCommand(0x00); // Start at column 0
    SSD1306_WriteCommand(SSD1306_WIDTH - 1);

    SSD1306_WriteCommand(0x22); // Set Page Address
    SSD1306_WriteCommand(0x00); // Start at page 0
    SSD1306_WriteCommand(7);    // End at page 7

    // Write the entire buffer
    for (uint16_t i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_WriteData(SSD1306_Buffer[i]);
    }
}

// Modified Clear Display function
void SSD1306_ClearDisplay(void) {
    // Clear the buffer
    memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer));

    // Reset display addressing mode
    SSD1306_WriteCommand(0x20);  // Set Memory Addressing Mode
    SSD1306_WriteCommand(0x00);  // Horizontal addressing mode

    // Set column address range
    SSD1306_WriteCommand(0x21);  // Set Column Address
    SSD1306_WriteCommand(0x00);  // Start at column 0
    SSD1306_WriteCommand(SSD1306_WIDTH - 1);  // End at last column

    // Set page address range
    SSD1306_WriteCommand(0x22);  // Set Page Address
    SSD1306_WriteCommand(0x00);  // Start at page 0
    SSD1306_WriteCommand(7);     // End at page 7

    // Write zeros to all display memory
    for (uint16_t i = 0; i < (SSD1306_WIDTH * SSD1306_HEIGHT / 8); i++) {
        SSD1306_WriteData(0x00);
    }
}

// Draw a pixel in the buffer
void SSD1306_DrawPixel(uint8_t x, uint8_t y) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    // Calculate the byte position in the buffer
    uint16_t byte_pos = x + (y / 8) * SSD1306_WIDTH;
    // Calculate the bit position within the byte
    uint8_t bit_pos = y % 8;

    // Set the pixel
    SSD1306_Buffer[byte_pos] |= (1 << bit_pos);
}

// Example function to draw a simple pattern
void SSD1306_DrawTestPattern(void) {
    SSD1306_Clear();

    // Draw a border
    for (uint8_t x = 0; x < SSD1306_WIDTH; x++) {
        SSD1306_DrawPixel(x, 0);
        SSD1306_DrawPixel(x, SSD1306_HEIGHT-1);
    }
    for (uint8_t y = 0; y < SSD1306_HEIGHT; y++) {
        SSD1306_DrawPixel(0, y);
        SSD1306_DrawPixel(SSD1306_WIDTH-1, y);
    }

    SSD1306_UpdateDisplay();
}

void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch) {
    int index;

    if (ch >= '0' && ch <= '9') {
        index = 1 + (ch - '0');
    } else if (ch == '.') {
        index = 11;
    } else if (ch == ':') {
        index = 12;
    } else if (ch == ' ') {
        index = 13;
    } else if (ch >= 'A' && ch <= 'Z') {
        switch(ch) {
            case 'P': index = 14; break;
            case 'E': index = 15; break;
            case 'M': index = 16; break;
            case 'T': index = 17; break;
            case 'U': index = 18; break;
            case 'K': index = 19; break;
            case 'H': index = 20; break;
            case 'D': index = 21; break;
            case 'C': index = 22; break;
            case 'L': index = 23; break;
            case 'N': index = 24; break;
            case 'R': index = 25; break;
            case 'S': index = 26; break;
            case 'A': index = 27; break;
            case 'B': index = 28; break;
            default: return;
        }
    } else if (ch >= 'a' && ch <= 'z') {
        ch = ch - 'a' + 'A';
        SSD1306_DrawChar(x, y, ch);
        return;
    } else {
        return;
    }

    const uint8_t* char_pixels = &font5x8[index * 5];
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = char_pixels[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                SSD1306_DrawPixel(x + i, y + j);
            }
        }
    }
}

// Function to print a string
void SSD1306_PrintString(uint8_t x, uint8_t y, const char* str) {
    uint8_t x_pos = x;
    while(*str) {
        SSD1306_DrawChar(x_pos, y, *str);
        x_pos += 6; // 5 pixels for character + 1 pixel space
        str++;

        // Wrap text if it reaches the end of the display
        if(x_pos > SSD1306_WIDTH - 6) {
            x_pos = 0;
            y += 9; // Move to next line (8 pixels height + 1 pixel space)
            if(y > SSD1306_HEIGHT - 8) break; // Stop if we reach bottom of screen
        }
    }
}

void float_to_string(float value, char* buffer, int precision) {
    int intPart = (int)value;
    float decPart = value - intPart;

    // Convert integer part
    int i = 0;
    if (intPart == 0) {
        buffer[i++] = '0';
    } else {
        int temp = intPart;
        while (temp > 0) {
            temp /= 10;
            i++;
        }
        temp = intPart;
        int j = i - 1;
        while (temp > 0) {
            buffer[j--] = '0' + (temp % 10);
            temp /= 10;
        }
    }

    // Add decimal point
    buffer[i++] = '.';

    // Convert decimal part
    for (int j = 0; j < precision; j++) {
        decPart *= 10;
        int digit = (int)decPart;
        buffer[i++] = '0' + digit;
        decPart -= digit;
    }

    buffer[i] = '\0';
}

uint8_t get_string_length(const char* str) {
    uint8_t length = 0;
    while(str[length] != '\0') {
        length++;
    }
    return length;
}

// Function to calculate horizontal center position for text
uint8_t get_center_position_x(const char* str) {
    uint8_t length = get_string_length(str);
    // Each character is 6 pixels wide (5 pixels + 1 space)
    uint8_t string_width = length * 6;
    // Center position = (display width - string width) / 2
    if (string_width >= SSD1306_WIDTH) {
        return 0;
    }
    return (SSD1306_WIDTH - string_width) / 2;
}

// Modified function to draw a custom symbol
void SSD1306_DrawSymbol(uint8_t x, uint8_t y, uint8_t symbol_index) {
    // Each symbol is 8 bytes (8x8 pixels)
    const uint8_t* symbol_pixels = &custom_symbols[symbol_index * 8];

    // Draw the 8x8 symbol
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t line = symbol_pixels[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << (7-j))) {  // MSB first
                SSD1306_DrawPixel(x + j, y + i);
            }
        }
    }
}


// Get time string function
void get_time_string(char* buffer) {
    sprintf(buffer, "%02d:%02d", current_hours, current_minutes);
}

uint8_t get_random_bpm() {
    return 78 + (rand() % 23); // Random value between 78-100 for BPM
}

void display_all_parameters_OLED(float temp_value, uint16_t steps) {
    char float_buffer[16];
    char display_buffer[32];
    char time_buffer[6];
    uint8_t text_x;

    SSD1306_Clear();

    // Time at top
    get_time_string(time_buffer);
    text_x = (SSD1306_WIDTH - (strlen(time_buffer) * 6)) / 2;
    SSD1306_PrintString(text_x, 0, time_buffer);

    // Temperature line (moved down)
    float_to_string(temp_value, float_buffer, 2);
    strcpy(display_buffer, "TEMP: ");
    strcat(display_buffer, float_buffer);
    text_x = (SSD1306_WIDTH - (strlen(display_buffer) * 6)) / 2;

    // Draw temperature symbol and text with more space
    SSD1306_DrawSymbol(text_x - 12, 16, 0);  // Temperature symbol
    SSD1306_PrintString(text_x, 16, display_buffer);

    sprintf(display_buffer, "STEPS: %u", steps);
    text_x = (SSD1306_WIDTH - (strlen(display_buffer) * 6)) / 2;

    // Draw steps symbol and text with more space
    SSD1306_DrawSymbol(text_x - 12, 32, 1);  // Steps symbol
    SSD1306_PrintString(text_x, 32, display_buffer);

    // BPM line
    uint8_t bpm = get_random_bpm();
    sprintf(display_buffer, "BPM: %u", bpm);
    text_x = (SSD1306_WIDTH - (strlen(display_buffer) * 6)) / 2;

    // Draw heart symbol and text with more space
    SSD1306_DrawSymbol(text_x - 12, 48, 2);  // Heart symbol
    SSD1306_PrintString(text_x, 48, display_buffer);

    SSD1306_UpdateDisplay();
}

//volatile uint32_t currentSteps = 0;

//// Example interrupt handler (implement in your main application)
//void GPIO_EVEN_IRQHandler(void) {
//  uint32_t flags = GPIO_IntGet();
//
//      if (flags & (1 << BMA400_GPIO_PIN)) {
//          if (BMA400_isStepDetected()) {
//            // A new step has been detected
//            // Add your step processing code here
//            currentSteps = BMA400_readStepCount();
//
//            // Clear the interrupt status by reading it
//            (void)I2C_readByte(BMA400_I2C_ADDRESS, BMA400_INT_STATUS);
//        }
//    }
//
//    uart_send_string("INT\r\n");
//
//    GPIO_IntClear(flags);
//}

bool BMA400_verifyConfiguration(void) {
    uint8_t power_mode = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG0);
    uint8_t acc_config = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_ACC_CONFIG1);
    uint8_t step_enable = I2C_readByte(BMA400_I2C_ADDRESS, BMA400_STEP_CNT_CONFIG_0);

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Power Mode: 0x%02X (should be 0x02)\r\n", power_mode);
    uart_send_string(buffer);

    snprintf(buffer, sizeof(buffer), "Acc Config: 0x%02X (should be 0x09)\r\n", acc_config);
    uart_send_string(buffer);

    snprintf(buffer, sizeof(buffer), "Step Enable: 0x%02X (should be 0x02)\r\n", step_enable);
    uart_send_string(buffer);

    return (power_mode == 0x02) && (acc_config == 0x09) && (step_enable == 0x02);
}

int main(void)
{
  sl_system_init();
  uart_init();
  uart_send_string("Code starting Now \r\n");
  timer_init();
  I2C_init();
  blocking_delay_ms(100);


  uint8_t errorCode = BMA400_init();
  if(errorCode != 0x90) {
      uart_send_number(errorCode);
      uart_send_string("Chip ID not matched !!\r\n");
      return 1;
  }

  BMA400_initStepCounter();

  // Verify configuration
  if (!BMA400_verifyConfiguration()) {
      uart_send_string("BMA400 configuration failed!\r\n");
  }

//  BMA400_enableStepInterrupt();
//  BMA400_configureGPIOInterrupt();

  uart_send_string("Reading values from Sensors:\r\n");
  //timer_init();
  blocking_delay_ms(1000);
  float x, y, z;
  floatToStr(-7.82, buffer, 2);
  uart_send_string(buffer);
  uart_send_string("\r\n");

  //uint16_t steps = 0;
  float temperatureC = 29.0;

  SSD1306_Init();

      SSD1306_ClearDisplay();

         // Draw test pattern
         SSD1306_DrawTestPattern();


  while (1) {
      BMA400_readAccel(&x, &y, &z);
      uint32_t steps = BMA400_readStepCount();
      floatToStr(steps, buffer, 2);
      uart_send_string("steps :");
      uart_send_string(buffer);

//      floatToStr(y, buffer, 2);
//      uart_send_string(", y:");
//      uart_send_string(buffer);
//
//      floatToStr(z, buffer, 2);
//      uart_send_string(", z:");
//      uart_send_string(buffer);

      uart_send_string("\r\n");
      blocking_delay_ms(500);

      uint16_t temperatureRaw = i2c_read_temperature();
      if (temperatureRaw != 0xFFFF) {
           temperatureC = convertToCelsius(temperatureRaw);
           floatToStr(temperatureC, buffer, 2);
           uart_send_string("Temperature: ");
           uart_send_string(buffer);
           uart_send_string("\r\n");
         }

      //steps = (uint16_t) x;
      display_all_parameters_OLED(temperatureC, steps);
      blocking_delay_ms(2000);
  }
 }
