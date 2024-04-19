
#ifndef INC_LCD_H_
#define INC_LCD_H_

// included to use GPIO_TypeDef
#include "stm32l476xx.h"
// included to use uint16_t
#include <stdint.h>

typedef struct {
        union{
                struct{
                        uint16_t RS;
                        uint16_t RW;
                        uint16_t E;
                        // NOTE: could optimize with a single uint16_t
                        uint16_t D7;
                        uint16_t D6;
                        uint16_t D5;
                        uint16_t D4;
                };
                uint16_t array[7];
        }pins;
        GPIO_TypeDef * PORT;
} LCD_t;

//Define Macros for LCD control bits
//#define RS GPIO_PIN_6
//#define RW GPIO_PIN_4
//#define E GPIO_PIN_5
//#define DATA_OUT GPIOC

//Define Mask Macros
#define LCD_UPPER_NIB_MSK 0xF0
#define LCD_LOWER_NIB_MSK 0x0F

//Define LCD Command Macros
#define LCD_COMMAND_RETURN_HOME 0x80
#define LCD_COMMAND_CLEAR_DISPLAY 0x01
#define LCD_COMMAND_NEW_LINE 0xc0

//Define Functions
void delay_us(const uint32_t time_us);
void SysTick_Init(void);
void LCD_write_command_byte(LCD_t lcd, uint8_t LCD_command);
void LCD_GPIO_Init(LCD_t lcd);
void LCD_screen_init(LCD_t lcd);
void LCD_write_string(LCD_t lcd, char LCD_input_string[]);

#endif /* INC_LCD_H_ */
