/*The following functions are used to initialize and send data
 * to the NHD-0216HZ-FSW-FBW-33V3C LCD. Their purposes are described therein.
 */

//Include necessary libraries
#include "LCD.h"
#include <stdint.h>

//Initializes MPU GPIO for LCD
void LCD_GPIO_Init(LCD_t lcd)
{
        //Configure GPIOC Pins {GPIO interface between LCD and MPU (MPU_Data_Bus)
        //Connect GPIOC to clock
        RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);
        //Set MODER Mask = 0
        lcd.PORT->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 |GPIO_MODER_MODE2
                |GPIO_MODER_MODE3 | GPIO_MODER_MODE4 |GPIO_MODER_MODE5| GPIO_MODER_MODE6);
        //Set MPU_Data_Bus to output mode with MODER[01]
        lcd.PORT->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0
                |GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 |GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0);
        lcd.PORT->MODER &= ~(GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE2_1
                |GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |GPIO_MODER_MODE6_1);
        //Set the output type to Push-Pull: OT = [0]
        lcd.PORT->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3
                |GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6);
        //Set MPU_Data_Bus to NO PULL UP/PULL DOWN: PUPD registers = [00]
        lcd.PORT->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2
                | GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6);
        //Sets OSPEED to 3 which is "Very high Speed"
        lcd.PORT->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |
                (3 << GPIO_OSPEEDR_OSPEED1_Pos)|(3 << GPIO_OSPEEDR_OSPEED2_Pos) |
                (3 << GPIO_OSPEEDR_OSPEED3_Pos) | (3 << GPIO_OSPEEDR_OSPEED4_Pos) |
                (3 << GPIO_OSPEEDR_OSPEED5_Pos)|(3 << GPIO_OSPEEDR_OSPEED6_Pos));
}


// Write a byte of data to the LCD
// This is in 4-bit mode
void LCD_write_command_byte(LCD_t lcd, uint8_t LCD_command)
{
        //Reset RS for command Register Select and RW for Write
        lcd.PORT->BRR = lcd.pins.RS;
        lcd.PORT->BRR = lcd.pins.RW;

        //Clear lcd.PORT
        lcd.PORT->BRR = (lcd.pins.D4| lcd.pins.D5| lcd.pins.D6| lcd.pins.D7);

        //Set E to enable data transfer
        lcd.PORT->BSRR = lcd.pins.E;

        //Delay to account for rise time
        delay_us(1);

        //Output upper nibble
        lcd.PORT->ODR |= ((LCD_command & LCD_UPPER_NIB_MSK)>>4);

        //Delay to allow for data transfer
        delay_us(1);

        //Reset E to end data transfer
        lcd.PORT->BRR = lcd.pins.E;

        //Delay data clear for address hold
        delay_us(1);

        //Clear lcd.PORT
        lcd.PORT->BRR = (lcd.pins.D4| lcd.pins.D5| lcd.pins.D6| lcd.pins.D7);

        //Set E to enable data transfer
        lcd.PORT->BSRR = lcd.pins.E;

        //Delay to account for rise time
        delay_us(1);

        //Output lower nibble
        lcd.PORT->ODR |= (LCD_command & LCD_LOWER_NIB_MSK);

        //Delay to allow for data transfer
        delay_us(1);

        //Reset E to end data transfer
        lcd.PORT->BRR = lcd.pins.E;

        //Delay data clear for address hold
        delay_us(1);
        //Clear lcd.PORT
        lcd.PORT->BRR = (lcd.pins.D4| lcd.pins.D5| lcd.pins.D6| lcd.pins.D7);
}

void LCD_write_byte(LCD_t lcd, uint8_t LCD_input_byte)
{
        // Set RS to do data transfer
        lcd.PORT->BSRR = lcd.pins.RS;
        //Clear lcd.PORT
        lcd.PORT->BRR = (lcd.pins.D4| lcd.pins.D5| lcd.pins.D6| lcd.pins.D7);

        //Set E to enable data transfer
        lcd.PORT->BSRR = lcd.pins.E;
        //Delay to account for rise time
        delay_us(1);
        //Output upper nibble
        lcd.PORT->ODR |= ((LCD_input_byte & LCD_UPPER_NIB_MSK)>>4);
        //Delay to allow for data transfer
        delay_us(1);
        //Reset E to end data transfer
        lcd.PORT->BRR = lcd.pins.E;

        //Delay data clear for address hold
        delay_us(1);
        //Clear lcd.PORT
        lcd.PORT->BRR = (lcd.pins.D4| lcd.pins.D5| lcd.pins.D6| lcd.pins.D7);

        //Set E to enable data transfer
        lcd.PORT->BSRR = lcd.pins.E;
        //Delay to account for rise time
        delay_us(1);
        //Output lower nibble
        lcd.PORT->ODR |= (LCD_input_byte & LCD_LOWER_NIB_MSK);
        //Delay to allow for data transfer
        delay_us(1);
        //Reset E to end data transfer
        lcd.PORT->BRR = lcd.pins.E;

        //Delay data clear for address hold
        delay_us(1);
        //Clear lcd.PORT
        lcd.PORT->BRR = (lcd.pins.D4| lcd.pins.D5| lcd.pins.D6| lcd.pins.D7);
}

void LCD_write_string(LCD_t lcd, char LCD_input_string[])
{
        uint8_t cursor= 0;\
        //Iterate through the string
        //If the line is longer than 15 characters, the rest of the characters will be ignored
        for(cursor= 0; (cursor < 16) && (LCD_input_string[cursor] != '\0'); cursor++)
        {
                LCD_write_byte(lcd, LCD_input_string[cursor]);
        }
}

void LCD_screen_init(LCD_t lcd)
{
        //Long delay after initial power up
        delay_us(50e3);

        //Reset RS and RW
        lcd.PORT->BRR = lcd.pins.RS;
        lcd.PORT->BRR = lcd.pins.RW;

        //Function set
        LCD_write_command_byte(lcd,0x30);

        //Function set
        LCD_write_command_byte(lcd,0x28);
        delay_us(40);

        // This is a fix some reason needs the E
        lcd.PORT->BSRR = lcd.pins.E;
        delay_us(15);
        lcd.PORT->BRR = lcd.pins.E;

        //Function set
        LCD_write_command_byte(lcd,0x28);
        delay_us(40);

        //Display ON/OFF
        LCD_write_command_byte(lcd,0x0f);
        delay_us(400);

        //Display Clear
        LCD_write_command_byte(lcd,0x01);
        delay_us(2e3);

        //Entry Mode
        LCD_write_command_byte(lcd,0x06);
        //delay_us(45);
}

////////////////////////////////////////////////////////////////////////////////
/// Given Code Below
////////////////////////////////////////////////////////////////////////////////

/* Configure SysTick Timer for use with delay_us function. This will break
 * break compatibility with HAL_delay() by disabling interrupts to allow for
 * shorter delay timing.
 */
void SysTick_Init(void){
        SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |	       // enable SysTick Timer
                SysTick_CTRL_CLKSOURCE_Msk);     // select CPU clock
        SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);      // disable interrupt,
        // breaks HAL delay function
}

/* Delay function using the SysTick timer to count CPU clock cycles for more
 * precise delay timing. Passing a time of 0 will cause an error and result
 * in the maximum delay. Short delays are limited by the clock speed and will
 * often result in longer delay times than specified. @ 4MHz, a delay of 1us
 * will result in a delay of 10-15 us.
 */
void delay_us(const uint32_t time_us) {
        // set the counts for the specified delay
        SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
        SysTick->VAL = 0;                                      // clear the timer count
        SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);        // clear the count flag
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for the flag
}
