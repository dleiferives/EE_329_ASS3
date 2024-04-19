/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#define STM32L476xx

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32l4xx_hal.h"
#include "LCD.h"
void Error_Handler(void);
void SystemClock_Config(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
