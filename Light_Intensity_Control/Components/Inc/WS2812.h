/**
  ******************************************************************************
  * @file    WS2812.h
  * @author  George
  * @version 1.0
  * @date    Jan 29, 2024
  * @brief   Header file for WS2812 LED driver. [heavily inspired]
  * @source  https://controllerstech.com/interface-ws2812-with-stm32/
  ******************************************************************************
  */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_


#include "tim.h"

/**
 * @defgroup WS2812_Functions WS2812 Functions
 * @brief Functions for controlling WS2812 LEDs.
 * @note Also works with WS2812B. Main difference to note is the duty cycle for HIGH and LOW PWM.
 * @{
 */

/**
 * @brief Maximum number of LEDs.
 */
#define MAX_LED 2
/**
 * @brief Flag for using brightness adjustment.
 */
#define USE_BRIGHTNESS 1

/**
 * @brief Array storing LED data.
 */
extern uint8_t LED_Data[MAX_LED][4];
/**
 * @brief Flag indicating whether data has been sent (for debugging).
 */
extern uint8_t LED_Mod[MAX_LED][4];  // for brightness
extern int datasentflag; // for debugging

/**
 * @brief Set the color of an LED.
 * @param LEDnum The LED number (index).
 * @param Red The intensity of the red color (0-255).
 * @param Green The intensity of the green color (0-255).
 * @param Blue The intensity of the blue color (0-255).
 */
void Set_LED (int LEDnum, int Red, int Green, int Blue);

/**
 * @brief Set the brightness of the LEDs.
 * @param brightness The brightness level (0-255).
 */
void Set_Brightness (int brightness);

/**
 * @brief Send data to the WS2812 LEDs.
 */
void WS2812_Send (void);

/**
 * @brief Reset all LEDs to off state.
 */
void Reset_LEDs (void);

#endif /* INC_WS2812_H_ */
