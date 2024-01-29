/**
  ******************************************************************************
  * @file    WS2812.c
  * @author  George
  * @version 1.0
  * @date    Jan 29, 2024
  * @brief   Source file for WS2812 LED driver. [heavily inspired]
  * @source  https://controllerstech.com/interface-ws2812-with-stm32/
  ******************************************************************************
  */

#include "WS2812.h"

/**
 * @brief Array to hold PWM data for LED control.
 */
uint16_t pwmData[(24*MAX_LED)+50];

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness
int datasentflag = 0; // for debugging

/**
 * @brief Set the color of an LED.
 * @param LEDnum The LED number (index).
 * @param Red The intensity of the red color (0-255).
 * @param Green The intensity of the green color (0-255).
 * @param Blue The intensity of the blue color (0-255).
 */
void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
  LED_Data[LEDnum][0] = LEDnum;
  LED_Data[LEDnum][1] = Green;
  LED_Data[LEDnum][2] = Red;
  LED_Data[LEDnum][3] = Blue;
}

/**
 * @brief Set the brightness of the LEDs.
 * @param brightness The brightness level (0-255).
 */
void Set_Brightness (int brightness)  // 0-255
{
#if USE_BRIGHTNESS

  if (brightness > 255) brightness = 255;
  for (int i = 0; i < MAX_LED; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      LED_Mod[i][j] = (LED_Data[i][j] * brightness) / 255;
    }
  }

#endif

}

/**
 * @brief Send data to the WS2812 LEDs.
 */
void WS2812_Send (void)
{
  uint32_t indx=0;
  uint32_t color;


  for (int i= 0; i<MAX_LED; i++)
  {
#if USE_BRIGHTNESS
    color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
    color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

    for (int i=23; i>=0; i--)
    {
      if (color&(1<<i))
      {
        pwmData[indx] = 80;  // 60 is 2/3 of 90 for 72Mhz or 80 is 2/3 of 134 for 108Mhz
      }

      else pwmData[indx] = 40;  // 30 is 1/3 of 90 for 72Mhz or 40 is 1/3 of 134 for 108Mhz

      indx++;
    }

  }

  for (int i=0; i<50; i++)
  {
    pwmData[indx] = 0;
    indx++;
  }

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
  while (!datasentflag){};
  datasentflag = 0;
}

/**
 * @brief Reset all LEDs to off state.
 */
void Reset_LEDs (void)
{
  for (int i=0; i<MAX_LED; i++)
  {
    LED_Data[i][0] = i;
    LED_Data[i][1] = 0;
    LED_Data[i][2] = 0;
    LED_Data[i][3] = 0;
  }
}

