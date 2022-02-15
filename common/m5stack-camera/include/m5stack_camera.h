/*
 * MIT License
 *
 * Copyright (c) 2021 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __TIMER_CAMERA_H__
#define __TIMER_CAMERA_H__

#include <esp_err.h>
#include "bm8563.h"

// M5STACK camera series PIN Map
// M5Camera        shell and battery
// TimerCAM        no shell no battery
// TimerCAM    X   DFOV 66.5 with shell and battery
// TimerCAM    F   DFOV 120 with shell and battery
// Camera DIY KIT  DIY shell with 2 lens no battery
// UNIT CAM        no shell no battery

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET 15 //software reset will be performed
#define CAM_PIN_XCLK 27
#ifdef CONFIG_TIMER_CAMERA_X_F
#define CAM_PIN_SIOD 25
#else
#define CAM_PIN_SIOD 22
#endif
#define CAM_PIN_SIOC 23

#define CAM_PIN_D7 19
#define CAM_PIN_D6 36
#define CAM_PIN_D5 18
#define CAM_PIN_D4 39
#define CAM_PIN_D3 5
#define CAM_PIN_D2 34
#define CAM_PIN_D1 35
#define CAM_PIN_D0 32


#if defined(CONFIG_TIMER_CAMERA_X_F)
#define CAM_PIN_VSYNC 22
#else
#define CAM_PIN_VSYNC 25
#endif
#define CAM_PIN_HREF 26
#define CAM_PIN_PCLK 21

#if defined(CONFIG_TIMER_CAMERA_X_F)
#define BLINK_LED_PIN 2
#define BLINK_LED_LEDC_CHANNEL LEDC_CHANNEL_0

#define BAT_HOLD_PIN 33
#define BAT_ADC_PIN  38
#define BAT_ADC_CHANNEL ADC1_CHANNEL_2

#define PWR_BTN_PIN  37

#elif defined(CONFIG_UNIT_CAM)
#define BLINK_LED_PIN 2
#define BLINK_LED_LEDC_CHANNEL LEDC_CHANNEL_0

#define BAT_HOLD_PIN 33
#define BAT_ADC_PIN  38
#define BAT_ADC_CHANNEL ADC1_CHANNEL_2

#define PWR_BTN_PIN  37

#else
#define BLINK_LED_PIN 14
#define BLINK_LED_LEDC_CHANNEL LEDC_CHANNEL_0
#endif


extern const char *M5STACK_LOGO;
extern i2c_dev_t bm8563_dev;  // thread safe

esp_err_t m5_camera_init(void);

esp_err_t m5_camera_led_init(void);
esp_err_t m5_camera_led_set_brightness(uint8_t bn);
esp_err_t m5_camera_led_set_update_duty(uint32_t duty);
esp_err_t m5_camera_led_start_with_fade_time(uint32_t duty, int time);

esp_err_t m5_camera_battery_init(void);
int m5_camera_battery_voltage(void);
esp_err_t m5_camera_battery_set_level(bool level);
esp_err_t m5_camera_battery_hold_power(void);
esp_err_t m5_camera_battery_release_power(void);

esp_err_t m5_camera_button_init(void);
int m5_camera_button_get_level(void);

bool m5_camera_check_rtc_alarm_flag(void);
esp_err_t m5_camera_clear_rtc_alarm_flag(void);
esp_err_t m5_camera_set_timer(int seconds);
bool m5_camera_check_rtc_timer_flag(void);
esp_err_t m5_camera_clear_rtc_timer_flag(void);


#endif