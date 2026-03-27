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
#include <stdio.h>
#include <esp_log.h>
#include "driver/gpio.h"

#include "m5stack_camera.h"

static const char *TAG = "m5stack:wake-up";

void app_main()
{
    printf("%s", M5STACK_LOGO);
    // initialize LED, Button, BAT, BM8563
    m5_camera_init();
	#ifdef CONFIG_TIMER_CAMERA_X_F
    m5_camera_battery_hold_power();
    ESP_LOGI(TAG, "Battery voltage: %dmV", m5_camera_battery_voltage());
    // Turn on LED
    for(int i = 0; i < 5; i++) {
        m5_camera_led_set_brightness(10);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        m5_camera_led_set_brightness(0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    m5_camera_led_set_brightness(10);
    ESP_LOGI(TAG, "Power off and wake up in 5 seconds");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    m5_camera_timer_sleep(10);
#endif

}