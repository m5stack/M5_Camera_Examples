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

#include "m5stack_camera.h"

static const char *TAG = "m5stack:wake-up";

void app_main()
{
    printf("%s", M5STACK_LOGO);
    // initialize LED, Button, BAT, BM8563
    m5_camera_init();
#ifdef CONFIG_TIMER_CAMERA_X_F
    // Power on
    // m5_camera_battery_hold_power();
    // Check battery voltage
    m5_camera_battery_hold_power();
    // Check battery voltage
    ESP_LOGI(TAG, "Battery voltage: %dmV", m5_camera_battery_voltage());
    // Turn on LED
    m5_camera_led_set_brightness(10);

    if (m5_camera_check_rtc_alarm_flag()) {
        ESP_LOGI(TAG, "Clear alarm flag");
        m5_camera_clear_rtc_alarm_flag();
    }

    if (m5_camera_check_rtc_timer_flag()) {
        ESP_LOGI(TAG, "Clear timer flag");
        m5_camera_clear_rtc_timer_flag();
    }

    vTaskDelay(5000 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "Power off and wake up in 30 seconds");
    m5_camera_set_timer(30);
    vTaskDelay(500 / portTICK_RATE_MS);
    // Power off.
    m5_camera_battery_release_power();
#endif

}