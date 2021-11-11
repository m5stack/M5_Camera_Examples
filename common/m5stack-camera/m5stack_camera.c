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
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

#include "m5stack_camera.h"

const char *M5STACK_LOGO = \
                           "\r\n"\
                           "    __  _______________________   ________ __\r\n"\
                           "   /  |/  / ____/ ___/_  __/   | / ____/ //_/\r\n"\
                           "  / /|_/ /___ \\ \\__ \\ / / / /| |/ /   / ,<   \r\n"\
                           " / /  / /___/ /___/ // / / ___ / /___/ /| |  \r\n"\
                           "/_/  /_/_____//____//_/ /_/  |_\\____/_/ |_|  \r\n"\
                           "                                             \r\n";
const char *TAG = "M5Camera";

#ifdef CONFIG_TIMER_CAMERA_X_F
i2c_dev_t bm8563_dev;  // thread safe
static esp_adc_cal_characteristics_t *adc_chars;
#define DEFAULT_VREF    3600        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#endif

esp_err_t m5_camera_init(void)
{
    esp_err_t ret = ESP_FAIL;
    ret = m5_camera_led_init();
#ifdef CONFIG_TIMER_CAMERA_X_F
    ret = m5_camera_battery_init();
    ret = m5_camera_button_init();
    ret = bm8563_init_desc(&bm8563_dev, 0, CONFIG_I2C_MANAGER_0_SDA, CONFIG_I2C_MANAGER_0_SCL);
#endif
    return ret;
}

esp_err_t m5_camera_led_init(void)
{
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[1] = {
        {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel    = BLINK_LED_LEDC_CHANNEL,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = BLINK_LED_PIN,
            .duty       = 0,
            .hpoint     = 0,
        }
    };

    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel[0]);

    // Initialize fade service.
    return ledc_fade_func_install(0);
}

esp_err_t m5_camera_led_set_brightness(uint8_t bn)
{
    uint32_t duty = (uint32_t)((bn / 100.0) * 8192.0);
    return m5_camera_led_set_update_duty(duty);
}

esp_err_t m5_camera_led_set_update_duty(uint32_t duty)
{
    /*
     * PWM
     * frequency is 5Khz
     * duty resolution is LEDC_TIMER_13_BIT
     * duty range: 0 ~ 8192
     */
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, BLINK_LED_LEDC_CHANNEL, duty);
    return ledc_update_duty(LEDC_HIGH_SPEED_MODE, BLINK_LED_LEDC_CHANNEL);
}

esp_err_t m5_camera_led_start_with_fade_time(uint32_t duty, int time)
{
    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE,
                            BLINK_LED_LEDC_CHANNEL, duty, time);
    return ledc_fade_start(LEDC_HIGH_SPEED_MODE,
                           BLINK_LED_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
}

#ifdef CONFIG_TIMER_CAMERA_X_F
esp_err_t m5_camera_battery_init(void)
{
    /*
     * Battery manager pin:
     * High level is enable battery power(power on)
     * Low level will disable battery power(power off)

     * Boot sequence:
     * 1. RTC or BTN -> wake up (timer or alarm or button)
     * 2. Power on -> hold battery pin(set high level)
     * 3. Clear RTC timer or alarm flag for next time power off
     */
    gpio_pad_select_gpio(BAT_HOLD_PIN);
    gpio_set_direction(BAT_HOLD_PIN, GPIO_MODE_OUTPUT);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BAT_ADC_CHANNEL, ADC_ATTEN_DB_11);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    return ESP_OK;
}

int m5_camera_battery_voltage(void)
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)BAT_ADC_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = (uint32_t)(esp_adc_cal_raw_to_voltage(adc_reading, adc_chars) / 0.661);
    // printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    return voltage;
}

esp_err_t m5_camera_battery_set_level(bool level)
{
    return gpio_set_level(BAT_HOLD_PIN, level);
}

esp_err_t m5_camera_battery_hold_power(void)
{
    return gpio_set_level(BAT_HOLD_PIN, 1);
}

esp_err_t m5_camera_battery_release_power(void)
{
    return gpio_set_level(BAT_HOLD_PIN, 0);
}

esp_err_t m5_camera_button_init(void)
{
    gpio_pad_select_gpio(PWR_BTN_PIN);
    gpio_set_direction(PWR_BTN_PIN, GPIO_MODE_INPUT);
    return gpio_set_pull_mode(PWR_BTN_PIN, GPIO_PULLUP_ONLY);
}

int m5_camera_button_get_level(void)
{
    return gpio_get_level(PWR_BTN_PIN);
}

bool m5_camera_check_rtc_alarm_flag(void)
{
    bool flag = false;
    bm8563_get_alarm_flag(&bm8563_dev, &flag);
    return flag;
}

esp_err_t m5_camera_clear_rtc_alarm_flag(void)
{
    return bm8563_clear_alarm_flag(&bm8563_dev);
}

esp_err_t m5_camera_set_timer(int seconds)
{
    if (seconds > 255) {
        uint8_t minute = (seconds / 60) & 0xFF;
        bm8563_set_timer_settings(&bm8563_dev, true, BM8563_TIMER_1_60HZ);
        bm8563_set_timer_value(&bm8563_dev, minute);
    } else {
        bm8563_set_timer_settings(&bm8563_dev, true, BM8563_TIMER_1HZ);
        bm8563_set_timer_value(&bm8563_dev, (uint8_t)seconds);
    }
    return bm8563_start_timer(&bm8563_dev);
}

bool m5_camera_check_rtc_timer_flag(void)
{
    bool flag = false;
    bm8563_get_timer_flag(&bm8563_dev, &flag);
    return flag;
}

esp_err_t m5_camera_clear_rtc_timer_flag(void)
{
    return bm8563_clear_timer_flag(&bm8563_dev);
}
#endif