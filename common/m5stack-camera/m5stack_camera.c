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

// 移除旧的ADC头文件
// #include "driver/adc.h"
// #include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
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
// static esp_adc_cal_characteristics_t *adc_chars;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration = false;
#define DEFAULT_VREF    3600        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#endif

esp_err_t m5_camera_init(void){
    esp_err_t ret = ESP_FAIL;
    ret = m5_camera_led_init();
#ifdef CONFIG_TIMER_CAMERA_X_F
    ret = m5_camera_battery_init();
    ret = m5_camera_button_init();
    ret = bm8563_init_desc(&bm8563_dev, 0, CONFIG_I2C_MANAGER_0_SDA, CONFIG_I2C_MANAGER_0_SCL);
#endif
    return ret;
}

// Initialize the LED for M5Camera
esp_err_t m5_camera_led_init(void){
    // 配置LEDC定时器和通道
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "ledc_timer_config failed");
        return ret;
    }

    ledc_channel_config_t ledc_channel = {
        .channel    = BLINK_LED_LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = BLINK_LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ret = ledc_fade_func_install(0);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "ledc_fade_func_install failed");
        return ret;
    }

    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "ledc_channel_config failed");
        return ret;
    }
    return ESP_OK;
}

//  Set LED brightness
esp_err_t m5_camera_led_set_brightness(uint8_t brightness){
    esp_err_t ret = ESP_FAIL;
    uint32_t duty = ((uint32_t)brightness * 8191u + 127u) / 255u;
    ret = ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLINK_LED_LEDC_CHANNEL, duty, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "ledc_set_duty_and_update failed, duty: %lu", duty);
        return ret;
    }
    return ESP_OK;
}

// Update LED duty cycle
esp_err_t m5_camera_led_set_update_duty(uint32_t duty)
{
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, BLINK_LED_LEDC_CHANNEL, duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, BLINK_LED_LEDC_CHANNEL);
    }
    return ret;
}

// Start LED with fade time
esp_err_t m5_camera_led_start_with_fade_time(uint32_t duty, int time)
{
    return ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLINK_LED_LEDC_CHANNEL, duty, time);
}

#ifdef CONFIG_TIMER_CAMERA_X_F

// adc 标定校准初始化
static esp_err_t adc_calibration_init(void)
{
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif    

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    do_calibration = calibrated;
    return ret;
}

// 电池初始化
esp_err_t m5_camera_battery_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // 配置电池保持引脚
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BAT_HOLD_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    // 初始化ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // 配置ADC1通道
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ret = adc_oneshot_config_channel(adc1_handle, BAT_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        return ret;
    }

    // 初始化ADC校准
    ret = adc_calibration_init();
    
    return ret;
}

// 获取电池电压
int m5_camera_battery_voltage(void)
{
    int adc_raw = 0;
    int voltage = 0;
    uint32_t adc_reading = 0;
    
    // 多次采样取平均
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_ADC_CHANNEL, &adc_raw));
        adc_reading += adc_raw;
    }
    adc_reading /= NO_OF_SAMPLES;
    
    // 转换为电压值
    if (do_calibration) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage));
    } else {
        // 如果没有校准，使用默认计算
        voltage = adc_reading * DEFAULT_VREF / 4096;
    }
    
    // 电池电压需要乘以分压比（通常是2）
    return voltage * 2;
}

esp_err_t m5_camera_battery_set_level(bool level)
{
    return gpio_set_level(BAT_HOLD_PIN, level ? 1 : 0);
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
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PWR_BTN_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;  // 使能上拉
    return gpio_config(&io_conf);
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