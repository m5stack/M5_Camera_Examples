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
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_sntp.h"

#include "esp_camera.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
#define MD5Init esp_rom_md5_init
#define MD5Update esp_rom_md5_update
#define MD5Final esp_rom_md5_final
#include "esp_rom_md5.h"
#else
#include "esp32/rom/md5_hash.h"
#endif // ESP_IDF_VERSION

#include "esp_http_client.h"

#include "bm8563.h"
#include "m5stack_camera.h"

#define WIFI_SSID "XXXXXXXXXXXX"        // <<<<< change here
#define WIFI_PSWD "XXXXXXXXXXXX"        // <<<<< change here
#define WIFI_RETRY_NUM 10

#define NTP_SERVER_0 "ntp.aliyun.com"
#define NTP_SERVER_1 "ntp1.aliyun.com"

// It is recommended to use a RAM account
// please ensure that you have OSS write permissions
#define ACCESS_KEY_ID      "XXXXXXXXXXXXXXXXX" // "xxxxxxxxxxxxxxxxxxxx"
#define ACCESS_KEY_SECRET  "XXXXXXXXXXXXXXXXX" // "xxxxxxxxxxxxxxxxxxxx"
#define OSS_ENDPOINT_NAME  "XXXXXXXXXXXXXXXXX" // "xxxxxxx.aliyuncs.com"
#define OSS_BUCKET_NAME    "XXXXXXXXXXXXXXXXX" // "xxxxxxx"
#define OSS_PATH_NAME      "XXXXXXXXXXXXXXXXX" // "xxxxx/xxxxxx"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static const char *TAG = "m5stack:ali-oss";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 2,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_RETRY_NUM) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &event_handler,
                    NULL,
                    &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &event_handler,
                    NULL,
                    &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PSWD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, NTP_SERVER_0);
    sntp_setservername(1, NTP_SERVER_1);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}

extern int hmac_sha1(const uint8_t *, size_t, const uint8_t *, size_t data_len, uint8_t *);
extern unsigned char *base64_encode(const unsigned char *src, size_t len, size_t *out_len);

static char *calculate_file_md5(const uint8_t *data, uint32_t data_len)
{
    uint8_t digest[16];
    struct MD5Context ctx;

    MD5Init(&ctx);
    MD5Update(&ctx, (unsigned char *)data, data_len);
    MD5Final(digest, &ctx);
    return (char *)base64_encode((unsigned char *)digest, 16, NULL);
}

static char *calculate_signature(const char *md5, const char *file_name, const char *gmt_time)
{
    uint8_t hash[20];
    char *string_to_signature;

    asprintf(&string_to_signature, "%s\n%s%s\n%s\n/%s/%s", "PUT", md5, "image/jpeg", gmt_time, OSS_BUCKET_NAME, file_name);
    hmac_sha1((const uint8_t *)ACCESS_KEY_SECRET, strlen(ACCESS_KEY_SECRET), (uint8_t *)string_to_signature, strlen(string_to_signature), hash);
    free(string_to_signature);
    return (char *)base64_encode((unsigned char *)hash, 20, NULL);
}

void oss_put_task(void *pvParameters)
{
    camera_fb_t *fb = NULL;
    char *md5 = NULL;
    char *signature = NULL;
    char url[200];
    char file_len[10];
    char file_name[100];
    char authorization[200];
    time_t now;
    struct tm *timeinfo;
    char GMT_DATA_TIME[30];
    int s_len = 0;

    for (;;) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        now = time(NULL);
        timeinfo = localtime(&now);
        memset(GMT_DATA_TIME, 0, sizeof(GMT_DATA_TIME));
        strftime(GMT_DATA_TIME, sizeof(GMT_DATA_TIME), "%a, %d %b %Y %H:%M:%S GMT", timeinfo);

        memset(file_name, 0, sizeof(file_name));
        s_len = sprintf(file_name, "%s/M5TimerCamera-%d-%d-%d-%d-%d-%d.jpg", OSS_PATH_NAME, timeinfo->tm_year, timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
        file_name[s_len] = '\0';

        md5 = calculate_file_md5(fb->buf, fb->len);

        signature = calculate_signature(md5, file_name, GMT_DATA_TIME);

        memset(authorization, 0, sizeof(authorization));
        s_len = sprintf(authorization, "OSS %s:%s", ACCESS_KEY_ID, signature);
        authorization[s_len - 1] = '\0';

        memset(file_len, 0, sizeof(file_len));
        s_len = sprintf(file_len, "%d", fb->len);
        file_len[s_len] = '\0';

        memset(url, 0, sizeof(url));
        s_len = sprintf(url, "http://%s.%s/%s", OSS_BUCKET_NAME, OSS_ENDPOINT_NAME, file_name);
        url[s_len] = '\0';
        ESP_LOGI(TAG, "URL: %s", url);

        esp_http_client_config_t config = {
            .url = url,
            .buffer_size_tx = 1024
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_http_client_set_method(client, HTTP_METHOD_PUT);
        esp_http_client_set_header(client, "content-type", "image/jpeg");
        esp_http_client_set_header(client, "content-md5", md5);
        esp_http_client_set_header(client, "content-length", file_len);
        esp_http_client_set_header(client, "date", GMT_DATA_TIME);
        esp_http_client_set_header(client, "authorization", authorization);
        esp_http_client_set_post_field(client, (const char *)fb->buf, fb->len);

        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP PUT OK");
        } else {
            ESP_LOGI(TAG, "HTTP PUT ERROR");
        }

        int status_code = esp_http_client_get_status_code(client);

        if (status_code == 403) {
            ESP_LOGE(TAG, "PUT image to Ali-OSS error,please check ACCESS_KEY_ID or ACCESS_KEY_SECRET!");
        } else if (status_code == 200) {
            ESP_LOGI(TAG, "PUT image to Ali-OSS OK.");
        }
        esp_camera_fb_return(fb);

        vTaskDelay(5000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    printf("%s", M5STACK_LOGO);
    // initialize LED, Button, BAT, BM8563
    m5_camera_init();
#ifdef CONFIG_TIMER_CAMERA_X_F
    // Power on
    // m5_camera_battery_hold_power();
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
#endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // initialize the camera
    if (ESP_OK != init_camera()) {
        return;
    }

    // flip the camera fream
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);

    wifi_init_sta();

    obtain_time();

    xTaskCreate(oss_put_task, "oss_put_task", 1024 * 50, NULL, 4, NULL);
}