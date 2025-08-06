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

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "esp_camera.h"
#include "esp_sntp.h"
#include "protocol_examples_common.h"
#include "esp_http_client.h"
#include "mbedtls/sha256.h"

#include "bm8563.h"

#include "m5stack_camera.h"

static const char *TAG = "m5stack:wake-up";

#define HASH_HEX_FORMART "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"

#define CONFIG_ESP_WIFI_SSID "WIFI_SSID"
#define CONFIG_ESP_WIFI_PASSWORD "WIFI_PASSWORD"

// without the "http"
char* CONFIG_AMAZON_S3_HOST = "example.s3.amazonaws.com";
char* CONFIG_ACCESS_KEY = "xxxxxxxxx";
char* CONFIG_SECRET_ACCESS_KEY = "AWS4xxxxxxxxxxxxxxxx";;

char* PIC_PATH = "/path";
char* PIC_NAME = "name";
int INTERVAL = 30;
int SIZE = FRAMESIZE_UXGA;

static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
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
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
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
		ESP_LOGI(TAG, "connected to ap SSID:%s",
				 CONFIG_ESP_WIFI_SSID);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
				 CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);
}

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
    .frame_size = FRAMESIZE_UXGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 13, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
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

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "ntp.aliyun.com");
    sntp_setservername(1, "ntp1.aliyun.com");
    // sntp_setservername(0, "ntp1.aliyunyunyun.com");
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

void ByteToHexStr(char *str, int len, char *dest)
{
    char tmp;
    char stb[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
    for (size_t i = 0; i < len; i++) {
        tmp = str[i];
        dest[i * 2] = stb[tmp >> 4];
        dest[i * 2 + 1] = stb[tmp & 15];
    }
    return;
}

extern int hmac_sha256(const uint8_t *, size_t, const uint8_t *, size_t data_len, uint8_t *);

char *CanonicalRequest_Base = "PUT\n%s\n\nhost:%s\nx-amz-content-sha256:%s\nx-amz-date:%s\n\nhost;x-amz-content-sha256;x-amz-date\n%s";
char *StringToSign_Base = "AWS4-HMAC-SHA256\n%s\n%s/us-east-1/s3/aws4_request\n%s";
char *Authorization_Header_Base = "AWS4-HMAC-SHA256 Credential=%s/%s/us-east-1/s3/aws4_request,SignedHeaders=host;x-amz-content-sha256;x-amz-date,Signature=%s";


void tcp_client_task(void *pvParameters)
{
    camera_fb_t *fb = NULL;

    char path[50];
    int count = 0;

    for(uint8_t count=0; count<10; count++) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }
        esp_camera_fb_return(fb);
    }

    for (;;) {

        time_t now;
        char ISO_DATE[9];
        char ISO_DATE_TIME[17];
        char GMT_DATA_TIME[30];
        struct tm *timeinfo;
        now = time(NULL);

        timeinfo = localtime(&now);
        strftime(ISO_DATE, sizeof(ISO_DATE), "%Y%m%d", timeinfo);
        strftime(ISO_DATE_TIME, sizeof(ISO_DATE_TIME), "%Y%m%dT%H%M%SZ", timeinfo);
        strftime(GMT_DATA_TIME, sizeof(GMT_DATA_TIME), "%a, %d %b %Y %H:%M:%S GMT", timeinfo);
        printf("%s\n", ISO_DATE);
        printf("%s\n", ISO_DATE_TIME);
        printf("%s\n", GMT_DATA_TIME);

        sprintf(path, "%s/%s-%s.jpg", PIC_PATH, PIC_NAME, ISO_DATE_TIME);

        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }
        // SHA256
        unsigned char file_sha256[32];
        char file_sha256_hex[65];
        mbedtls_sha256_context sha256_flash;
        mbedtls_sha256_init(&sha256_flash);
        mbedtls_sha256_starts(&sha256_flash, 0); // 0表示传sha256 ， 1 表示传SHA-244
        mbedtls_sha256_update(&sha256_flash, fb->buf, fb->len);
        mbedtls_sha256_finish(&sha256_flash, file_sha256);
        mbedtls_sha256_free(&sha256_flash);

        ByteToHexStr((char *)file_sha256, 32, file_sha256_hex);
        file_sha256_hex[64] = '\0';

        char CanonicalRequest[512] = {};
        sprintf(CanonicalRequest, CanonicalRequest_Base, path, CONFIG_AMAZON_S3_HOST, file_sha256_hex, ISO_DATE_TIME, file_sha256_hex);

        char CanonicalRequest_hash[33];
        char CanonicalRequest_hash_hex[65];
        mbedtls_sha256_context sha256_CanonicalRequest;
        mbedtls_sha256_init(&sha256_CanonicalRequest);
        mbedtls_sha256_starts(&sha256_CanonicalRequest, 0); // 0表示传sha256 ， 1 表示传SHA-244
        mbedtls_sha256_update(&sha256_CanonicalRequest, (unsigned char *)CanonicalRequest, strlen(CanonicalRequest));
        mbedtls_sha256_finish(&sha256_CanonicalRequest, (unsigned char *)CanonicalRequest_hash);
        mbedtls_sha256_free(&sha256_CanonicalRequest);

        ByteToHexStr((char *)CanonicalRequest_hash, 32, CanonicalRequest_hash_hex);
        CanonicalRequest_hash_hex[64] = '\0';
        // printf("CanonicalRequest_hash_hex: %s\r\n\r\n", CanonicalRequest_hash_hex);
        // free(CanonicalRequest_hash);

        uint8_t StringToSign[512];
        int len1 = sprintf((char *)StringToSign, StringToSign_Base, ISO_DATE_TIME, ISO_DATE, CanonicalRequest_hash_hex);
        StringToSign[len1] = '\0';

        uint8_t DateKey[32];
        uint8_t DateRegionKey[32];
        uint8_t DateRegionServiceKey[32];
        uint8_t SigningKey[32];
        uint8_t Signature[32];
        hmac_sha256((const uint8_t *)CONFIG_SECRET_ACCESS_KEY, strlen(CONFIG_SECRET_ACCESS_KEY), (const uint8_t *)ISO_DATE, strlen(ISO_DATE), (uint8_t *)DateKey);
        hmac_sha256((const uint8_t *)DateKey, 32, (const uint8_t *)"us-east-1", strlen("us-east-1"), (uint8_t *)DateRegionKey);
        hmac_sha256((const uint8_t *)DateRegionKey, 32, (const uint8_t *)"s3", 2, (uint8_t *)DateRegionServiceKey);
        hmac_sha256((const uint8_t *)DateRegionServiceKey, 32, (const uint8_t *)"aws4_request", 12, (uint8_t *)SigningKey);
        hmac_sha256((const uint8_t *)SigningKey, 32, (const uint8_t *)StringToSign, strlen((char *)StringToSign), (uint8_t *)Signature);

        char Signature_hex[65];
        ByteToHexStr((char *)Signature, 32, Signature_hex);
        Signature_hex[64] = '\0';

        char authorization[512];
        len1 = sprintf(authorization, Authorization_Header_Base, CONFIG_ACCESS_KEY, ISO_DATE, Signature_hex);
        authorization[len1] = '\0';

        char len[5];
        len1 = sprintf(len, "%d", fb->len);
        len[len1] = '\0';

        char url[100];
        len1 = sprintf(url, "http://%s%s",CONFIG_AMAZON_S3_HOST, path);
        url[len1] = '\0';

        printf("URL: %s\r\n\r\n", url);

        esp_http_client_config_t config = {
            .url = url,
            .buffer_size_tx = 1024
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_http_client_set_method(client, HTTP_METHOD_PUT);
        esp_http_client_set_header(client, "content-length", len);
        esp_http_client_set_header(client, "x-amz-content-sha256", file_sha256_hex);
        esp_http_client_set_header(client, "x-amz-date", ISO_DATE_TIME);
        esp_http_client_set_header(client, "authorization", authorization);
        esp_http_client_set_header(client, "host", CONFIG_AMAZON_S3_HOST);

        esp_http_client_set_header(client, "Content-Type", "image/jpeg");

        esp_http_client_set_post_field(client, (const char *)fb->buf, fb->len);

        printf("content-length: %s\r\n", len);
        printf("x-amz-content-sha256: %s\r\n", file_sha256_hex);
        printf("x-amz-date: %s\r\n", ISO_DATE_TIME);
        printf("authorization: %s\r\n", authorization);
        printf("host: %s\r\n", CONFIG_AMAZON_S3_HOST);

        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            printf("OK\r\n");
        } else {
            printf("ERROR\r\n");
        }

        int status_code = esp_http_client_get_status_code(client);

        printf("status_code: %d\r\n", status_code);

        if (status_code == 403) {
            uint8_t *buffer = (uint8_t *)malloc(2048 * sizeof(uint8_t));
            int content_length =  esp_http_client_read(client, (char *)buffer, 2048);
            printf("content_length: %d\r\n", content_length);
            if (content_length > 0) {
                printf("%s\r\n", buffer);
            }
            free(buffer);
        }
        esp_camera_fb_return(fb);

        if (m5_camera_check_rtc_alarm_flag()) {
            ESP_LOGI(TAG, "Clear alarm flag");
            m5_camera_clear_rtc_alarm_flag();
        }

        if (m5_camera_check_rtc_timer_flag()) {
            ESP_LOGI(TAG, "Clear timer flag");
            m5_camera_clear_rtc_timer_flag();
        }


        ESP_LOGI(TAG, "Power off and wake up in %d seconds", INTERVAL);
        m5_camera_set_timer(INTERVAL);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // Power off.
        m5_camera_battery_release_power();
        vTaskDelay(INTERVAL*1000 / portTICK_PERIOD_MS);

    }

    while (1) {
        /* code */
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{


    printf("%s", M5STACK_LOGO);
    // initialize LED, Button, BAT, BM8563
    m5_camera_init();
    // Power on
    // m5_camera_battery_hold_power();
    // Check battery voltage
    m5_camera_battery_hold_power();
    // Check battery voltage
    ESP_LOGI(TAG, "Battery voltage: %dmV", m5_camera_battery_voltage());
    // Turn on LED
    m5_camera_led_set_brightness(10);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    obtain_time();
    camera_config.frame_size = (framesize_t)SIZE;
    init_camera();

    // flip the camera fream
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);

    xTaskCreate(tcp_client_task, "tcp_client_task", 1024 * 50, NULL, 4, NULL);
}


