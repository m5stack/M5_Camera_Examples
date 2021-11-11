/* SMB Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   Therefore, it is necessary to add the following to the end of components/libsmb2/include/esp/config.h.
   #define MD5Init esp_rom_md5_init
   #define MD5Update esp_rom_md5_update
   #define MD5Final esp_rom_md5_final

*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "string.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "smb2.h"
#include "libsmb2.h"
#include "libsmb2-raw.h"
#include <sys/param.h>
#include "esp_camera.h"
#include <time.h>
#include <sys/time.h>
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "bm8563.h"
#include "m5stack_camera.h"

RTC_DATA_ATTR static int boot_count = 0;

static const char *TAG = "TimerCamera";

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */

static void obtain_time(void);
static void initialize_sntp(void);

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

#define CONFIG_ESP_WIFI_SSID "WIFI_SSID"
#define CONFIG_ESP_WIFI_PASSWORD "WIFI_PASSWORD"

//HOST IP
char* CONFIG_SMB_HOST = "xxx.xxx.x.xxx";

//If not, you can leave it blank ""
char* CONFIG_SMB_USER = "";
char* CONFIG_SMB_PASSWORD = "";

char* CONFIG_SMB_PATH = "path";
char* PIC_NAME = "TIMER";

//It is recommended to be greater than 10s
int INTERVAL = 30;
int SIZE = FRAMESIZE_UXGA;

int CONFIG_ESP_MAXIMUM_RETRY=5;

#define MAXBUF 64
uint8_t buf[MAXBUF];
uint32_t pos;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT	   BIT1


static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
								int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
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
    
	esp_wifi_init(&cfg);

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

    .jpeg_quality = 8, //0-63 lower number means higher quality
    .fb_count = 2,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}


void smb_task(void *pvParameters) {


	camera_fb_t *fb = NULL;

	struct smb2_context *smb2;
	struct smb2fh *fh;

    for(uint8_t count=0; count<10; count++) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }
        esp_camera_fb_return(fb);
    }

	for(;;){
		char strftime_buf[100];
		
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

		sprintf(strftime_buf, "%s_%s.jpg", 
			PIC_NAME,
			ISO_DATE_TIME
		);
		
		ESP_LOGI("File Name", "%s", strftime_buf);
	
		smb2 = smb2_init_context();
		if (smb2 == NULL) {
			ESP_LOGE(TAG, "Failed to init context");
			while(1){ vTaskDelay(1); }
		}

		smb2_set_user(smb2, CONFIG_SMB_USER);
		smb2_set_password(smb2, CONFIG_SMB_PASSWORD);
		smb2_set_security_mode(smb2, SMB2_NEGOTIATE_SIGNING_ENABLED);

		if (smb2_connect_share(smb2, CONFIG_SMB_HOST, CONFIG_SMB_PATH, CONFIG_SMB_USER) < 0) {
			ESP_LOGE(TAG, "smb2_connect_share failed. %s", smb2_get_error(smb2));
			smb2_disconnect_share(smb2);
			smb2_destroy_context(smb2);
			continue;
		}

		fb = esp_camera_fb_get();
		if (!fb) {
			ESP_LOGE(TAG, "Camera capture failed");

			smb2_disconnect_share(smb2);
			smb2_destroy_context(smb2);
			continue;
		}

		fh = smb2_open(smb2, strftime_buf , O_WRONLY | O_CREAT);
		if (fh == NULL) {
			ESP_LOGE(TAG, "smb2_open failed. %s", smb2_get_error(smb2));
			smb2_disconnect_share(smb2);
			smb2_destroy_context(smb2);
			continue;
		}

		ESP_LOGI("camera", "height: %d  width: %d  len: %d", fb->height, fb->width, fb->len);

		int loc = 0;
		int len = 4096;
		while (loc < fb->len)
		{
			int res= smb2_write(smb2, fh, fb->buf + loc, len);
			if (res > 0)
			{
				if ((loc + len * 2) > fb->len) {
					loc += len;
					len = (fb->len - loc);  // Remaining buffer len
				}
				else {
					loc += len;
				}
			}
			else
			{
				ESP_LOGE(TAG, "error writing to smb: %s\n", smb2_get_error(smb2));
			}
		}

		smb2_close(smb2, fh);
		smb2_disconnect_share(smb2);
		smb2_destroy_context(smb2);
		ESP_LOGI(TAG, "smb close");
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
        vTaskDelay(500 / portTICK_RATE_MS);
        // Power off.
        m5_camera_battery_release_power();
        vTaskDelay(INTERVAL*1000 / portTICK_RATE_MS);
	}

    while (1) {
        /* code */
        vTaskDelay(500 / portTICK_RATE_MS);
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

	
	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

	wifi_init_sta();
	init_camera();

    // flip the camera fream
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
	
	obtain_time();
    xTaskCreate(smb_task, "smb_task", 1024 * 50, NULL, 4, NULL);

}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "ntp.aliyun.com");
    sntp_setservername(1, "ntp1.aliyun.com");
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
