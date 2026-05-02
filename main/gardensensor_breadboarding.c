/*
 * Modular Garden Hub - Google Sheets Logger 
 * Target: ESP32-C3 | Framework: ESP-IDF v6.0
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "esp_sntp.h" 

// --- YOUR SECURE CREDENTIALS ---
#include "esp_wifi_credentials.h"

static const char *TAG = "garden_hub";

/* --- HUB IDENTIFICATION --- */
#define NODE_ID             "Hub_Alpha" 

/* --- PIN DEFINITIONS --- */
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 

#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;
adc_oneshot_unit_handle_t adc1_handle;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* =========================================================================
 * WIFI INITIALIZATION
 * ========================================================================= */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected. Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); 
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {};
    // Pulling credentials from your esp_wifi_credentials.h file
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* =========================================================================
 * HARDWARE INITIALIZATION
 * ========================================================================= */
static void init_hardware(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t sht30_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SHT30_SENSOR_ADDR, .scl_speed_hz = I2C_MASTER_FREQ_HZ };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &sht30_cfg, &sht30_handle));

    i2c_device_config_t ltr390_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = LTR390_SENSOR_ADDR, .scl_speed_hz = I2C_MASTER_FREQ_HZ };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ltr390_cfg, &ltr390_handle));

    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config));
}

/* =========================================================================
 * HTTPS POST FUNCTION
 * ========================================================================= */
static void post_to_google_sheets(float temp, float hum, uint32_t light, uint32_t uv, int s1, int s2, int s3) {
    char post_data[512];
    snprintf(post_data, sizeof(post_data), 
             "{"
             "\"node_id\":\"%s\","
             "\"Air Temp (C)\":%.2f,"
             "\"Humidity (%%)\":%.2f,"
             "\"Light Level\":%lu,"
             "\"UV Index\":%lu,"
             "\"Soil Temp (C)\":0,"
             "\"Soil Moisture 1\":%d,"
             "\"Soil Moisture 2\":%d,"
             "\"Soil Moisture 3\":%d,"
             "\"Wind Avg\":0,"
             "\"Wind Peak\":0"
             "}", 
             NODE_ID, temp, hum, light, uv, s1, s2, s3);

    esp_http_client_config_t config = {
        .url = GOOGLE_SCRIPT_URL, // From your header file
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    
    // Accept ESP_OK or the specific Google Chunking quirk as a success
    if (err == ESP_OK || err == ESP_ERR_HTTP_INCOMPLETE_DATA) {
        ESP_LOGI(TAG, "Cloud Sync Successful. Google received the payload.");
    } else {
        ESP_LOGE(TAG, "Cloud Sync Failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

/* =========================================================================
 * MAIN APPLICATION LOOP
 * ========================================================================= */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    
    /* --- SYNC SYSTEM TIME OVER INTERNET --- */
    ESP_LOGI(TAG, "Initializing SNTP for Time Sync...");
    
    // Updated to modern ESP-IDF v6.0 SNTP functions
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;
    
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    
    if (retry == retry_count) {
        ESP_LOGE(TAG, "Failed to get network time. HTTPS will likely fail.");
    } else {
        ESP_LOGI(TAG, "Time Synchronized! Current Year: %d", timeinfo.tm_year + 1900);
    }
    /* -------------------------------------- */

    init_hardware();
    ESP_LOGI(TAG, "Hardware armed. Commencing Cloud Logging.");

    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];
    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000);

    while (1) {
        float temp_c = 0.0, humidity = 0.0;
        uint32_t als_raw = 0;
        uint32_t uv_raw = 0; 
        long m1_sum = 0, m2_sum = 0, m3_sum = 0;

        for(int i = 0; i < 50; i++) {
            int r1 = 0, r2 = 0, r3 = 0;
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_1_CHAN, &r1);
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_2_CHAN, &r2);
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_3_CHAN, &r3);
            m1_sum += r1; m2_sum += r2; m3_sum += r3;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
        vTaskDelay(pdMS_TO_TICKS(50)); 
        if (i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000) == ESP_OK) {
            uint16_t temp_raw = (sht_data[0] << 8) | sht_data[1];
            uint16_t hum_raw = (sht_data[3] << 8) | sht_data[4];
            temp_c = -45.0 + (175.0 * temp_raw / 65535.0);
            humidity = 100.0 * hum_raw / 65535.0;
        }

        uint8_t ltr_reg = 0x0D;
        uint8_t ltr_data[3];
        if (i2c_master_transmit_receive(ltr390_handle, &ltr_reg, 1, ltr_data, sizeof(ltr_data), 1000) == ESP_OK) {
            als_raw = ltr_data[0] | (ltr_data[1] << 8) | (ltr_data[2] << 16);
        }

        post_to_google_sheets(temp_c, humidity, als_raw, uv_raw, (m1_sum/50), (m2_sum/50), (m3_sum/50));

        // Sleep for 60 minutes
        vTaskDelay(pdMS_TO_TICKS(3600000)); 
    }
}