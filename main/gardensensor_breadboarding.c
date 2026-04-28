/*
 * Integrated Sensor Node - I2C & Analog Architecture
 * Target: ESP32-C3
 * Framework: ESP-IDF v6.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "sensor_node";

/* --- PIN DEFINITIONS --- */
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 // GPIO 0
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 // GPIO 1
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 // GPIO 3

/* --- SENSOR ADDRESSES --- */
#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;
adc_oneshot_unit_handle_t adc1_handle;


static void init_i2c_sensors(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t sht30_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT30_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &sht30_cfg, &sht30_handle));

    i2c_device_config_t ltr390_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LTR390_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ltr390_cfg, &ltr390_handle));
}

static void init_analog_sensors(void) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 12-bit resolution (0-4095)
        .atten = ADC_ATTEN_DB_12,         // 12dB attenuation for ~0-3.3V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config));
}

void app_main(void) {
    init_i2c_sensors();
    init_analog_sensors();
    ESP_LOGI(TAG, "I2C and ADC hardware architecture established.");

    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];

    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    esp_err_t ltr_wake_err = i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000);
    if (ltr_wake_err == ESP_OK) {
        ESP_LOGI(TAG, "LTR390 awakened from standby.");
    } else {
        ESP_LOGE(TAG, "CRITICAL: Failed to wake LTR390.");
    }

    while (1) {
        float temp_c = 0.0, humidity = 0.0;
        uint32_t als_raw = 0;
        int moist1_raw = 0, moist2_raw = 0, moist3_raw = 0;

        /* --- 1. TRIGGER I2C MEASUREMENTS --- */
        i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
        
        // Give the SHT30 time to process the reading
        vTaskDelay(pdMS_TO_TICKS(50)); 

        /* --- 2. RETRIEVE I2C DATA --- */
        // SHT30
        if (i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000) == ESP_OK) {
            uint16_t temp_raw = (sht_data[0] << 8) | sht_data[1];
            uint16_t hum_raw = (sht_data[3] << 8) | sht_data[4];
            temp_c = -45.0 + (175.0 * temp_raw / 65535.0);
            humidity = 100.0 * hum_raw / 65535.0;
        }

        // LTR390
        uint8_t ltr_reg = 0x0D;
        uint8_t ltr_data[3];
        if (i2c_master_transmit_receive(ltr390_handle, &ltr_reg, 1, ltr_data, sizeof(ltr_data), 1000) == ESP_OK) {
            als_raw = ltr_data[0] | (ltr_data[1] << 8) | (ltr_data[2] << 16);
        }

        /* --- 3. RETRIEVE ADC DATA --- */
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_1_CHAN, &moist1_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_2_CHAN, &moist2_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_3_CHAN, &moist3_raw));
        
        float moist_avg = (moist1_raw + moist2_raw + moist3_raw) / 3.0f;

        /* --- 4. SERIAL TELEMETRY --- */
        ESP_LOGI(TAG, "Air Temp: %.2f C | Hum: %.2f %% | Light: %lu", temp_c, humidity, als_raw);
        ESP_LOGI(TAG, "Moisture 1: %d | Moisture 2: %d | Moisture 3: %d | Avg: %.1f", 
                 moist1_raw, moist2_raw, moist3_raw, moist_avg);
        ESP_LOGI(TAG, "--------------------------------------------------");

        // 10-second polling interval
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}