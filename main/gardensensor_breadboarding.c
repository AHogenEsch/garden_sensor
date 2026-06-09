/*
 * Autonomous Garden Hub - Sensor Diagnostic Loop
 * Target: ESP32-C3 | Framework: ESP-IDF v6.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "DIAGNOSTIC";

/* --- PIN DEFINITIONS --- */
#define SENSOR_POWER_PIN            GPIO_NUM_5 

#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define ONEWIRE_BUS_GPIO            7          

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 // GPIO 0 
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 // GPIO 1 
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 // GPIO 3 

#define UART_RX_PIN                 10
#define TRIG_PIN                    9

/* --- SENSOR ADDRESSES --- */
#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53 // Default I2C address for LTR390

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;
adc_oneshot_unit_handle_t adc1_handle;
onewire_bus_handle_t ow_bus;
ds18b20_device_handle_t ds18b20 = NULL;

void app_main(void) {
    ESP_LOGI(TAG, "=== STARTING CONTINUOUS SENSOR DIAGNOSTIC ===");

    /* --- 0. SEVER ALL JTAG CONNECTIONS --- */
    // Reclaim the pins from the hardware debugging matrix
    gpio_reset_pin(SENSOR_POWER_PIN);  // Pin 5
    gpio_reset_pin(I2C_MASTER_SDA_IO); // Pin 4
    gpio_reset_pin(I2C_MASTER_SCL_IO); // Pin 6
    gpio_reset_pin(ONEWIRE_BUS_GPIO);  // Pin 7

    /* --- 1. ENERGIZE POWER RAIL SAFELY --- */
    gpio_set_direction(SENSOR_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_POWER_PIN, 0); 
    ESP_LOGI(TAG, "Power Rail ON. Waiting for sensors to boot...");
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    /* --- 2. INITIALIZE UART (ATTINY) --- */
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);

    uart_config_t uart_config = { .baud_rate = 9600, .data_bits = UART_DATA_8_BITS, 
                                  .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, 
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

    /* --- 3. INITIALIZE I2C (SHT30 & LTR390) --- */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = { .enable_internal_pullup = true }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t sht30_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SHT30_SENSOR_ADDR, .scl_speed_hz = I2C_MASTER_FREQ_HZ };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &sht30_cfg, &sht30_handle));

    i2c_device_config_t ltr390_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = LTR390_SENSOR_ADDR, .scl_speed_hz = I2C_MASTER_FREQ_HZ };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ltr390_cfg, &ltr390_handle));

    // Wake up the LTR390 (Write 0x02 to MAIN_CTRL register 0x00)
    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000);

    /* --- 4. INITIALIZE ADC (MOISTURE) --- */
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config));

    /* --- 5. INITIALIZE 1-WIRE (DS18B20) --- */
    onewire_bus_config_t ow_config = { .bus_gpio_num = ONEWIRE_BUS_GPIO };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&ow_config, &rmt_config, &ow_bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_ow_device;
    onewire_new_device_iter(ow_bus, &iter);
    if (onewire_device_iter_get_next(iter, &next_ow_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        ds18b20_new_device_from_enumeration(&next_ow_device, &ds_cfg, &ds18b20);
        ds18b20_set_resolution(ds18b20, DS18B20_RESOLUTION_12B);
        ESP_LOGI(TAG, "DS18B20 found and initialized.");
    } else {
        ESP_LOGE(TAG, "DS18B20 not found on bus!");
    }
    onewire_del_device_iter(iter);

    ESP_LOGI(TAG, "Hardware initialized. Entering continuous loop...\n");

    /* =========================================================================
     * CONTINUOUS 10-SECOND LOOP
     * ========================================================================= */
    while(1) {
        ESP_LOGI(TAG, "--- POLLING SENSORS ---");

        // 1. WIND PULSES
        unsigned long wind_pulses = 0;
        gpio_set_level(TRIG_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100)); 
        uint8_t uart_data[64] = {0};
        int len = uart_read_bytes(UART_NUM_1, uart_data, 63, pdMS_TO_TICKS(500));
        if (len > 0) {
            uart_data[len] = '\0';
            sscanf((char*)uart_data, "Pulses: %lu", &wind_pulses);
        }
        gpio_set_level(TRIG_PIN, 0);

        // 2. MOISTURE SENSORS
        long m1 = 0, m2 = 0, m3 = 0;
        for(int i = 0; i < 20; i++) {
            int r1, r2, r3;
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_1_CHAN, &r1);
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_2_CHAN, &r2);
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_3_CHAN, &r3);
            m1 += r1; m2 += r2; m3 += r3;
            vTaskDelay(pdMS_TO_TICKS(5)); 
        }

        // 3. SHT30 AIR SENSOR
        float air_temp = 0.0, air_hum = 0.0;
        uint8_t sht_cmd[2] = {0x2C, 0x06};
        uint8_t sht_data[6];
        i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
        vTaskDelay(pdMS_TO_TICKS(50)); 
        esp_err_t sht_err = i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000);
        if (sht_err == ESP_OK) {
            air_temp = -45.0 + (175.0 * ((sht_data[0] << 8) | sht_data[1]) / 65535.0);
            air_hum = 100.0 * ((sht_data[3] << 8) | sht_data[4]) / 65535.0;
        } else {
            ESP_LOGE(TAG, "SHT30 Read Failed: %s", esp_err_to_name(sht_err));
        }

        // 4. LTR390 LIGHT SENSOR
        uint32_t als_val = 0;
        uint8_t ltr_reg = 0x0D; 
        uint8_t ltr_data[3] = {0};
        esp_err_t ltr_err = i2c_master_transmit_receive(ltr390_handle, &ltr_reg, 1, ltr_data, 3, 1000);
        if (ltr_err == ESP_OK) {
            als_val = ltr_data[0] | (ltr_data[1] << 8) | ((ltr_data[2] & 0x0F) << 16);
        } else {
            ESP_LOGE(TAG, "LTR390 Read Failed: %s", esp_err_to_name(ltr_err));
        }

        // 5. DS18B20 SOIL SENSOR
        float soil_temp = -99.9; 
        if (ds18b20 != NULL) {
            ds18b20_trigger_temperature_conversion(ds18b20);
            vTaskDelay(pdMS_TO_TICKS(800)); 
            ds18b20_get_temperature(ds18b20, &soil_temp);
        }

        // 6. OUTPUT RESULTS
        printf("Wind Pulses: %lu\n", wind_pulses);
        printf("Moisture:    S1:%ld | S2:%ld | S3:%ld\n", m1/20, m2/20, m3/20);
        printf("Air Temp:    %.2f C\n", air_temp);
        printf("Air Hum:     %.2f %%\n", air_hum);
        printf("Ambient Lgt: %lu\n", als_val);
        printf("Soil Temp:   %.2f C\n", soil_temp);
        printf("---------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(8500)); 
    }
}