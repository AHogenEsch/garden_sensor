/*
 * Autonomous Garden Node - Full Sensor BLE Beacon
 * Target: ESP32-C3 | Framework: ESP-IDF v6.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "onewire_bus.h"
#include "ds18b20.h"

// BLE Includes
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

static const char *TAG = "GARDEN_NODE";

/* --- CONFIGURATION --- */
#define SLEEP_DURATION_SECONDS      10  // Set to 10 for testing
#define CUSTOM_COMPANY_ID           0x1337

/* --- PIN DEFINITIONS --- */
#define SENSOR_POWER_PIN            GPIO_NUM_5
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define ONEWIRE_BUS_GPIO            7
#define TRIG_PIN                    9
#define UART_RX_PIN                 10

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 

#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53

/* --- THE DATA STRUCTURE --- */
typedef struct __attribute__((packed)) {
    uint32_t wind_pulses;
    uint16_t moisture[3];
    int16_t soil_temp;  
    int16_t air_temp;   
    uint16_t air_hum;   
    uint32_t light_level;
} sensor_data_t;

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;
adc_oneshot_unit_handle_t adc1_handle;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20, 
    .adv_int_max       = 0x40, 
    .adv_type          = ADV_TYPE_NONCONN_IND, 
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void app_main(void) {
    ESP_LOGI(TAG, "=== NODE WAKING UP ===");

    /* --- 0. SEVER ALL JTAG CONNECTIONS --- */
    gpio_reset_pin(SENSOR_POWER_PIN);
    gpio_reset_pin(I2C_MASTER_SDA_IO);
    gpio_reset_pin(I2C_MASTER_SCL_IO);
    gpio_reset_pin(ONEWIRE_BUS_GPIO);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* --- 1. ENERGIZE POWER RAIL SAFELY --- */
    gpio_set_direction(SENSOR_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_POWER_PIN, 0); 
    vTaskDelay(pdMS_TO_TICKS(1000)); // Power stabilization

    /* --- 2. INITIALIZE ALL PROTOCOLS --- */
    // UART
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);
    uart_config_t uart_config = { .baud_rate = 9600, .data_bits = UART_DATA_8_BITS, 
                                  .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, 
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

    // I2C
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0, .sda_io_num = I2C_MASTER_SDA_IO, .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .flags = { .enable_internal_pullup = true }
    };
    i2c_new_master_bus(&bus_config, &bus_handle);
    i2c_device_config_t sht30_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SHT30_SENSOR_ADDR, .scl_speed_hz = 100000 };
    i2c_master_bus_add_device(bus_handle, &sht30_cfg, &sht30_handle);
    i2c_device_config_t ltr390_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = LTR390_SENSOR_ADDR, .scl_speed_hz = 100000 };
    i2c_master_bus_add_device(bus_handle, &ltr390_cfg, &ltr390_handle);

    // ADC
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config);
    adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config);
    adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config);

    // 1-Wire
    onewire_bus_handle_t ow_bus;
    onewire_bus_config_t ow_config = { .bus_gpio_num = ONEWIRE_BUS_GPIO };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    onewire_new_bus_rmt(&ow_config, &rmt_config, &ow_bus);
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_ow_device;
    ds18b20_device_handle_t ds18b20 = NULL;
    onewire_new_device_iter(ow_bus, &iter);
    if (onewire_device_iter_get_next(iter, &next_ow_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        ds18b20_new_device_from_enumeration(&next_ow_device, &ds_cfg, &ds18b20);
    }
    onewire_del_device_iter(iter);

    /* --- 3. THE PARALLEL TRIGGER PARADIGM --- */
    // Trigger everything simultaneously so we only have to wait once
    gpio_set_level(TRIG_PIN, 1); // Wake ATtiny
    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000); // Wake LTR390
    if (ds18b20 != NULL) ds18b20_trigger_temperature_conversion(ds18b20); // Start DS18B20 conversion

    vTaskDelay(pdMS_TO_TICKS(800)); // The Unified Hardware Wait Window

    /* --- 4. GATHER ALL DATA --- */
    unsigned long final_wind_pulses = 0;
    uint8_t uart_data[64] = {0};
    int len = uart_read_bytes(UART_NUM_1, uart_data, 63, pdMS_TO_TICKS(500));
    if (len > 0) {
        uart_data[len] = '\0';
        sscanf((char*)uart_data, "Pulses: %lu", &final_wind_pulses);
    }
    gpio_set_level(TRIG_PIN, 0); // Sleep ATtiny
    uart_driver_delete(UART_NUM_1); 

    uint32_t als_val = 0;
    uint8_t ltr_reg = 0x0D; 
    uint8_t ltr_data[3] = {0};
    if (i2c_master_transmit_receive(ltr390_handle, &ltr_reg, 1, ltr_data, 3, 1000) == ESP_OK) {
        als_val = ltr_data[0] | (ltr_data[1] << 8) | ((ltr_data[2] & 0x0F) << 16);
    }

    float air_temp = 0.0, air_hum = 0.0;
    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];
    i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000) == ESP_OK) {
        air_temp = -45.0 + (175.0 * ((sht_data[0] << 8) | sht_data[1]) / 65535.0);
        air_hum = 100.0 * ((sht_data[3] << 8) | sht_data[4]) / 65535.0;
    }

    long m1=0, m2=0, m3=0;
    for(int i=0; i<10; i++) {
        int r1, r2, r3;
        adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_1_CHAN, &r1);
        adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_2_CHAN, &r2);
        adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_3_CHAN, &r3);
        m1+=r1; m2+=r2; m3+=r3;
    }

    float soil_temp = -99.9;
    if (ds18b20 != NULL) {
        ds18b20_get_temperature(ds18b20, &soil_temp);
        ds18b20_del_device(ds18b20);
    }
    onewire_bus_del(ow_bus);

    /* --- POWER DOWN SENSORS --- */
    gpio_set_level(SENSOR_POWER_PIN, 1); 

    /* --- 5. PACK THE BLE STRUCT --- */
    sensor_data_t payload;
    payload.wind_pulses = final_wind_pulses;
    payload.moisture[0] = m1/10;
    payload.moisture[1] = m2/10;
    payload.moisture[2] = m3/10;
    payload.soil_temp = (int16_t)(soil_temp * 100);
    payload.air_temp = (int16_t)(air_temp * 100);
    payload.air_hum = (uint16_t)(air_hum * 100);
    payload.light_level = als_val; 

    /* --- 6. START BLE BEACON --- */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    uint8_t raw_adv_data[31] = {0};
    raw_adv_data[0] = sizeof(sensor_data_t) + 3; 
    raw_adv_data[1] = 0xFF;                      
    raw_adv_data[2] = CUSTOM_COMPANY_ID & 0xFF;  
    raw_adv_data[3] = (CUSTOM_COMPANY_ID >> 8) & 0xFF; 
    
    memcpy(&raw_adv_data[4], &payload, sizeof(sensor_data_t));

    esp_ble_gap_config_adv_data_raw(raw_adv_data, raw_adv_data[0] + 1);
    esp_ble_gap_start_advertising(&adv_params);

    ESP_LOGI(TAG, "Broadcasting -> Wind: %lu | M: %u,%u,%u | AirT: %.2fC | AirH: %.2f%% | Lgt: %lu | SoilT: %.2fC",
             payload.wind_pulses, payload.moisture[0], payload.moisture[1], payload.moisture[2],
             payload.air_temp / 100.0, payload.air_hum / 100.0, payload.light_level, payload.soil_temp / 100.0);

    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_ble_gap_stop_advertising();

    /* --- 7. DEEP SLEEP --- */
    ESP_LOGI(TAG, "Going to sleep for %d seconds...", SLEEP_DURATION_SECONDS);
    esp_deep_sleep(SLEEP_DURATION_SECONDS * 1000000ULL);
}