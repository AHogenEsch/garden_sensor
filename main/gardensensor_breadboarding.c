/*
 * Autonomous Garden Hub - Local Storage Logger
 * Target: ESP32-C3 | Framework: ESP-IDF v6.0
 * Generated on: Friday, June 5, 2026 at 9:35 PM PDT
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "onewire_bus.h"
#include "ds18b20.h"

static const char *TAG = "garden_hub";

/* --- LOGGING CONFIGURATION --- */
#define SLEEP_DURATION_MINUTES      60

/* --- PIN DEFINITIONS --- */
#define SENSOR_POWER_PIN            GPIO_NUM_5 // Connected to LP0701 Gate

#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define ONEWIRE_BUS_GPIO            7          

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 // GPIO 0 
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 // GPIO 1 
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 // GPIO 3 

#define SHT30_SENSOR_ADDR           0x44

/* --- RTC MEMORY --- */
RTC_DATA_ATTR int current_hour = 1; 

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
adc_oneshot_unit_handle_t adc1_handle;


/* =========================================================================
 * HARDWARE & STORAGE INITIALIZATION
 * ========================================================================= */
static void init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true 
    };
    if (esp_vfs_spiffs_register(&conf) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS.");
    } else {
        ESP_LOGI(TAG, "SPIFFS Mounted Successfully.");
    }
}

static void init_sensors(void) {
    // 1. Init I2C Bus & Devices
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

    // 2. Init ADC for Moisture Sensors
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config));
}

/* =========================================================================
 * THE SINGLE-KEY COMMAND TERMINAL
 * ========================================================================= */
void enter_command_mode() {
    while (fgetc(stdin) != EOF); 

    printf("\n\n=========================================\n");
    printf("   GARDEN HUB COMMAND TERMINAL V2.0      \n");
    printf("=========================================\n");
    printf("Press a single key:\n");
    printf(" [H] - HELLO (Test connection)\n");
    printf(" [D] - DUMP  (Print CSV data)\n");
    printf(" [C] - CLEAR (Erase data & reset hour)\n");
    printf(" [X] - EXIT  (Continue to logging & sleep)\n");
    printf("=========================================\n\n");
    
    printf("Hub> ");
    fflush(stdout);

    while(1) {
        int c = fgetc(stdin);
        
        if (c != EOF && c != '\n' && c != '\r' && c != 0 && c != 255) {
            printf("%c\n", (char)c); 

            if (c == 'h' || c == 'H') {
                printf("Hello World! Connection is solid.\n");
            } 
            else if (c == 'd' || c == 'D') {
                printf("\n--- BEGIN CSV DATA DUMP ---\n");
                printf("Hour, S1, S2, S3, AirTemp(C), AirHum(%%), SoilTemp(C)\n");
                
                FILE* f = fopen("/spiffs/data.csv", "r");
                if (f == NULL) {
                    printf("No data found. The file is empty or does not exist.\n");
                } else {
                    char buffer[128];
                    while (fgets(buffer, sizeof(buffer), f) != NULL) {
                        printf("%s", buffer);
                    }
                    fclose(f);
                }
                printf("--- END DATA DUMP ---\n");
            } 
            else if (c == 'c' || c == 'C') {
                remove("/spiffs/data.csv");
                current_hour = 1; 
                printf("Data file deleted. RTC Hour Counter reset to 1.\n");
            }
            else if (c == 'x' || c == 'X') {
                printf("Exiting terminal. Proceeding to autonomous logging...\n");
                break;
            }
            else {
                printf("Unrecognized command. Please use H, D, C, or X.\n");
            }
            printf("\nHub> ");
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

/* =========================================================================
 * MAIN APPLICATION LOOP
 * ========================================================================= */
void app_main(void) {
    if (esp_sleep_get_wakeup_causes() == 0) {
        current_hour = 1;
    }

    init_spiffs();

    printf("\n=== SYSTEM AWAKE (HOUR %d) ===\n", current_hour);
    printf("Waiting 10 seconds. Press ANY KEY to access Terminal...\n");

    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); 

    int key_pressed = 0;
    
    // The 10-Second Countdown
    for (int i = 0; i < 100; i++) {
        int c = fgetc(stdin);
        if (c != EOF && c != '\n' && c != '\r' && c != 0 && c != 255) {
            key_pressed = 1;
            break;
        }
        if (i % 10 == 0) {
            printf("%d... ", 10 - (i / 10));
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
    printf("\n");

    if (key_pressed) {
        printf("\nKeyboard input detected!\n");
        enter_command_mode();
    } else {
        printf("No input detected. Logging data...\n");
    }

    /* --- 1. TURN ON SENSORS --- */
    gpio_set_direction(SENSOR_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_POWER_PIN, 0); 
    vTaskDelay(pdMS_TO_TICKS(500)); 

    init_sensors();

    /* --- 2. THE 1-WIRE SETUP & TRIGGER --- */
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
        ds18b20_set_resolution(ds18b20, DS18B20_RESOLUTION_12B);
        ds18b20_trigger_temperature_conversion(ds18b20);
    } else {
        ESP_LOGE(TAG, "DS18B20 not found on bus!");
    }
    onewire_del_device_iter(iter);


    /* --- 3. GATHER MOISTURE & SHT30 DATA --- */
    float air_temp = 0.0, air_hum = 0.0;
    long m1_sum = 0, m2_sum = 0, m3_sum = 0;

    for(int i = 0; i < 50; i++) {
        int r1, r2, r3;
        adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_1_CHAN, &r1);
        adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_2_CHAN, &r2);
        adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_3_CHAN, &r3);
        m1_sum += r1; m2_sum += r2; m3_sum += r3;
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }

    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];
    i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    
    if (i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000) == ESP_OK) {
        uint16_t temp_raw = (sht_data[0] << 8) | sht_data[1];
        uint16_t hum_raw = (sht_data[3] << 8) | sht_data[4];
        air_temp = -45.0 + (175.0 * temp_raw / 65535.0);
        air_hum = 100.0 * hum_raw / 65535.0;
    }


    /* --- 4. RETRIEVE THE DS18B20 DATA --- */
    vTaskDelay(pdMS_TO_TICKS(450)); 
    
    float soil_temp = -99.9; 
    if (ds18b20 != NULL) {
        ds18b20_get_temperature(ds18b20, &soil_temp);
    }

    /* --- 5. APPEND TO STORAGE --- */
    FILE* f = fopen("/spiffs/data.csv", "a");
    if (f != NULL) {
        fprintf(f, "%d, %ld, %ld, %ld, %.2f, %.2f, %.2f\n", 
                current_hour, (m1_sum/50), (m2_sum/50), (m3_sum/50), air_temp, air_hum, soil_temp);
        fclose(f);
        ESP_LOGI(TAG, "Data successfully written to drive.");
    } else {
        ESP_LOGE(TAG, "Failed to open file for appending.");
    }

    /* --- 6. TURN OFF SENSORS AND BUS --- */
    if (ds18b20 != NULL) {
        ds18b20_del_device(ds18b20);
    }
    onewire_bus_del(ow_bus);

    gpio_set_level(SENSOR_POWER_PIN, 1); 
    ESP_LOGI(TAG, "Power Rail: OFF");

    current_hour++;

    /* --- 7. DEEP SLEEP --- */
    uint64_t sleep_time_us = (uint64_t)SLEEP_DURATION_MINUTES * 60ULL * 1000000ULL;
    ESP_LOGI(TAG, "Entering Deep Sleep for %d minutes...", SLEEP_DURATION_MINUTES);
    esp_deep_sleep(sleep_time_us); 
}