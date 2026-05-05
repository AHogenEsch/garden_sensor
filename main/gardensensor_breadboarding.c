/*
 * Autonomous Deep Sleep Logger with On-Demand Command Terminal
 * Target: ESP32-C3 | Framework: ESP-IDF v6.0
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_spiffs.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "black_box";

/* --- PIN DEFINITIONS --- */
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 // GPIO 0 (Dry Control)
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 // GPIO 1 (Draining Cell)
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 // GPIO 3 (Wet Control)

#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53

/* --- RTC MEMORY (Survives Deep Sleep) --- */
// This variable keeps track of the hour count even when the CPU turns off
RTC_DATA_ATTR int current_hour = 1; 

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;
adc_oneshot_unit_handle_t adc1_handle;


/* =========================================================================
 * HARDWARE & STORAGE INITIALIZATION
 * ========================================================================= */
static void init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true // Automatically formats the 1MB drive on first boot
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS. Did you configure partitions.csv?");
    } else {
        ESP_LOGI(TAG, "SPIFFS Storage Mounted Successfully.");
    }
}

static void init_sensors(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = { .enable_internal_pullup = true }
    };
    i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t sht30_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SHT30_SENSOR_ADDR, .scl_speed_hz = I2C_MASTER_FREQ_HZ };
    i2c_master_bus_add_device(bus_handle, &sht30_cfg, &sht30_handle);

    i2c_device_config_t ltr390_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = LTR390_SENSOR_ADDR, .scl_speed_hz = I2C_MASTER_FREQ_HZ };
    i2c_master_bus_add_device(bus_handle, &ltr390_cfg, &ltr390_handle);

    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config);
    adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config);
    adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config);
}


/* =========================================================================
 * THE COMMAND TERMINAL
 * ========================================================================= */
void enter_command_mode() {
    printf("\n\n=========================================\n");
    printf("   GARDEN HUB COMMAND TERMINAL V1.0      \n");
    printf("=========================================\n");
    printf("Type a command and press ENTER.\n");
    printf("Available commands: HELLO, DUMP, CLEAR, EXIT\n\n");

    char line[128];

    while(1) {
        printf("Hub> ");
        fflush(stdout);

        // Wait infinitely for user input
        if (fgets(line, sizeof(line), stdin) != NULL) {
            
            // Strip the newline character
            line[strcspn(line, "\r\n")] = 0; 
            
            if (strlen(line) == 0) continue; // Ignore empty enter presses

            if (strcmp(line, "HELLO") == 0) {
                printf("Hello World! The command interface is working perfectly.\n");
            } 
            else if (strcmp(line, "DUMP") == 0) {
                printf("\n--- BEGIN CSV DATA DUMP ---\n");
                printf("Hour, S1(Dry), S2(Drain), S3(Wet), Temp(C), Hum(%%), Light\n");
                
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
            else if (strcmp(line, "CLEAR") == 0) {
                remove("/spiffs/data.csv");
                current_hour = 1; // Reset the RTC hour counter
                printf("Data file deleted. RTC Hour Counter reset to 1.\n");
            }
            else if (strcmp(line, "EXIT") == 0) {
                printf("Exiting terminal. Proceeding to log data and sleep...\n");
                break;
            }
            else {
                printf("Unrecognized command: '%s'\n", line);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}


/* =========================================================================
 * MAIN APPLICATION LOOP
 * ========================================================================= */
void app_main(void) {
    // If the board was physically unplugged, reset the hour counter. 
    // If it is just waking up from deep sleep, leave the counter alone.
    if (esp_sleep_get_wakeup_causes() == 0) {
        current_hour = 1;
    }

    init_spiffs();

    printf("\n=== SYSTEM AWAKE (HOUR %d) ===\n", current_hour);
    printf("Waiting 10 seconds. Press ENTER to access the Command Terminal...\n");

    // 1. Wait for Serial Input (The 10-Second Window)
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    int result = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

    if (result > 0) {
        // The user pressed a key during the 10 seconds!
        enter_command_mode();
    } else {
        printf("No input detected. Proceeding with autonomous logging...\n");
    }

    // 2. Wake up sensors and gather data
    init_sensors();
    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];
    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000);

    float temp_c = 0.0, humidity = 0.0;
    uint32_t als_raw = 0;
    long m1_sum = 0, m2_sum = 0, m3_sum = 0;

    for(int i = 0; i < 50; i++) {
        int r1, r2, r3;
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

    // 3. Append the CSV String to Permanent Storage
    FILE* f = fopen("/spiffs/data.csv", "a");
    if (f != NULL) {
        fprintf(f, "%d, %ld, %ld, %ld, %.2f, %.2f, %lu\n", 
                current_hour, (m1_sum/50), (m2_sum/50), (m3_sum/50), temp_c, humidity, als_raw);
        fclose(f);
        ESP_LOGI(TAG, "Data successfully written to drive.");
    } else {
        ESP_LOGE(TAG, "Failed to open file for appending.");
    }

    current_hour++;

    // 4. Power Down the System (Deep Sleep for 60 Minutes)
    ESP_LOGI(TAG, "Entering Deep Sleep for 60 minutes. Goodnight!");
    esp_deep_sleep(3600000000ULL); // Time is in microseconds (60 * 60 * 1,000,000)
}