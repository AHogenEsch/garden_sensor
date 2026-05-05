/*
 * RAM-Only Hourly Soil Hydrology Logger
 * Target: ESP32-C3 | Framework: ESP-IDF v6.0
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "hourly_logger";

/* --- PIN DEFINITIONS --- */
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define MOISTURE_SENSOR_1_CHAN      ADC_CHANNEL_0 // GPIO 0 (Dry Control)
#define MOISTURE_SENSOR_2_CHAN      ADC_CHANNEL_1 // GPIO 1 (Draining Cell)
#define MOISTURE_SENSOR_3_CHAN      ADC_CHANNEL_3 // GPIO 3 (Wet Control)

/* --- SENSOR ADDRESSES --- */
#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53

/* --- TEST SCOPE CONFIGURATION --- */
#define BURST_READINGS              10   // Number of readings to average per hour

/* --- GLOBAL HANDLES --- */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;
adc_oneshot_unit_handle_t adc1_handle;

/* --- RAM TRACKING VARIABLES --- */
int current_hour = 0;

// Baselines (Set during Hour 0)
int drain_hr0 = 0, wet_hr0 = 0, dry_hr0 = 0;

// Previous Hour (For calculating 1hr shift)
int drain_prev = 0, wet_prev = 0;


/* =========================================================================
 * HARDWARE INITIALIZATION
 * ========================================================================= */
static void init_hardware(void) {
    // I2C Init
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

    // ADC Init
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_1_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_2_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOISTURE_SENSOR_3_CHAN, &config));
}

float calc_relative_dryness(int dry_val, int wet_val, int drain_val) {
    if (dry_val == wet_val) return 0.0f;
    float percentage = ((float)(drain_val - wet_val) / (float)(dry_val - wet_val)) * 100.0f;
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    return percentage;
}

/* =========================================================================
 * MAIN APPLICATION LOOP
 * ========================================================================= */
void app_main(void) {
    init_hardware();
    ESP_LOGI(TAG, "Hardware armed. Commencing RAM-Backed Hourly Logging.");

    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];
    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000);

    while (1) {
        
        /* --- 1. ENVIRONMENT POLLING --- */
        i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
        vTaskDelay(pdMS_TO_TICKS(50)); 

        float temp_f = 0.0, humidity = 0.0;
        uint32_t als_raw = 0;

        // Retrieve SHT30 & Convert to Fahrenheit
        if (i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000) == ESP_OK) {
            uint16_t temp_raw = (sht_data[0] << 8) | sht_data[1];
            uint16_t hum_raw = (sht_data[3] << 8) | sht_data[4];
            float temp_c = -45.0 + (175.0 * temp_raw / 65535.0);
            temp_f = (temp_c * 9.0 / 5.0) + 32.0;
            humidity = 100.0 * hum_raw / 65535.0;
        }

        // Retrieve LTR390
        uint8_t ltr_reg = 0x0D;
        uint8_t ltr_data[3];
        if (i2c_master_transmit_receive(ltr390_handle, &ltr_reg, 1, ltr_data, sizeof(ltr_data), 1000) == ESP_OK) {
            als_raw = ltr_data[0] | (ltr_data[1] << 8) | (ltr_data[2] << 16);
        }

        /* --- 2. BURST SAMPLING (ADC) --- */
        long sum_dry = 0, sum_drain = 0, sum_wet = 0;
        
        for(int i = 0; i < BURST_READINGS; i++) {
            int r_dry = 0, r_drain = 0, r_wet = 0;
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_1_CHAN, &r_dry);
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_2_CHAN, &r_drain);
            adc_oneshot_read(adc1_handle, MOISTURE_SENSOR_3_CHAN, &r_wet);
            
            sum_dry += r_dry;
            sum_drain += r_drain;
            sum_wet += r_wet;
            
            vTaskDelay(pdMS_TO_TICKS(100)); // 100ms gap
        }

        int dry_now = sum_dry / BURST_READINGS;
        int drain_now = sum_drain / BURST_READINGS;
        int wet_now = sum_wet / BURST_READINGS;

        /* --- 3. KINEMATIC CALCULATIONS --- */
        // Lock in baselines if this is the first run
        if (current_hour == 0) {
            drain_hr0 = drain_now;
            wet_hr0 = wet_now;
            dry_hr0 = dry_now;
            drain_prev = drain_now;
            wet_prev = wet_now;
        }

        int delta_drain_1h = drain_now - drain_prev;
        int delta_wet_1h = wet_now - wet_prev;

        int delta_drain_total = drain_now - drain_hr0;
        int delta_wet_total = wet_now - wet_hr0;

        float rel_dryness = calc_relative_dryness(dry_now, wet_now, drain_now);

        /* --- 4. THE HOURLY REPORT --- */
        printf("\n");
        ESP_LOGI(TAG, "=========================================================");
        ESP_LOGI(TAG, "   [HOUR %d] CONTINUOUS DRAINAGE REPORT                  ", current_hour);
        ESP_LOGI(TAG, "=========================================================");
        
        ESP_LOGI(TAG, " ENVIRONMENT:");
        ESP_LOGI(TAG, "  - Air Temp      : %.1f F", temp_f);
        ESP_LOGI(TAG, "  - Humidity      : %.1f %%", humidity);
        ESP_LOGI(TAG, "  - Light Level   : %lu", als_raw);
        ESP_LOGI(TAG, " ");

        ESP_LOGI(TAG, " DRAINING CELL (Clay Core):");
        ESP_LOGI(TAG, "  - Current ADC   : %d", drain_now);
        ESP_LOGI(TAG, "  - Shift (1hr)   : %+d", delta_drain_1h);
        ESP_LOGI(TAG, "  - Total Shift   : %+d (Since Session Start)", delta_drain_total);
        ESP_LOGI(TAG, "  - Rel. Dryness  : %.1f %%", rel_dryness);
        
        ESP_LOGI(TAG, " ");
        ESP_LOGI(TAG, " WET CONTROL BASELINE:");
        ESP_LOGI(TAG, "  - Current ADC   : %d", wet_now);
        ESP_LOGI(TAG, "  - Shift (1hr)   : %+d", delta_wet_1h);
        ESP_LOGI(TAG, "  - Total Shift   : %+d (Since Session Start)", delta_wet_total);
        
        ESP_LOGI(TAG, " ");
        ESP_LOGI(TAG, " DRY CONTROL BASELINE:");
        ESP_LOGI(TAG, "  - Current ADC   : %d", dry_now);
        ESP_LOGI(TAG, "=========================================================\n");

        /* --- 5. PREPARE FOR NEXT CYCLE --- */
        drain_prev = drain_now;
        wet_prev = wet_now;
        current_hour++;

        ESP_LOGI(TAG, "Sleeping for 60 minutes...\n");
        
        // Wait exactly 60 minutes (minus the 1 second it took to burst read)
        vTaskDelay(pdMS_TO_TICKS((60 * 60 * 1000) - 1000)); 
    }
}