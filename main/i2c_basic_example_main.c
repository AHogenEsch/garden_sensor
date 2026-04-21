/*
 * Garden Sensor Node - I2C Bare Metal Implementation
 * Target: ESP32-C3
 * Framework: ESP-IDF v5.x
 * * This architecture uses the modernized v5 I2C driver model, where a single 
 * physical bus is instantiated, and individual sensors are attached to it 
 * as discrete "Device Handles". This prevents bus collisions and simplifies 
 * multi-sensor routing.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "garden_node";

/* ==========================================================================
 * HARDWARE MAPPING & CONSTANTS
 * ========================================================================== */
// DevKit I2C Routing
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
// 100kHz is the standard "Safe Mode" speed for I2C. 
#define I2C_MASTER_FREQ_HZ          100000

// Sensor Memory Addresses (Determined by hardware pull-up resistors on the breakouts)
#define SHT30_SENSOR_ADDR           0x44
#define LTR390_SENSOR_ADDR          0x53

/* ==========================================================================
 * GLOBAL HANDLES
 * ========================================================================== 
 * Declared globally so the init function can create them, and the 
 * infinite main loop can access them continuously.
 */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sht30_handle;
i2c_master_dev_handle_t ltr390_handle;


/* ==========================================================================
 * FUNCTION: init_sensors
 * PURPOSE:  Energizes the physical pins and registers the devices to the OS.
 * ========================================================================== */
static void init_sensors(void) {
    // 1. Configure the Physical Copper (The Master Bus)
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // The ESP32 has weak internal pullups. We enable them here as a safety net,
        // though the Adafruit breakout boards have their own physical resistors.
        .flags.enable_internal_pullup = true, 
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // 2. Attach the SHT30 Temperature/Humidity Sensor
    i2c_device_config_t sht30_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT30_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &sht30_cfg, &sht30_handle));

    // 3. Attach the LTR390 UV/Light Sensor
    i2c_device_config_t ltr390_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LTR390_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ltr390_cfg, &ltr390_handle));
}


/* ==========================================================================
 * FUNCTION: app_main
 * PURPOSE:  The primary FreeRTOS entry point. Acts as the brain of the node.
 * ========================================================================== */
void app_main(void) {
    // Bring up the hardware
    init_sensors();
    ESP_LOGI(TAG, "Hardware architecture established. I2C Bus active.");

    /* --- SENSOR BOOT COMMANDS --- */
    // The SHT30 command for a "Single Shot, High Repeatability" measurement.
    // 0x2C is the clock stretching command, 0x06 sets the high detail mode.
    uint8_t sht_cmd[2] = {0x2C, 0x06};
    uint8_t sht_data[6];

    // The LTR390 powers on in standby mode to save battery. 
    // LTR390 Command: Write 0x02 to Register 0x00 (MAIN_CTRL) to turn on Ambient Light Sensing
    uint8_t ltr_init_cmd[2] = {0x00, 0x02};
    esp_err_t ltr_wake_err = i2c_master_transmit(ltr390_handle, ltr_init_cmd, sizeof(ltr_init_cmd), 1000);
    if (ltr_wake_err == ESP_OK) {
        ESP_LOGI(TAG, "LTR390 awakened from standby.");
    } else {
        ESP_LOGE(TAG, "CRITICAL: Failed to wake LTR390! Check wiring to Pin 4 (SDA) and Pin 6 (SCL).");
    }


    /* --- THE INFINITE EVENT LOOP --- */
    while (1) {
        
        // ---------------------------------------------------------
        // 1. POLL SHT30 (ENVIRONMENTAL)
        // ---------------------------------------------------------
        // Send the trigger command
        i2c_master_transmit(sht30_handle, sht_cmd, sizeof(sht_cmd), 1000);
        
        // The SHT30 silicon requires approximately 15ms to physically measure the air.
        // We yield the processor for 50ms to be safe and avoid blocking other OS tasks.
        vTaskDelay(pdMS_TO_TICKS(50)); 
        
        // Read the 6 bytes back: [Temp MSB, Temp LSB, Temp CRC, Hum MSB, Hum LSB, Hum CRC]
        esp_err_t sht_err = i2c_master_receive(sht30_handle, sht_data, sizeof(sht_data), 1000);
        
        float temp_c = 0.0, humidity = 0.0;
        if (sht_err == ESP_OK) {
            // Bitshift the Most Significant Byte and merge it with the Least Significant Byte
            uint16_t temp_raw = (sht_data[0] << 8) | sht_data[1];
            uint16_t hum_raw = (sht_data[3] << 8) | sht_data[4];
            
            // Apply the manufacturer's datasheet conversion formulas
            temp_c = -45.0 + (175.0 * temp_raw / 65535.0);
            humidity = 100.0 * hum_raw / 65535.0;
        } else {
            ESP_LOGE(TAG, "SHT30 Bus Timeout");
        }

        // ---------------------------------------------------------
        // 2. POLL LTR390 (OPTICAL)
        // ---------------------------------------------------------
        // Ambient Light Data is stored sequentially across 3 registers starting at 0x0D.
        // We write the starting register address, and the sensor automatically auto-increments
        // its internal pointer as we read the next 3 bytes.
        uint8_t ltr_reg = 0x0D;
        uint8_t ltr_data[3];
        
        esp_err_t ltr_err = i2c_master_transmit_receive(ltr390_handle, &ltr_reg, 1, ltr_data, sizeof(ltr_data), 1000);
        
        uint32_t als_raw = 0;
        if (ltr_err == ESP_OK) {
            // Reconstruct the 24-bit value from the three 8-bit registers
            als_raw = ltr_data[0] | (ltr_data[1] << 8) | (ltr_data[2] << 16);
        } else {
            ESP_LOGE(TAG, "LTR390 Bus Timeout");
        }

        // ---------------------------------------------------------
        // 3. SERIAL TELEMETRY OUTPUT
        // ---------------------------------------------------------
        // Push the calculated floats and integers out over the USB-UART bridge
        ESP_LOGI(TAG, "Temp: %.2f C | Humidity: %.2f %% | Light (Raw ALS): %lu", temp_c, humidity, als_raw);

        // Put the FreeRTOS task to sleep for exactly 5000 milliseconds (5 seconds).
        // This prevents the watchdog timer from panicking.
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}