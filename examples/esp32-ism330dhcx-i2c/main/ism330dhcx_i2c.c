/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "ism330dhcx.h"
#include "ism330dhcx_reg.h"

static const char *TAG = "ISM330DHCX_TEST";

// I2C Configuration
#define I2C_MASTER_SCL_IO           9      /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           8      /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// ISM330DHCX I2C Address (SA0 = 0)
#define ISM330DHCX_I2C_ADDR         (ISM330DHCX_I2C_ADD_L >> 1)

// Global sensor object (non-static so other tasks can access it)
ISM330DHCX_Object_t imu_obj;

/**
 * @brief I2C master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, 
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief I2C write wrapper for ISM330DHCX driver
 */
static int32_t platform_write(uint16_t dev_addr, uint16_t reg, uint8_t *bufp, uint16_t len)
{
    esp_err_t ret;
    uint8_t *data = malloc(len + 1);
    if (data == NULL) {
        return -1;
    }

    data[0] = (uint8_t)reg;
    memcpy(&data[1], bufp, len);

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, (uint8_t)dev_addr, 
                                      data, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    free(data);
    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief I2C read wrapper for ISM330DHCX driver
 */
static int32_t platform_read(uint16_t dev_addr, uint16_t reg, uint8_t *bufp, uint16_t len)
{
    esp_err_t ret;
    uint8_t reg_addr = (uint8_t)reg;

    ret = i2c_master_write_read_device(I2C_MASTER_NUM, (uint8_t)dev_addr,
                                        &reg_addr, 1, bufp, len, 
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief Delay function wrapper for ISM330DHCX driver
 */
static void platform_delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * @brief Init function wrapper for ISM330DHCX driver
 */
static int32_t platform_init(void)
{
    return 0;
}

/**
 * @brief DeInit function wrapper for ISM330DHCX driver
 */
static int32_t platform_deinit(void)
{
    return 0;
}

/**
 * @brief Initialize ISM330DHCX sensor
 */
static esp_err_t imu_init(void)
{
    ISM330DHCX_IO_t io_ctx;
    uint8_t id;
    int32_t ret;

    // Configure IO interface
    io_ctx.BusType     = ISM330DHCX_I2C_BUS;
    io_ctx.Address     = ISM330DHCX_I2C_ADDR;
    io_ctx.Init        = platform_init;
    io_ctx.DeInit      = platform_deinit;
    io_ctx.ReadReg     = platform_read;
    io_ctx.WriteReg    = platform_write;
    io_ctx.GetTick     = NULL;
    io_ctx.Delay       = platform_delay;

    // Register bus IO
    ret = ISM330DHCX_RegisterBusIO(&imu_obj, &io_ctx);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to register bus IO");
        return ESP_FAIL;
    }

    // Read WHO_AM_I register
    ret = ISM330DHCX_ReadID(&imu_obj, &id);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ESP_FAIL;
    }

    if (id != ISM330DHCX_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X)", id, ISM330DHCX_ID);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ISM330DHCX detected! Device ID: 0x%02X", id);

    // Initialize the sensor
    ret = ISM330DHCX_Init(&imu_obj);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        return ESP_FAIL;
    }

    // Enable accelerometer (416 Hz, 2g full scale)
    ret = ISM330DHCX_ACC_Enable(&imu_obj);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer");
        return ESP_FAIL;
    }

    ret = ISM330DHCX_ACC_SetOutputDataRate(&imu_obj, 416.0f);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer ODR");
        return ESP_FAIL;
    }

    ret = ISM330DHCX_ACC_SetFullScale(&imu_obj, 2);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer full scale");
        return ESP_FAIL;
    }

    // Enable gyroscope (416 Hz, 2000 dps full scale)
    ret = ISM330DHCX_GYRO_Enable(&imu_obj);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to enable gyroscope");
        return ESP_FAIL;
    }

    ret = ISM330DHCX_GYRO_SetOutputDataRate(&imu_obj, 416.0f);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope ODR");
        return ESP_FAIL;
    }

    ret = ISM330DHCX_GYRO_SetFullScale(&imu_obj, 2000);
    if (ret != ISM330DHCX_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope full scale");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ISM330DHCX initialized successfully");
    return ESP_OK;
}

/**
 * @brief FreeRTOS task to read and display sensor data
 */
static void imu_read_task(void *pvParameters)
{
    ISM330DHCX_Axes_t acc_data;
    ISM330DHCX_Axes_t gyro_data;
    
    while (1) {
        // Read accelerometer data
        if (ISM330DHCX_ACC_GetAxes(&imu_obj, &acc_data) == ISM330DHCX_OK) {
            // Accelerometer values are in mg (milli-g)
            ESP_LOGI(TAG, "Accelerometer [mg]: X=%.2f, Y=%.2f, Z=%.2f", 
                     acc_data.x / 1.0f, acc_data.y / 1.0f, acc_data.z / 1.0f);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
        }

        // Read gyroscope data
        if (ISM330DHCX_GYRO_GetAxes(&imu_obj, &gyro_data) == ISM330DHCX_OK) {
            // Gyroscope values are in mdps (milli-degrees per second)
            ESP_LOGI(TAG, "Gyroscope [mdps]: X=%.2f, Y=%.2f, Z=%.2f", 
                     gyro_data.x / 1.0f, gyro_data.y / 1.0f, gyro_data.z / 1.0f);
        } else {
            ESP_LOGE(TAG, "Failed to read gyroscope data");
        }

        ESP_LOGI(TAG, "----------------------------------------");
        
        // Delay between readings
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief FreeRTOS task to monitor for event detections
 * 
 * This task demonstrates event detection features:
 * - Double Tap Detection
 * - Wake Up Detection
 * - Free Fall Detection
 * 
 * Uncomment the events you want to enable below.
 */
static void event_detection_task(void *pvParameters)
{
    static const char *TAG_EVENT = "ISM330DHCX_EVENT";
    ISM330DHCX_Event_Status_t event_status;
    
    ESP_LOGI(TAG_EVENT, "Starting event detection");
    
    // Enable Double Tap Detection on INT1 pin
    if (ISM330DHCX_ACC_Enable_Double_Tap_Detection(&imu_obj, ISM330DHCX_INT1_PIN) == ISM330DHCX_OK) {
        ESP_LOGI(TAG_EVENT, "Double tap detection enabled - tap the device twice quickly!");
    } else {
        ESP_LOGE(TAG_EVENT, "Failed to enable double tap detection");
    }
    
    // Uncomment to enable Wake Up Detection on INT2 pin
    // if (ISM330DHCX_ACC_Enable_Wake_Up_Detection(&imu_obj, ISM330DHCX_INT2_PIN) == ISM330DHCX_OK) {
    //     ESP_LOGI(TAG_EVENT, "Wake up detection enabled - move the device!");
    // }
    
    // Uncomment to enable Free Fall Detection on INT1 pin
    // if (ISM330DHCX_ACC_Enable_Free_Fall_Detection(&imu_obj, ISM330DHCX_INT1_PIN) == ISM330DHCX_OK) {
    //     ESP_LOGI(TAG_EVENT, "Free fall detection enabled - drop the device gently!");
    // }
    
    // Uncomment to enable Single Tap Detection (can't use with double tap)
    // if (ISM330DHCX_ACC_Enable_Single_Tap_Detection(&imu_obj, ISM330DHCX_INT1_PIN) == ISM330DHCX_OK) {
    //     ESP_LOGI(TAG_EVENT, "Single tap detection enabled - tap once!");
    // }
    
    while (1) {
        // Check for events
        if (ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status) == ISM330DHCX_OK) {
            
            if (event_status.DoubleTapStatus) {
                ESP_LOGW(TAG_EVENT, "*** DOUBLE TAP DETECTED! ***");
            }
            
            if (event_status.TapStatus) {
                ESP_LOGW(TAG_EVENT, "*** SINGLE TAP DETECTED! ***");
            }
            
            if (event_status.WakeUpStatus) {
                ESP_LOGW(TAG_EVENT, "*** WAKE UP EVENT! ***");
            }
            
            if (event_status.FreeFallStatus) {
                ESP_LOGW(TAG_EVENT, "*** FREE FALL DETECTED! ***");
            }
        }
        
        // Check events every 50ms for responsive detection
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ISM330DHCX IMU Test with Event Detection");

    // Initialize I2C bus
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize IMU sensor
    ESP_ERROR_CHECK(imu_init());

    // Create FreeRTOS task for reading sensor data
    xTaskCreate(imu_read_task, "imu_read_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "IMU read task created");
    
    // Create FreeRTOS task for event detection
    xTaskCreate(event_detection_task, "event_detect_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Event detection task created");
    
    ESP_LOGI(TAG, "All tasks started - Try double tapping the device!");
}
