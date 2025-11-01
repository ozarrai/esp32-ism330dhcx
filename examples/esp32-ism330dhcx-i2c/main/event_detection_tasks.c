/*
 * ISM330DHCX Event Detection Examples using FreeRTOS Tasks
 * 
 * This file demonstrates how to use the ISM330DHCX driver's event detection
 * features in separate FreeRTOS tasks:
 * - Free Fall Detection
 * - Wake Up Detection
 * - Single Tap Detection
 * - Double Tap Detection
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ism330dhcx.h"

static const char *TAG_FF = "FREE_FALL";
static const char *TAG_WU = "WAKE_UP";
static const char *TAG_TAP = "TAP";
static const char *TAG_DTAP = "DOUBLE_TAP";

// External reference to the IMU object (assumed to be initialized elsewhere)
extern ISM330DHCX_Object_t imu_obj;

/**
 * @brief FreeRTOS task for Free Fall Detection
 * 
 * This task monitors for free fall events. Free fall is detected when
 * acceleration on all axes is close to 0g for a specified duration.
 * 
 * Configuration:
 * - Threshold: 312mg (configurable via ISM330DHCX_ACC_Set_Free_Fall_Threshold)
 * - Duration: 6 samples at 416Hz (~14.4ms)
 * 
 * @param pvParameters Task parameters (unused)
 */
static void free_fall_detection_task(void *pvParameters)
{
    ISM330DHCX_Event_Status_t event_status;
    
    ESP_LOGI(TAG_FF, "Starting Free Fall Detection");
    
    // Enable free fall detection on INT1 pin
    if (ISM330DHCX_ACC_Enable_Free_Fall_Detection(&imu_obj, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK) {
        ESP_LOGE(TAG_FF, "Failed to enable free fall detection");
        vTaskDelete(NULL);
        return;
    }
    
    // Optional: Customize threshold (0-7 levels: 156mg to 500mg)
    // ISM330DHCX_ACC_Set_Free_Fall_Threshold(&imu_obj, 0x03);  // 312mg
    
    // Optional: Customize duration (0-63, duration = value * 1/ODR)
    // ISM330DHCX_ACC_Set_Free_Fall_Duration(&imu_obj, 0x06);  // 6 samples
    
    ESP_LOGI(TAG_FF, "Free fall detection enabled - Monitoring for events...");
    
    while (1) {
        // Check event status
        if (ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status) == ISM330DHCX_OK) {
            if (event_status.FreeFallStatus) {
                ESP_LOGW(TAG_FF, "*** FREE FALL DETECTED! ***");
                ESP_LOGI(TAG_FF, "Device is in free fall - all axes near 0g");
            }
        }
        
        // Check every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief FreeRTOS task for Wake Up Detection
 * 
 * This task monitors for wake-up events, which occur when the device
 * experiences a sudden motion/acceleration change above a threshold.
 * Useful for motion-activated applications.
 * 
 * Configuration:
 * - Threshold: 2 LSB (configurable, 1 LSB = FS_XL / 64)
 * - Duration: 0 (immediate detection)
 * 
 * @param pvParameters Task parameters (unused)
 */
static void wake_up_detection_task(void *pvParameters)
{
    ISM330DHCX_Event_Status_t event_status;
    
    ESP_LOGI(TAG_WU, "Starting Wake Up Detection");
    
    // Enable wake up detection on INT1 pin
    if (ISM330DHCX_ACC_Enable_Wake_Up_Detection(&imu_obj, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK) {
        ESP_LOGE(TAG_WU, "Failed to enable wake up detection");
        vTaskDelete(NULL);
        return;
    }
    
    // Optional: Customize threshold (0-63, threshold = value * FS_XL/64)
    // For 2g FS: 1 LSB = 31.25mg, value of 2 = 62.5mg
    // ISM330DHCX_ACC_Set_Wake_Up_Threshold(&imu_obj, 0x02);  // ~62.5mg
    
    // Optional: Customize duration (0-3, duration in ODR cycles)
    // ISM330DHCX_ACC_Set_Wake_Up_Duration(&imu_obj, 0x00);  // Immediate
    
    ESP_LOGI(TAG_WU, "Wake up detection enabled - Device is monitoring motion...");
    
    while (1) {
        // Check event status
        if (ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status) == ISM330DHCX_OK) {
            if (event_status.WakeUpStatus) {
                ESP_LOGW(TAG_WU, "*** WAKE UP EVENT DETECTED! ***");
                ESP_LOGI(TAG_WU, "Motion detected - device woken from sleep/idle");
            }
        }
        
        // Check every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief FreeRTOS task for Single Tap Detection
 * 
 * This task monitors for single tap events on the device.
 * A tap is detected when acceleration exceeds a threshold for a short duration.
 * 
 * Configuration:
 * - Threshold: 8 (tap sensitivity on X axis)
 * - Shock: 2 * 8 / ODR = ~38ms maximum duration of over-threshold event
 * - Quiet: 1 * 4 / ODR = ~10ms expected quiet time after tap
 * - Axes: X, Y, and Z enabled
 * 
 * @param pvParameters Task parameters (unused)
 */
static void single_tap_detection_task(void *pvParameters)
{
    ISM330DHCX_Event_Status_t event_status;
    
    ESP_LOGI(TAG_TAP, "Starting Single Tap Detection");
    
    // Enable single tap detection on INT1 pin
    if (ISM330DHCX_ACC_Enable_Single_Tap_Detection(&imu_obj, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK) {
        ESP_LOGE(TAG_TAP, "Failed to enable single tap detection");
        vTaskDelete(NULL);
        return;
    }
    
    // Optional: Customize tap parameters
    // Tap threshold (0-31, higher = less sensitive)
    // ISM330DHCX_ACC_Set_Tap_Threshold(&imu_obj, 0x08);  // Medium sensitivity
    
    // Tap shock time (0-3, duration of over-threshold event)
    // ISM330DHCX_ACC_Set_Tap_Shock_Time(&imu_obj, 0x02);
    
    // Tap quiet time (0-3, expected quiet time after tap)
    // ISM330DHCX_ACC_Set_Tap_Quiet_Time(&imu_obj, 0x01);
    
    ESP_LOGI(TAG_TAP, "Single tap detection enabled - Tap the device once!");
    
    while (1) {
        // Check event status
        if (ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status) == ISM330DHCX_OK) {
            if (event_status.TapStatus) {
                ESP_LOGW(TAG_TAP, "*** SINGLE TAP DETECTED! ***");
                ESP_LOGI(TAG_TAP, "Device was tapped once");
            }
        }
        
        // Check every 50ms for more responsive tap detection
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief FreeRTOS task for Double Tap Detection
 * 
 * This task monitors for double tap events (two taps in quick succession).
 * Useful for gesture-based controls or device wake-up features.
 * 
 * Configuration:
 * - Threshold: 8 (tap sensitivity on X axis)
 * - Shock: 3 * 8 / ODR = ~58ms maximum duration of over-threshold event
 * - Quiet: 3 * 4 / ODR = ~29ms expected quiet time after tap
 * - Duration: 8 * 32 / ODR = ~615ms maximum time between two taps
 * - Axes: X, Y, and Z enabled
 * 
 * @param pvParameters Task parameters (unused)
 */
static void double_tap_detection_task(void *pvParameters)
{
    ISM330DHCX_Event_Status_t event_status;
    
    ESP_LOGI(TAG_DTAP, "Starting Double Tap Detection");
    
    // Enable double tap detection on INT1 pin
    if (ISM330DHCX_ACC_Enable_Double_Tap_Detection(&imu_obj, ISM330DHCX_INT1_PIN) != ISM330DHCX_OK) {
        ESP_LOGE(TAG_DTAP, "Failed to enable double tap detection");
        vTaskDelete(NULL);
        return;
    }
    
    // Optional: Customize tap parameters
    // Tap threshold (0-31, higher = less sensitive)
    // ISM330DHCX_ACC_Set_Tap_Threshold(&imu_obj, 0x08);  // Medium sensitivity
    
    // Tap shock time (0-3)
    // ISM330DHCX_ACC_Set_Tap_Shock_Time(&imu_obj, 0x03);
    
    // Tap quiet time (0-3)
    // ISM330DHCX_ACC_Set_Tap_Quiet_Time(&imu_obj, 0x03);
    
    // Tap duration time (0-15, time between two taps)
    // ISM330DHCX_ACC_Set_Tap_Duration_Time(&imu_obj, 0x08);
    
    ESP_LOGI(TAG_DTAP, "Double tap detection enabled - Tap the device twice quickly!");
    
    while (1) {
        // Check event status
        if (ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status) == ISM330DHCX_OK) {
            if (event_status.DoubleTapStatus) {
                ESP_LOGW(TAG_DTAP, "*** DOUBLE TAP DETECTED! ***");
                ESP_LOGI(TAG_DTAP, "Device was tapped twice in quick succession");
            }
        }
        
        // Check every 50ms for more responsive tap detection
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Example function to create all event detection tasks
 * 
 * Call this function from app_main() after initializing the IMU sensor.
 * You can choose to create all tasks or just the ones you need.
 * 
 * Note: Only one tap detection task should be active at a time
 * (either single or double tap, not both)
 */
void create_event_detection_tasks(void)
{
    // Example 1: Enable Free Fall Detection
    // xTaskCreate(free_fall_detection_task, "free_fall_task", 3072, NULL, 5, NULL);
    
    // Example 2: Enable Wake Up Detection
    // xTaskCreate(wake_up_detection_task, "wake_up_task", 3072, NULL, 5, NULL);
    
    // Example 3: Enable Single Tap Detection
    // xTaskCreate(single_tap_detection_task, "single_tap_task", 3072, NULL, 5, NULL);
    
    // Example 4: Enable Double Tap Detection (don't enable with single tap)
    // xTaskCreate(double_tap_detection_task, "double_tap_task", 3072, NULL, 5, NULL);
    
    ESP_LOGI("EVENT_DETECTION", "Event detection tasks created");
}

/**
 * @brief Combined example: All events in one task
 * 
 * This demonstrates monitoring multiple events simultaneously.
 * Note: Tap detections are mutually exclusive - choose single OR double.
 */
static void combined_event_monitoring_task(void *pvParameters)
{
    ISM330DHCX_Event_Status_t event_status;
    
    ESP_LOGI("EVENTS", "Starting Combined Event Monitoring");
    
    // Enable multiple event detections
    ISM330DHCX_ACC_Enable_Free_Fall_Detection(&imu_obj, ISM330DHCX_INT1_PIN);
    ISM330DHCX_ACC_Enable_Wake_Up_Detection(&imu_obj, ISM330DHCX_INT2_PIN);  // Use INT2 for wake-up
    ISM330DHCX_ACC_Enable_Double_Tap_Detection(&imu_obj, ISM330DHCX_INT1_PIN);
    
    ESP_LOGI("EVENTS", "Monitoring: Free Fall, Wake Up, and Double Tap");
    
    while (1) {
        if (ISM330DHCX_ACC_Get_Event_Status(&imu_obj, &event_status) == ISM330DHCX_OK) {
            if (event_status.FreeFallStatus) {
                ESP_LOGW("EVENTS", ">>> FREE FALL!");
            }
            if (event_status.WakeUpStatus) {
                ESP_LOGW("EVENTS", ">>> WAKE UP!");
            }
            if (event_status.DoubleTapStatus) {
                ESP_LOGW("EVENTS", ">>> DOUBLE TAP!");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
