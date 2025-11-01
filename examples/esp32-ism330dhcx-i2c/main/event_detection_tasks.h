/*
 * ISM330DHCX Event Detection Tasks Header
 *
 * Function declarations and documentation for event detection tasks
 */

#ifndef EVENT_DETECTION_TASKS_H
#define EVENT_DETECTION_TASKS_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Create all event detection tasks
     *
     * This function creates FreeRTOS tasks for various ISM330DHCX event detections:
     * - Free Fall Detection
     * - Wake Up Detection
     * - Single Tap Detection
     * - Double Tap Detection
     *
     * Uncomment the specific tasks you want to enable in the implementation.
     *
     * @note Must be called after IMU initialization
     * @note Only enable ONE tap detection at a time (single OR double, not both)
     */
    void create_event_detection_tasks(void);

#ifdef __cplusplus
}
#endif

#endif // EVENT_DETECTION_TASKS_H
