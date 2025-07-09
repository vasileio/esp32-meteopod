/**
 * @file task_config.h
 * @brief Configuration constants for FreeRTOS task priorities and stack sizes.
 *
 * This header defines task priorities relative to tskIDLE_PRIORITY
 * and stack sizes (in words) based on configMINIMAL_STACK_SIZE.
 */
#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Task priorities, relative to tskIDLE_PRIORITY and < configMAX_PRIORITIES.
 *
 * - PRIO_BLINK:    Low priority for simple LED toggle operations.
 * - PRIO_MONITOR:  Medium priority for system metrics and logging.
 * - PRIO_SENSORS:  Higher priority for periodic sensor sampling.
 * - PRIO_MQTT:     High priority for network/TLS communication.
 * - PRIO_WATCHDOG: Highest priority for fault detection and recovery.
 */
enum {
    PRIO_BLINK    = tskIDLE_PRIORITY + 1,  // = 1
    PRIO_MONITOR  = tskIDLE_PRIORITY + 1,  // = 1
    PRIO_SENSORS  = tskIDLE_PRIORITY + 5,  // = 5
    PRIO_MQTT     = tskIDLE_PRIORITY + 5,  // = 5
    PRIO_WATCHDOG = configMAX_PRIORITIES - 1  /* Reserved for watchdog */
};

/**
 * @brief Stack sizes (in words) for each task, starting from minimal.
 *
 * - STACK_BLINK:     Minimal stack + LED toggle overhead.
 * - STACK_MONITOR:   Extra headroom for logging and metrics.
 * - STACK_SENSORS:   Modest stack for I2C driver and logging.
 * - STACK_MQTT:      Larger stack for TCP/TLS operations.
 * - STACK_WATCHDOG:  Small stack for quick checks and resets.
 */
#define STACK_WATCHDOG_WORDS  (configMINIMAL_STACK_SIZE + 640)   // = 768 bytes
#define STACK_BLINK_WORDS     (configMINIMAL_STACK_SIZE + 384)   // = 512 bytes  
#define STACK_MONITOR_WORDS   (configMINIMAL_STACK_SIZE + 1024)  // = 1152 bytes
#define STACK_MQTT_WORDS      (configMINIMAL_STACK_SIZE + 3968)  // = 4096 bytes (keep as-is)
#define STACK_SENSORS_WORDS   (configMINIMAL_STACK_SIZE + 1408)  // = 1536 bytes

#endif /* TASK_CONFIG_H */
