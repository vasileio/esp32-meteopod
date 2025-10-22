/**
 * @file wifi.h
 * @brief Wi-Fi station mode initialization and connection management
 *
 * Provides Wi-Fi connectivity setup for ESP32 Meteopod. Configures the device
 * as a Wi-Fi station (client) and handles connection to the configured network.
 * Credentials are set via ESP-IDF's menuconfig system.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "nvs_flash.h"

/**
 * @brief Initialize Wi-Fi in station mode and connect to configured network
 *
 * Sets up Wi-Fi station mode, reads SSID and password from NVS configuration
 * (set via `idf.py menuconfig`), and attempts to connect to the network.
 * This function blocks until connection is established or fails.
 *
 * Wi-Fi credentials are configured via:
 * - `idf.py menuconfig` → Wi-Fi Configuration → Wi-Fi SSID/Password
 * - Or set CONFIG_WIFI_SSID and CONFIG_WIFI_PASS in sdkconfig
 *
 * @note This function initializes NVS flash and TCP/IP stack as prerequisites
 */
void wifi_init_sta(void);

#ifdef __cplusplus
}
#endif
