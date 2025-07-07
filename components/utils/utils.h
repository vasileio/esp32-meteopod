#ifndef UTILS_H
#define UTILS_H

#include <esp_err.h>
#include <stdio.h>
#include <stddef.h>

/**
 * @brief Build a full MQTT topic from a prefix and suffix.
 *
 * Concatenates `prefix` and `suffix` with a slash in between,
 * writing the result into `buf`.
 *
 * @param[in]  prefix   Topic prefix (e.g. "meteopod/ABCDEF012345"), without trailing slash.
 * @param[in]  suffix   Topic suffix (e.g. "availability"), without leading slash.
 * @param[out] buf      Buffer to receive the full topic string.
 * @param[in]  buf_len  Length of `buf` in bytes.
 * @return
 *   - ESP_OK on success (string fits in buffer, NUL‐terminated).  
 *   - ESP_ERR_INVALID_ARG if any pointer is NULL or buf_len is 0.  
 *   - ESP_ERR_INVALID_SIZE if the formatted string didn’t fit.
 */
esp_err_t utils_build_topic(const char *prefix, const char *suffix, char *buf, size_t buf_len);

#endif // UTILS_H
