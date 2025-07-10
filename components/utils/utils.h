#ifndef UTILS_H
#define UTILS_H

#include <esp_err.h>
#include <stdio.h>
#include <stddef.h>
#include <time.h>
#include <sys/time.h>  // gettimeofday()
#include <string.h>

/**
 * @brief Get the current UTC time as an ISO-8601 string.
 *
 * Returns a pointer to a thread-safe, statically-allocated buffer
 * of the form "YYYY-MM-DDThh:mm:ssZ". You must have an RTC or SNTP
 * synchronized before calling this, otherwise it will all be zeros.
 *
 * @return Pointer to an NUL-terminated ISO-8601 timestamp.
 */
const char *now_iso8601(void);

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
