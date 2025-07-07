#include "utils.h"

esp_err_t utils_build_topic(const char *prefix, const char *suffix, char *buf, size_t buf_len)
{
    if (!prefix || !suffix || !buf || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Format "prefix/suffix" */
    int len = snprintf(buf, buf_len, "%s/%s", prefix, suffix);

    /* Check for encoding errors or truncation */
    if (len < 0 || (size_t)len >= buf_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}
