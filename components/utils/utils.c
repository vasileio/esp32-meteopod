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

const char *now_iso8601(void)
{
    static char buf[21];  // "YYYY-MM-DDThh:mm:ssZ\0"
    struct timeval tv;
    struct tm tm;

    if (gettimeofday(&tv, NULL) != 0 ||
        gmtime_r(&tv.tv_sec, &tm) == NULL) {
        // fallback to all-zeros
        strcpy(buf, "0000-00-00T00:00:00Z");
        return buf;
    }

    // format: YYYY-MM-DDThh:mm:ssZ
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}