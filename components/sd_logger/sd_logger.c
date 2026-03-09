#include "sd_logger.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "sd_log";

static bool sd_mounted = false;
static sdmmc_card_t *sd_card = NULL;

// Current session state
static FILE *session_file = NULL;
static char session_filename[128] = {0};
static uint32_t session_start_tick = 0;
static uint32_t session_msg_count = 0;

// Ring buffer for non-blocking writes
typedef struct {
    uint32_t timestamp_ms;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} can_log_entry_t;

#define LOG_QUEUE_SIZE 256
static QueueHandle_t log_queue = NULL;
static TaskHandle_t writer_task_handle = NULL;

static void sd_writer_task(void *arg);

esp_err_t sd_logger_init(void)
{
    // Initialize SPI bus for SD card
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_MOSI,
        .miso_io_num = SD_PIN_MISO,
        .sclk_io_num = SD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    // Use DMA channel 2 (channel 1 is used by the display on SPI2)
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Mount SD card
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_PIN_CS;
    slot_cfg.host_id = SPI3_HOST;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &sd_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card mount failed: %s (no card?)", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        return ret;
    }

    sd_mounted = true;
    sdmmc_card_print_info(stdout, sd_card);
    ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);

    // Create log queue and writer task
    log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(can_log_entry_t));
    xTaskCreatePinnedToCore(sd_writer_task, "sd_writer", 4096, NULL, 2, &writer_task_handle, 0);

    return ESP_OK;
}

bool sd_logger_is_mounted(void)
{
    return sd_mounted;
}

void sd_logger_start_session(void)
{
    if (!sd_mounted || session_file) return;

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    if (t->tm_year > (2024 - 1900)) {
        // Time is synced — use date/time filename
        snprintf(session_filename, sizeof(session_filename),
                 SD_MOUNT_POINT "/%04d-%02d-%02d_%02d%02d%02d.csv",
                 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
    } else {
        // No time sync — use boot tick
        snprintf(session_filename, sizeof(session_filename),
                 SD_MOUNT_POINT "/session_%lu.csv",
                 (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ));
    }

    session_file = fopen(session_filename, "w");
    if (!session_file) {
        ESP_LOGE(TAG, "Failed to create %s", session_filename);
        return;
    }

    // Write CSV header
    fprintf(session_file, "timestamp_ms,can_id,dlc,d0,d1,d2,d3,d4,d5,d6,d7\n");
    fflush(session_file);

    session_start_tick = xTaskGetTickCount();
    session_msg_count = 0;
    ESP_LOGI(TAG, "Logging session started: %s", session_filename);
}

void sd_logger_write(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (!sd_mounted || !log_queue) return;

    can_log_entry_t entry;
    entry.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    entry.can_id = can_id;
    entry.dlc = dlc > 8 ? 8 : dlc;
    memcpy(entry.data, data, entry.dlc);
    if (entry.dlc < 8) memset(entry.data + entry.dlc, 0, 8 - entry.dlc);

    // Non-blocking push — drop message if queue full
    xQueueSend(log_queue, &entry, 0);
}

void sd_logger_end_session(void)
{
    if (!session_file) return;

    fflush(session_file);
    fclose(session_file);
    session_file = NULL;

    uint32_t duration_sec = (xTaskGetTickCount() - session_start_tick) / configTICK_RATE_HZ;

    if (duration_sec < MIN_SESSION_SECONDS) {
        // Delete short sessions
        unlink(session_filename);
        ESP_LOGI(TAG, "Session too short (%lus < %ds), deleted %s",
                 (unsigned long)duration_sec, MIN_SESSION_SECONDS, session_filename);
    } else {
        ESP_LOGI(TAG, "Session saved: %s (%lu msgs, %lus)",
                 session_filename, (unsigned long)session_msg_count,
                 (unsigned long)duration_sec);
    }

    session_filename[0] = 0;
    session_msg_count = 0;
}

static void sd_writer_task(void *arg)
{
    can_log_entry_t entry;
    int flush_counter = 0;

    while (1) {
        if (xQueueReceive(log_queue, &entry, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (session_file) {
                fprintf(session_file, "%lu,0x%03lX,%d,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
                        (unsigned long)entry.timestamp_ms,
                        (unsigned long)entry.can_id, entry.dlc,
                        entry.data[0], entry.data[1], entry.data[2], entry.data[3],
                        entry.data[4], entry.data[5], entry.data[6], entry.data[7]);
                session_msg_count++;
                flush_counter++;

                // Flush every 100 messages or ~5 seconds worth
                if (flush_counter >= 100) {
                    fflush(session_file);
                    flush_counter = 0;
                }
            }
        } else {
            // Timeout — flush if we have pending data
            if (session_file && flush_counter > 0) {
                fflush(session_file);
                flush_counter = 0;
            }
        }
    }
}

int sd_logger_list_files(sd_file_info_t *files, int max_files)
{
    if (!sd_mounted || !files || max_files <= 0) return 0;

    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) return 0;

    int count = 0;
    struct dirent *de;
    while ((de = readdir(dir)) != NULL && count < max_files) {
        // Only list .csv files
        size_t nlen = strlen(de->d_name);
        if (nlen < 5) continue;
        if (strcmp(de->d_name + nlen - 4, ".csv") != 0) continue;

        strncpy(files[count].name, de->d_name, sizeof(files[count].name) - 1);
        files[count].name[sizeof(files[count].name) - 1] = 0;

        char fullpath[320];
        snprintf(fullpath, sizeof(fullpath), SD_MOUNT_POINT "/%s", de->d_name);
        struct stat st;
        if (stat(fullpath, &st) == 0) {
            files[count].size_bytes = st.st_size;
        } else {
            files[count].size_bytes = 0;
        }
        count++;
    }
    closedir(dir);

    // Sort by name descending (newest first, since names contain dates)
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (strcmp(files[i].name, files[j].name) < 0) {
                sd_file_info_t tmp = files[i];
                files[i] = files[j];
                files[j] = tmp;
            }
        }
    }

    return count;
}
