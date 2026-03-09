#include "ble_time_sync.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_time";

static bool time_synced = false;
static uint32_t sync_timestamp = 0; // boot ticks when synced

// Custom service UUID: 12345678-1234-1234-1234-123456789abc
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// Time write characteristic UUID: 12345678-1234-1234-1234-123456789abd
static const ble_uuid128_t chr_uuid =
    BLE_UUID128_INIT(0xbd, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static int time_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

    if (len == 4) {
        // 4 bytes: Unix timestamp (uint32_t, little-endian)
        uint32_t unix_ts = 0;
        os_mbuf_copydata(ctxt->om, 0, 4, &unix_ts);

        struct timeval tv = { .tv_sec = unix_ts, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        time_synced = true;
        sync_timestamp = xTaskGetTickCount();

        struct tm *t = localtime(&tv.tv_sec);
        ESP_LOGI(TAG, "Time synced: %04d-%02d-%02d %02d:%02d:%02d",
                 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
    } else if (len == 7) {
        // 7 bytes: year(2) month(1) day(1) hour(1) min(1) sec(1)
        uint8_t buf[7];
        os_mbuf_copydata(ctxt->om, 0, 7, buf);

        struct tm t = {0};
        t.tm_year = (buf[0] | (buf[1] << 8)) - 1900;
        t.tm_mon = buf[2] - 1;
        t.tm_mday = buf[3];
        t.tm_hour = buf[4];
        t.tm_min = buf[5];
        t.tm_sec = buf[6];

        time_t unix_ts = mktime(&t);
        struct timeval tv = { .tv_sec = unix_ts, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        time_synced = true;
        sync_timestamp = xTaskGetTickCount();

        ESP_LOGI(TAG, "Time synced: %04d-%02d-%02d %02d:%02d:%02d",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        ESP_LOGW(TAG, "Invalid time data length: %d (expected 4 or 7)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &chr_uuid.u,
                .access_cb = time_write_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 } // terminator
        },
    },
    { 0 } // terminator
};

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"MercedesCAN";
    fields.name_len = strlen("MercedesCAN");
    fields.name_is_complete = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    ble_gap_adv_set_fields(&fields);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, NULL, NULL);
    ESP_LOGI(TAG, "BLE advertising started: MercedesCAN");
}

static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, NULL);
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset, reason=%d", reason);
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_time_sync_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_svc_gap_device_name_set("MercedesCAN");

    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE time sync initialized");
}

bool ble_time_sync_is_synced(void)
{
    return time_synced;
}

uint32_t ble_time_sync_age(void)
{
    if (!time_synced) return 0;
    return (xTaskGetTickCount() - sync_timestamp) / configTICK_RATE_HZ;
}
