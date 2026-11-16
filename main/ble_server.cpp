

#include "ble_server.h"
#include "servo_controller.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_SERVER";
static const char *DEVICE_NAME = "ESP32S3-Arm";

#define BLE_CMD_MAX_LEN 512

// Nordic UART Service (NUS) UUIDs (Little-Endian)
static const ble_uuid128_t gatt_svr_svc_uart_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t gatt_svr_chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

static const ble_uuid128_t gatt_svr_chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle = 0;
static ble_cmd_callback_t s_cmd_handler = NULL;

static QueueHandle_t s_cmd_queue = NULL;
static TimerHandle_t s_rx_flush_timer = NULL;
static char s_rx_line_buf[BLE_CMD_MAX_LEN];
static size_t s_rx_line_len = 0;

static void ble_advertise(void);

static void ble_reply_fn(const char *reply) {
    if (reply != NULL && strlen(reply) > 0) {
        ble_server_send_notify(reply);
    }
}

static void dispatch_completed_line(void) {
    if (s_rx_line_len == 0) return;
    s_rx_line_buf[s_rx_line_len] = '\0';
    if (s_cmd_queue != NULL) {
        char msg[BLE_CMD_MAX_LEN];
        strncpy(msg, s_rx_line_buf, sizeof(msg) - 1);
        msg[sizeof(msg) - 1] = '\0';
        xQueueSend(s_cmd_queue, msg, 0);
    }
    s_rx_line_len = 0;
}

static void rx_flush_timer_cb(TimerHandle_t xTimer) {
    (void)xTimer;
    if (s_rx_line_len > 0) {
        dispatch_completed_line();
    }
}

static void ble_cmd_exec_task(void *pvParameters) {
    (void)pvParameters;
    char cmd_buf[BLE_CMD_MAX_LEN];

    while (1) {
        if (xQueueReceive(s_cmd_queue, cmd_buf, portMAX_DELAY) == pdTRUE) {
            if (s_cmd_handler != NULL && strlen(cmd_buf) > 0) {
                s_cmd_handler(cmd_buf, ble_reply_fn);
            }
        }
    }
}

static int gatt_svr_chr_access_rx(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0) return 0;

        char chunk[BLE_CMD_MAX_LEN];
        uint16_t read_len = (om_len >= sizeof(chunk)) ? (sizeof(chunk) - 1) : om_len;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, chunk, read_len, NULL);
        if (rc == 0) {
            if (s_rx_flush_timer != NULL) {
                xTimerStop(s_rx_flush_timer, 0);
            }

            for (uint16_t i = 0; i < read_len; i++) {
                char ch = chunk[i];
                if (ch == '\r' || ch == '\n') {
                    dispatch_completed_line();
                } else if ((uint8_t)ch >= 0x20 && (uint8_t)ch <= 0x7E) {
                    if (s_rx_line_len < sizeof(s_rx_line_buf) - 1) {
                        s_rx_line_buf[s_rx_line_len++] = ch;
                    }
                }
            }

            // If packet didn't end with '\n', flush shortly unless another chunk follows
            if (s_rx_line_len > 0 && s_rx_flush_timer != NULL) {
                xTimerStart(s_rx_flush_timer, 0);
            }
        }
        return 0;
    }
    return 0;
}

static int gatt_svr_chr_access_tx(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int s1 = 90, s2 = 90, s3 = 90;
        servo_get_all_angles(&s1, &s2, &s3);
        char buf[64];
        snprintf(buf, sizeof(buf), "ANG:%d,%d,%d\n", s1, s2, s3);
        int rc = os_mbuf_append(ctxt->om, buf, strlen(buf));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

static const struct ble_gatt_chr_def gatt_svr_chrs[] = {
    {
        .uuid = &gatt_svr_chr_rx_uuid.u,
        .access_cb = gatt_svr_chr_access_rx,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    },
    {
        .uuid = &gatt_svr_chr_tx_uuid.u,
        .access_cb = gatt_svr_chr_access_tx,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = &s_tx_val_handle,
        .cpfd = NULL,
    },
    {
        .uuid = NULL,
        .access_cb = NULL,
        .arg = NULL,
        .descriptors = NULL,
        .flags = 0,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    }
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uart_uuid.u,
        .includes = NULL,
        .characteristics = gatt_svr_chrs,
    },
    {
        .type = 0,
        .uuid = NULL,
        .includes = NULL,
        .characteristics = NULL,
    }
};

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connection %s (status=%d)",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_rx_line_len = 0;
        } else {
            ble_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnected (reason=%d)", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_rx_line_len = 0;
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "BLE MTU updated: %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void ble_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: rc=%d", rc);
        return;
    }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t *)&gatt_svr_svc_uart_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response fields: rc=%d", rc);
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising BLE service as '%s'...", DEVICE_NAME);
    }
}

static void ble_host_task(void *param) {
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_on_sync(void) {
    ESP_LOGI(TAG, "NimBLE host synchronized");
    ble_advertise();
}

static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "NimBLE host reset (reason=%d)", reason);
}

void ble_server_init(ble_cmd_callback_t cmd_handler) {
    s_cmd_handler = cmd_handler;
    s_cmd_queue = xQueueCreate(8, BLE_CMD_MAX_LEN);
    s_rx_flush_timer = xTimerCreate("ble_rx_flush", pdMS_TO_TICKS(60), pdFALSE, NULL, rx_flush_timer_cb);
    xTaskCreatePinnedToCore(ble_cmd_exec_task, "ble_cmd_task", 4096, NULL, 5, NULL, 1);

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE port: %d", ret);
        return;
    }

    ble_att_set_preferred_mtu(247);
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "BLE Server ready ('%s' Nordic UART Service)", DEVICE_NAME);
}

bool ble_server_is_connected(void) {
    return (s_conn_handle != BLE_HS_CONN_HANDLE_NONE);
}

void ble_server_send_notify(const char *message) {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_tx_val_handle == 0 || message == NULL) {
        return;
    }

    size_t len = strlen(message);
    if (len == 0) return;

    // Split long payloads safely into chunks so they fit even on smaller MTUs
    const size_t CHUNK_MAX = 160;
    size_t offset = 0;
    while (offset < len) {
        size_t chunk_len = len - offset;
        if (chunk_len > CHUNK_MAX) chunk_len = CHUNK_MAX;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(message + offset, chunk_len);
        if (om != NULL) {
            ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
        }
        offset += chunk_len;
        if (offset < len) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
