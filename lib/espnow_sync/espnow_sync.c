/* espnow_sync

Provides a capability to synchronize a timer via use of the ESPNOW p2p protocol.

*/

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp32/rom/ets_sys.h"
#include "espnow_sync.h"
#include "espnow_timer.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "soc/timer_group_struct.h"

static const char *TAG = "espnow_sync";

static xQueueHandle espnow_sync_queue;

static uint8_t example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t s_espnow_sync_seq[ESPNOW_SYNC_DATA_MAX] = {0, 0};

static int64_t diff_sum;
static int diff_count;
static uint16_t sync_iterations;

static void espnow_sync_deinit(espnow_sync_send_param_t *send_param);

static esp_err_t example_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* WiFi should start before using ESPNOW */
uint64_t espnow_sync_wifi_init(void)
{
    uint8_t mac[6];
    uint64_t ret_val;

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(example_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));

    /* Unclear whether this is necessary. From ESPNOW example.
     */
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0));
    ret_val = (mac[0]) | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24) |
        (mac[4] << 32) | (mac[5] << 40);
    return ret_val;
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_sync_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_sync_event_t evt;
    espnow_sync_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SYNC_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_sync_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_sync_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_sync_event_t evt;
    espnow_sync_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_SYNC_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &(recv_cb->ts));

    if (xQueueSend(espnow_sync_queue, &evt, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_sync_data_parse(uint8_t *data, uint16_t data_len,
                           uint8_t *state,
                           uint16_t *seq,
                           uint8_t *magic,
                           uint64_t *ts0_m)
{
    espnow_sync_data_t *buf = (espnow_sync_data_t *)data;

    if (data_len < sizeof(espnow_sync_data_t))
    {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    *ts0_m = buf->timestamp0_m;

    return buf->type;
}

/* Prepare ESPNOW data to be sent. */
void espnow_sync_data_prepare(espnow_sync_send_param_t *send_param)
{
    espnow_sync_data_t *buf = (espnow_sync_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_sync_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_SYNC_DATA_BROADCAST : ESPNOW_SYNC_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_sync_seq[buf->type]++;
    buf->magic = send_param->magic;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &(buf->timestamp0_m));
}

static void espnow_sync_task(void *pvParameter)
{
    espnow_sync_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint8_t recv_magic = 0;
    uint64_t recv_ts0_m = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(1000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    espnow_sync_send_param_t *send_param = (espnow_sync_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error: ESP_OK false on esp_now_send broadcast");
        espnow_sync_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(espnow_sync_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case ESPNOW_SYNC_SEND_CB:
        {
            espnow_sync_event_send_cb_t *send_cb = &evt.info.send_cb;
            is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

            ESP_LOGD(TAG, "Send data to " MACSTR ", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

            if (is_broadcast && (send_param->broadcast == false))
            {
                break;
            }

            if (!is_broadcast)
            {
                send_param->count--;
                if (send_param->count == 0)
                {
                    ESP_LOGI(TAG, "Send done");
                    espnow_sync_deinit(send_param);
                    vTaskDelete(NULL);
                }
            }

            /* Delay a while before sending the next data. */
            if (send_param->delay > 0)
            {
                vTaskDelay(send_param->delay / portTICK_RATE_MS);
            }

            ESP_LOGI(TAG, "send data to " MACSTR "", MAC2STR(send_cb->mac_addr));

            memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
            espnow_sync_data_prepare(send_param);

            /* Send the next data after the previous data is sent. */
            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
            {
                ESP_LOGE(TAG, "Send error");
                espnow_sync_deinit(send_param);
                vTaskDelete(NULL);
            }
            break;
        }
        case ESPNOW_SYNC_RECV_CB:
        {
            espnow_sync_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

            ret = espnow_sync_data_parse(recv_cb->data, recv_cb->data_len,
                                         &recv_state, &recv_seq, &recv_magic, &recv_ts0_m);
            uint64_t ts0_s_cb = recv_cb->ts;
            free(recv_cb->data);
            if (ret == ESPNOW_SYNC_DATA_BROADCAST)
            {

                ESP_LOGI(TAG, "Receive %dth broadcast data from: " MACSTR ", len: %d",
                         recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                ESP_LOGI(TAG, "Timestamp0_m: %llu  Timestamp0_s: %llu",
                         recv_ts0_m, ts0_s_cb);

                /* If MAC address does not exist in peer list, add it to peer list. */
                if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                {
                    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                    if (peer == NULL)
                    {
                        ESP_LOGE(TAG, "Malloc peer information fail");
                        espnow_sync_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                    memset(peer, 0, sizeof(esp_now_peer_info_t));
                    peer->channel = CONFIG_ESPNOW_CHANNEL;
                    peer->ifidx = ESPNOW_WIFI_IF;
                    peer->encrypt = true;
                    memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                    memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_ERROR_CHECK(esp_now_add_peer(peer));
                    free(peer);
                }

                /* Indicates that the device has received broadcast ESPNOW data. */
                if (send_param->state == 0)
                {
                    send_param->state = 1;
                }

                /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                if (recv_state == 1)
                {
                    /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                    if (send_param->unicast == false && send_param->magic >= recv_magic)
                    {
                        ESP_LOGI(TAG, "Start sending unicast data");
                        ESP_LOGI(TAG, "send data to " MACSTR "", MAC2STR(recv_cb->mac_addr));

                        /* Start sending unicast ESPNOW data. */
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        espnow_sync_data_prepare(send_param);
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Send error");
                            espnow_sync_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        else
                        {
                            send_param->broadcast = false;
                            send_param->unicast = true;
                        }
                    }
                }
            }
            else if (ret == ESPNOW_SYNC_DATA_UNICAST)
            {
                ESP_LOGI(TAG, "Receive %dth unicast data from: " MACSTR ", len: %d",
                         recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                int64_t d;
                d = recv_ts0_m - ts0_s_cb;

                ESP_LOGI(TAG, "Timestamp0_m: %llu  Timestamp0_s: %llu  Diff: %lld",
                         recv_ts0_m, ts0_s_cb, d);

                diff_sum += d;

                if (diff_count % sync_iterations == 0 &&
                    diff_count > 0)
                {
                    int64_t avg = diff_sum / sync_iterations;
                    ESP_LOGI(TAG, "Set Delta: %lld", avg);
                    timer_set_delta_counter_value(TIMER_GROUP_0, TIMER_0, avg + ESPNOW_WIFI_DELTA);
                    diff_sum = 0;
                }

                diff_count++;

                /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                send_param->broadcast = false;
            }
            else
            {
                ESP_LOGI(TAG, "Receive error data from: " MACSTR "", MAC2STR(recv_cb->mac_addr));
            }
            break;
        }
        default:
            ESP_LOGE(TAG, "Callback type error: %d", evt.id);
            break;
        }
    }
}

esp_err_t espnow_sync_exec(uint32_t magic, uint16_t count, uint16_t delay, uint16_t iterations)
{
    espnow_sync_send_param_t *send_param;

    diff_sum = (int64_t)0;
    sync_iterations = iterations;

    espnow_sync_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_sync_event_t));
    if (espnow_sync_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_sync_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_sync_recv_cb));

    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_sync_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_sync_send_param_t));
    memset(send_param, 0, sizeof(espnow_sync_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_sync_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = magic;
    send_param->count = count;
    send_param->delay = delay;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_sync_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_sync_data_prepare(send_param);

    xTaskCreate(espnow_sync_task, "espnow_sync_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_sync_deinit(espnow_sync_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_sync_queue);
    esp_now_deinit();
}
