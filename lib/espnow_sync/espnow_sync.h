/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_SYNC_H
#define ESPNOW_SYNC_H

// In the past, these have been configured by sdkconfig.

#define CONFIG_ESPNOW_PMK "pmk1234567890123"
#define CONFIG_ESPNOW_LMK "lmk1234567890123"
#define CONFIG_ESPNOW_CHANNEL 1
#define CONFIG_ESPNOW_SEND_LEN 200
#define CONFIG_ESPNOW_BLINK_GPIO 5
/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE 6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

// This is the empircally measured delta from master to slave for communication latency
#define ESPNOW_WIFI_DELTA 3500
// Time (in seconds) to wait before giving up on sync after call to espnow_sync_await_completion
#define ESPNOW_SYNC_AWAIT_LIMIT 120

#define ESPNOW_SYNC_TASK_NAME "espnow_sync_task"

typedef enum
{
    ESPNOW_SYNC_SEND_CB,
    ESPNOW_SYNC_RECV_CB,
} espnow_sync_event_id_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_sync_event_send_cb_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
    uint64_t ts;
} espnow_sync_event_recv_cb_t;

typedef union {
    espnow_sync_event_send_cb_t send_cb;
    espnow_sync_event_recv_cb_t recv_cb;
} espnow_sync_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct
{
    espnow_sync_event_id_t id;
    espnow_sync_event_info_t info;
} espnow_sync_event_t;

enum
{
    ESPNOW_SYNC_DATA_BROADCAST,
    ESPNOW_SYNC_DATA_UNICAST,
    ESPNOW_SYNC_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct
{
    uint8_t type;          //Broadcast or unicast ESPNOW data.
    uint8_t state;         //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;      //Sequence number of ESPNOW data.
    uint8_t magic;         //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint64_t timestamp0_m; //Master timestamp 0
    uint64_t timestamp0_s; //Slave  timestamp 0
    uint64_t timestamp1_m; //Master timestamp 1
} __attribute__((packed)) espnow_sync_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct
{
    bool unicast;                       //Send unicast ESPNOW data.
    bool broadcast;                     //Send broadcast ESPNOW data.
    uint8_t state;                      //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                      //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                     //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                     //Delay between sending two ESPNOW data, unit: ms.
    int len;                            //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                    //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN]; //MAC address of destination device.
} espnow_sync_send_param_t;


uint64_t espnow_sync_wifi_init(void);
esp_err_t espnow_sync_exec(uint32_t, uint16_t, uint16_t, uint16_t);
esp_err_t espnow_sync_await_completion();

#endif
