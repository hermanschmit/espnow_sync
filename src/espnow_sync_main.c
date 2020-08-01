/* ESPNOW Sync Main Example

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
#include <espnow_sync.h>
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "soc/timer_group_struct.h"

#define TIMER_INTR_SEL TIMER_INTR_LEVEL                                 /*!< Timer level interrupt */
#define TIMER_GROUP TIMER_GROUP_0                                       /*!< Test on timer group 0 */
#define TIMER_DIVIDER 80                                                /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)                    /*!< used to calculate counter value */
#define TIMER_FINE_ADJ (0 * (TIMER_BASE_CLK / TIMER_DIVIDER) / 1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC (0.5)                                       /*!< test interval for timer 0 */

// TODO: seems like this needs to be parameterized by board.
#define BLINK_GPIO 5

volatile int cnt = 0;

static const char *TAG = "espnow_sync_main";

void IRAM_ATTR timer_group0_isr(void *para)
{ // timer group 0, ISR
  int timer_idx = (int)para;
  uint32_t intr_status = TIMERG0.int_st_timers.val;
  TIMERG0.hw_timer[timer_idx].update = 1;
  uint64_t timer_counter_value =
      ((uint64_t)TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;

  // Round down to last alarm setting (could do this by storing)
  timer_counter_value = timer_counter_value / (uint64_t)(TIMER_INTERVAL0_SEC * TIMER_SCALE);
  timer_counter_value *= (uint64_t)(TIMER_INTERVAL0_SEC * TIMER_SCALE);

  if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0)
  {
    /*
      TIMERG0.hw_timer[timer_idx].update = 1;
      TIMERG0.int_clr_timers.t0 = 1;
      TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
    */
    TIMERG0.int_clr_timers.t0 = 1;
    timer_counter_value += (uint64_t)(TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ;
    TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t)(timer_counter_value >> 32);
    TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t)timer_counter_value;
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    gpio_set_level(BLINK_GPIO, cnt % 2);
    cnt++;
  }
}

static void example_tg0_init(int timer_idx)
{
  int timer_group = TIMER_GROUP_0;
  timer_config_t config;
  config.alarm_en = 1;
  config.auto_reload = 0;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = TIMER_DIVIDER;
  config.intr_type = TIMER_INTR_SEL;
  config.counter_en = TIMER_PAUSE;
  ESP_LOGI(TAG, "Start Timer Init");
  /*Configure timer*/
  timer_init(timer_group, timer_idx, &config);
  ESP_LOGI(TAG, "Stop Timer 0");
  /*Stop timer counter*/
  timer_pause(timer_group, timer_idx);
  /*Load counter value */
  ESP_LOGI(TAG, "Timer Set");
  timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
  /*Set alarm value*/
  ESP_LOGI(TAG, "Timer Alarm Value");
  timer_set_alarm_value(timer_group, timer_idx,
                        (TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
  /*Enable timer interrupt*/
  ESP_LOGI(TAG, "Timer Enable Intr");
  timer_enable_intr(timer_group, timer_idx);
  /*Set ISR handler*/
  ESP_LOGI(TAG, "Timer ISR register");
  timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void *)timer_idx, ESP_INTR_FLAG_IRAM, NULL);
  /*Start timer counter*/
  ESP_LOGI(TAG, "Timer start");
  timer_start(timer_group, timer_idx);
}

void app_main()
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Might want to use something like mac address here.
  // int magic = 1;

  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

  int timer_idx = TIMER_0;

  // TODO: not sure this works for anything other than TIMER_GROUP_0
  example_tg0_init(timer_idx);

  uint64_t mac = espnow_sync_wifi_init();
  int mac_trunc = (int) mac;
  ESP_LOGI(TAG, "Mac Addr %d", mac_trunc);
  espnow_sync_exec(mac_trunc, 100, 1000, 10);
}