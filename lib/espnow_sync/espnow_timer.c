/*

espnow_timer

Keeping this separate. Maybe it can be incorporated into ESP-IDF.

*/

#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "espnow_timer.h"

static timg_dev_t *TG[2] = {&TIMERG0, &TIMERG1};
static portMUX_TYPE timer_spinlock[TIMER_GROUP_MAX] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
#define TIMER_ENTER_CRITICAL(mux) portENTER_CRITICAL(mux);
#define TIMER_EXIT_CRITICAL(mux) portEXIT_CRITICAL(mux);

esp_err_t timer_set_delta_counter_value(timer_group_t group_num, timer_idx_t timer_num,
                                        int64_t delta_val)
{
  uint64_t timer_val;
  TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
  TG[group_num]->hw_timer[timer_num].update = 1;
  timer_val = ((uint64_t)TG[group_num]->hw_timer[timer_num].cnt_high << 32) | (TG[group_num]->hw_timer[timer_num].cnt_low);
  timer_val = (uint64_t)(timer_val + delta_val);
  TG[group_num]->hw_timer[timer_num].load_high = (uint32_t)(timer_val >> 32);
  TG[group_num]->hw_timer[timer_num].load_low = (uint32_t)timer_val;
  TG[group_num]->hw_timer[timer_num].reload = 1;
  TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
  return ESP_OK;
}
