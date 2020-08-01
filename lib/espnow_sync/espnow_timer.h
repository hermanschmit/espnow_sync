#ifndef ESPNOW_TIMER_H
#define ESPNOW_TIMER_H

#include "driver/timer.h"

esp_err_t timer_set_delta_counter_value(timer_group_t group_num, timer_idx_t timer_num, int64_t delta_val);

#endif
