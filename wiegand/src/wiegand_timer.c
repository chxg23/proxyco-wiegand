/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>
#include <stdint.h>
#include <assert.h>
#include "os/mynewt.h"

static uint32_t ticks_per_usec;    /* number of ticks per usec */
static struct hal_timer g_wiegand_timer;

static uint32_t
timer_get32(void)
{
    uint32_t time_cnt;

    time_cnt = hal_timer_read(MYNEWT_VAL(WIEGAND_NRF_TIMER));
    return time_cnt;
}

static void
timer_init(struct hal_timer *timer, hal_timer_cb fp, void *arg)
{
    assert(timer != NULL);
    assert(fp != NULL);

    hal_timer_set_cb(MYNEWT_VAL(WIEGAND_NRF_TIMER), timer, fp, arg);
}
/**
 * Wait until the number of ticks has elapsed. This is a blocking delay.
 *
 * @param ticks The number of ticks to wait.
 */
static void
delay_ticks(uint32_t ticks)
{
    uint32_t until;

    until = timer_get32() + ticks;
    while ((int32_t)(timer_get32() - until) < 0) {
        /* Loop here till finished */
    }
}


int
wiegand_timer_relative(uint32_t usecs)
{
    int rc;
    uint32_t time_cnt;

    time_cnt = timer_get32() + usecs * ticks_per_usec;
    rc = hal_timer_start_at(&g_wiegand_timer, time_cnt);

    return rc;
}

void
wiegand_timer_delay_usecs(uint32_t usecs)
{
    uint32_t ticks;

    ticks = (usecs * 1000 * ticks_per_usec + 999) / 1000;

    delay_ticks(ticks);
}

int
wiegand_timer_init(uint32_t clock_freq,hal_timer_cb fp )
{
    int rc;

    ticks_per_usec = clock_freq / 1000000U;

    rc = hal_timer_config(MYNEWT_VAL(WIEGAND_NRF_TIMER), clock_freq);
    if (rc) {
        return rc;
    }

    timer_init(&g_wiegand_timer, fp, NULL);

    return hal_timer_start(&g_wiegand_timer, 0);
}
