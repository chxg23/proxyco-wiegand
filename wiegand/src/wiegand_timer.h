#ifndef WIEGAND_TIMER_H
#define WIEGAND_TIMER_H

/**
 * Wait until 'usecs' microseconds has elapsed. This is a blocking delay.
 *
 * @param usecs The number of usecs to wait.
 */
void wiegand_timer_delay_usecs(uint32_t usecs);


/**
 * Sets a high resolution timer that will expire 'usecs' microseconds from the current
 * cputime.
 *
 * NOTE: This must be called when the timer is stopped.
 *
 * @param usecs The number of usecs from now at which the timer will expire.
 *
 * @return int 0 on success; EINVAL if timer already started or timer struct
 *         invalid
 */
int wiegand_timer_relative(uint32_t usecs);

/**
 * Initialize the Wiegand timer module. This must be called after os_init is called
 * and before any other timer API are used. This should be called only once
 * and should be called before the hardware timer is used.
 *
 * @param clock_freq The desired cputime frequency, in hertz (Hz).
 *
 * @return int 0 on success; -1 on error.
 */
int wiegand_timer_init(uint32_t clock_freq,hal_timer_cb fp );

#endif // WIEGAND_TIMER_H