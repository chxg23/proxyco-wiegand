#ifndef WIEGAND_LOG_H
#define WIEGAND_LOG_H

#include <log/log.h>

extern struct log wiegand_log;

#define WIEGAND_LOG_MODULE  (LOG_MODULE_PERUSER + 6)

/* Convenience macro for logging from the app. */
#define WIEGAND_LOG(lvl, ...) \
    LOG_ ## lvl(&wiegand_log, WIEGAND_LOG_MODULE, __VA_ARGS__)

#endif /* WIEGAND_LOG_H */
