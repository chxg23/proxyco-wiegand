#ifndef WIEGAND_LOG_H
#define WIEGAND_LOG_H

#include <modlog/modlog.h>

#define WIEGAND_LOG(lvl_, ...) MODLOG_ ## lvl_(MYNEWT_VAL(WIEGAND_LOG_MODULE), __VA_ARGS__)

#endif /* WIEGAND_LOG_H */
