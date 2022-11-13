#ifndef __SYSLOG_H__
#define __SYSLOG_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "usart.h"

#define SYS_LOG_EMERG   0 /* system is unusable */
#define SYS_LOG_ALERT   1 /* action must be taken immediately */
#define SYS_LOG_CRIT    2 /* critical conditions */
#define SYS_LOG_ERR     3 /* error conditions */
#define SYS_LOG_WARNING 4 /* warning conditions */
#define SYS_LOG_NOTICE  5 /* normal but significant condition */
#define SYS_LOG_INFO    6 /* informational */
#define SYS_LOG_DEBUG   7 /* debug-level messages */

#define LOG_LEVEL_ERROR   "ERR:"
#define LOG_LEVEL_WARNING "WRN:"
#define LOG_LEVEL_INFO    "INF:"
#define LOG_LEVEL_DEBUG   "DBG:"

#ifndef TAG
#define TAG "app"
#endif

#ifndef LOG_TAG
#define LOG_TAG "[" TAG "]"
#endif

#ifndef CONFIG_LOG_LEVEL
#define CONFIG_LOG_LEVEL SYS_LOG_WARNING
//#define CONFIG_LOG_LEVEL SYS_LOG_CLOSE
#endif

#define LOG(level, fmt, arg...) UsartPrintf("%s:%s <%s:%u>: " fmt "", level, LOG_TAG, __FUNCTION__, __LINE__, ##arg)

//#define LOG_ERR(fmt, arg...) LOG(LOG_LEVEL_ERROR, fmt, ##arg)

#if CONFIG_LOG_LEVEL == SYS_LOG_CLOSE

#define LOG_ERR(fmt, arg...)
#define LOG_WAR(fmt, arg...)
#define LOG_INF(fmt, arg...)
#define LOG_DBG(fmt, arg...)

#elif CONFIG_LOG_LEVEL == SYS_LOG_DEBUG

#define LOG_ERR(fmt, arg...)  LOG(LOG_LEVEL_ERROR, fmt, ##arg)
#define LOG_WAR(fmt, arg...) LOG(LOG_LEVEL_WARNING, fmt, ##arg)
#define LOG_INF(fmt, arg...) LOG(SYS_LOG_INFO, fmt, ##arg)
#define LOG_DBG(fmt, arg...)  LOG(LOG_LEVEL_DEBUG, fmt, ##arg)

#elif CONFIG_LOG_LEVEL == SYS_LOG_INFO

#define LOG_ERR(fmt, arg...)  LOG(LOG_LEVEL_ERROR, fmt, ##arg)
#define LOG_WAR(fmt, arg...) LOG(LOG_LEVEL_WARNING, fmt, ##arg)
#define LOG_INF(fmt, arg...) LOG(SYS_LOG_INFO, fmt, ##arg)
#define LOG_DBG(fmt, arg...)

#elif CONFIG_LOG_LEVEL == SYS_LOG_WARNING

#define LOG_ERR(fmt, arg...)  LOG(LOG_LEVEL_ERROR, fmt, ##arg)
#define LOG_WAR(fmt, arg...) LOG(LOG_LEVEL_WARNING, fmt, ##arg)
#define LOG_INF(fmt, arg...)
#define LOG_DBG(fmt, arg...)

#elif CONFIG_LOG_LEVEL == SYS_LOG_ERROR

#define LOG_ERR(fmt, arg...) LOG(LOG_LEVEL_ERROR, fmt, ##arg)
#define LOG_WAR(fmt, arg...)
#define LOG_INF(fmt, arg...)
#define LOG_DBG(fmt, arg...)

#endif

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
