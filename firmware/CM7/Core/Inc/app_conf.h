/*
 * app_conf.h
 *
 *  Created on: 11 paź 2026
 *      Author: Oleg
 */

#ifndef INC_APP_CONF_H_
#define INC_APP_CONF_H_

#ifndef APP_AGENT_IP
#define APP_AGENT_IP "192.168.50.2"
#endif
#ifndef APP_AGENT_PORT
#define APP_AGENT_PORT 8888U
#endif
#ifndef APP_LOCAL_UDP_PORT
#define APP_LOCAL_UDP_PORT 8888U
#endif
#ifndef APP_UDP_MAX_PACKET
#define APP_UDP_MAX_PACKET 1536U
#endif

/* The uploaded freertos.c had ILI9341 disabled. Enable only after verifying
 * its CS pin and that its driver completes each SPI transfer before returning. */
#ifndef APP_ENABLE_ILI9341
#define APP_ENABLE_ILI9341 0
#endif

#ifndef APP_CMD_TIMEOUT_MS
#define APP_CMD_TIMEOUT_MS 500U
#endif
#define APP_CMD_WATCHDOG_MS 10U
/* Set actual robot limits in m/s and rad/s. Zero disables that limit. */
#ifndef APP_MAX_LINEAR_SPEED
#define APP_MAX_LINEAR_SPEED 0.0
#endif
#ifndef APP_MAX_ANGULAR_SPEED
#define APP_MAX_ANGULAR_SPEED 0.0
#endif

#define APP_DISPLAY_PERIOD_MS 500U
#define APP_OLED_SPI_TIMEOUT_MS 100U
#define APP_TEMPERATURE_PERIOD_MS 1000U
#define APP_HUMIDITY_PERIOD_MS 2000U
#define APP_IMU_PERIOD_MS 10U
#define APP_ODOM_PERIOD_MS 50U
#define APP_AGENT_PING_PERIOD_MS 1000U
#define APP_AGENT_RETRY_MS 1000U
#define APP_AGENT_PING_TIMEOUT_MS 50U
#define APP_TIME_SYNC_PERIOD_MS 10000U
#define APP_TIME_SYNC_TIMEOUT_MS 100U
#define APP_RELIABLE_ACK_TIMEOUT_MS 20U

#endif /* INC_APP_CONF_H_ */
