/*
 *  drv_wifi.h
 *  Subsystem 1 -- WiFi (CYW43) + MQTT (lwIP) driver interface (kernel-free).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  <stdbool.h> and <stdint.h> only, so both drv_wifi.c (SDK side) and the
 *  Telemetry task (kernel side) include it (rule R1).
 *
 *  Built only with TEL_BACKEND_WIFI=ON.  Threading (assumption A9):
 *    - EVERY function here is called from the Telemetry task only.  The
 *      SDK runs in pico_cyw43_arch_lwip_poll mode: lwIP and the CYW43 driver
 *      do their work inside net_poll(), in that task.  No lwIP callback runs
 *      in an interrupt, and no interrupt handler calls the kernel.
 *    - The CYW43 GPIO24 host-wake interrupt only marks work pending (SDK
 *      shared IO_IRQ_BANK0 chain, alongside the echo and barcode handlers).
 *    - CYW43 uses PIO1 and two DMA channels (TEAM.md section 3, reserved).
 *
 *  Nothing here ever waits for the network.  net_init() is the one call that
 *  takes time (CYW43 firmware download, about 0.3-1 s of preemptible busy
 *  waiting at the Telemetry priority); the task calls it once at start.
 */

#ifndef DRV_WIFI_H
#define DRV_WIFI_H

#include <stdbool.h>
#include <stdint.h>

/* Return codes (negative = error). */
#define NET_OK              (0)
#define NET_E_INIT          (-1)    /**< cyw43_arch_init failed / not initialised */
#define NET_E_ARG           (-2)
#define NET_E_STATE         (-3)    /**< no MQTT session */
#define NET_E_FULL          (-4)    /**< MQTT output buffer full: frame not queued */
#define NET_E_LWIP          (-5)    /**< other lwIP error */

/* Connection stages (net_stage()). */
#define NET_STAGE_IDLE          (0U)    /**< nothing requested */
#define NET_STAGE_JOINING       (1U)    /**< WiFi join / DHCP in progress */
#define NET_STAGE_RESOLVING     (2U)    /**< DNS lookup of the broker */
#define NET_STAGE_MQTT          (3U)    /**< TCP + MQTT CONNECT, waiting for CONNACK */
#define NET_STAGE_UP            (4U)    /**< session accepted, subscribed */
#define NET_STAGE_FAILED        (5U)    /**< this attempt failed (wrong password, broker refused, ...) */

/** Received command message, as delivered by the broker. */
#define NET_RX_TOPIC_MAX        (32U)   /**< bytes incl. NUL */
#define NET_RX_DATA_MAX         (8U)    /**< commands are 3 bytes; longer = rejected later */

typedef struct
{
    char     topic[NET_RX_TOPIC_MAX];
    uint8_t  data[NET_RX_DATA_MAX];
    uint32_t len;                       /**< FULL received length (may exceed NET_RX_DATA_MAX) */
} net_rx_msg_t;

/**
 * Power up the CYW43 (country from config/secrets.h, default Singapore) in
 * station mode.  Call once from the Telemetry task.  Returns NET_OK or
 * NET_E_INIT (the task retries on the next connection attempt).
 */
int32_t net_init(void);

/** True after a successful net_init(). */
bool net_is_initialised(void);

/** Do pending network work (cyw43_arch_poll) and advance the connect stages.  Never waits. */
void net_poll(uint32_t now_ms);

/**
 * Start (or restart) a connection attempt: join WiFi if the link is down,
 * resolve the broker, open the MQTT session (keep-alive TEL_KEEPALIVE_MS,
 * last will car/status = '0'), then subscribe car/cmd/# (QoS 1).
 */
int32_t net_connect_begin(void);

/** Drop the MQTT session; with leave_wifi also leave the access point. */
void net_teardown(bool leave_wifi);

/** WiFi joined and has an IP address. */
bool net_wifi_up(void);

/** MQTT session accepted (CONNACK OK) and still open. */
bool net_mqtt_up(void);

/** NET_STAGE_* */
uint32_t net_stage(void);

/** Last CYW43 link status or MQTT connection status, for logs. */
int32_t net_last_status(void);

/**
 * Queue one publish (QoS 0) into the MQTT output buffer.  Never waits:
 * returns NET_E_FULL when the buffer has no room, NET_E_STATE when there is
 * no session.
 */
int32_t net_publish(char const *p_topic, uint8_t const *p_buf, uint32_t len, bool retain);

/** Take the oldest received command message.  Returns false when none is waiting. */
bool net_rx_pop(net_rx_msg_t *p_out);

/** Received messages dropped because the receive ring was full. */
uint32_t net_rx_dropped(void);

#endif /* DRV_WIFI_H */
