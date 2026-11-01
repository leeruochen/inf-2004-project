/*
 *  drv_wifi.c
 *  Subsystem 1 -- WiFi (CYW43) + MQTT (lwIP) driver.  SDK side: SDK and
 *  pure project headers only, never kernel headers (rule R1).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Built only with TEL_BACKEND_WIFI=ON.  See drv_wifi.h for the threading
 *  rules: every function runs in the Telemetry task; lwIP and the CYW43
 *  driver work inside cyw43_arch_poll() (pico_cyw43_arch_lwip_poll).  The
 *  lwIP callbacks below therefore also run in the Telemetry task, never in
 *  an interrupt, so they may touch the static state without locks.
 *
 *  Connection stages (net_connect_begin, then net_poll each wake):
 *    JOINING     cyw43_arch_wifi_connect_async(), wait for CYW43_LINK_UP
 *    RESOLVING   broker IP literal, or dns_gethostbyname()
 *    MQTT        mqtt_client_connect(), wait for CONNACK (connection callback)
 *    UP          subscribed to car/cmd/# (QoS 1), car/status = '1' (retained)
 *    FAILED      any step failed; the Telemetry link FSM times out, tears
 *                down and retries after its backoff
 *
 *  No call here waits for the network.  The MQTT client is a static object
 *  (no lwIP heap), with the last will car/status = '0' (retained), so the
 *  broker tells the dashboard when the car drops off.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"    /* struct mqtt_client_s, for a static client */
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#include "config/secrets.h"         /* WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_HOST, MQTT_BROKER_PORT */
#include "config/timing.h"          /* TEL_KEEPALIVE_MS */
#include "src/debug/dbg.h"          /* pure header: the dbg_ ring */
#include "drv_wifi.h"
#include "tel_wire.h"               /* topic names (pure) */

/* Optional settings in config/secrets.h, with defaults. */
#ifndef WIFI_COUNTRY
#define WIFI_COUNTRY            CYW43_COUNTRY_SINGAPORE
#endif
#ifndef WIFI_AUTH
#define WIFI_AUTH               CYW43_AUTH_WPA2_AES_PSK
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID          "robocar-is16"
#endif
#ifndef MQTT_USER
#define MQTT_USER               NULL        /* anonymous */
#endif
#ifndef MQTT_PASS
#define MQTT_PASS               NULL
#endif

#define NET_RX_SLOTS            (4U)        /* commands are rare: 4 is plenty */
#define NET_KEEPALIVE_S         ((uint16_t)(TEL_KEEPALIVE_MS / 1000U))

_Static_assert((NET_RX_SLOTS & (NET_RX_SLOTS - 1U)) == 0U, "NET_RX_SLOTS must be a power of two");
_Static_assert((TEL_KEEPALIVE_MS / 1000U) >= 1U, "MQTT keep-alive is at least 1 s");

static bool          s_init = false;
static uint32_t      s_stage = NET_STAGE_IDLE;
static int32_t       s_last_status = 0;
static bool          s_wifi_up = false;
static bool          s_mqtt_up = false;
static bool          s_dns_pending = false;
static bool          s_dns_done = false;
static bool          s_dns_failed = false;
static ip_addr_t     s_broker;
static mqtt_client_t s_client;

/* Receive ring of command messages (filled by the lwIP callbacks). */
static net_rx_msg_t  s_rx[NET_RX_SLOTS];
static uint32_t      s_rx_head = 0U;
static uint32_t      s_rx_tail = 0U;
static uint32_t      s_rx_dropped = 0U;
static net_rx_msg_t  s_rx_cur;          /* message being assembled */
static uint32_t      s_rx_fill = 0U;

/* ---------------------------------------------------------------------------
 * lwIP callbacks (run inside cyw43_arch_poll, in the Telemetry task)
 * ------------------------------------------------------------------------- */

static void on_subscribed(void *p_arg, err_t err)
{
    (void)p_arg;
    if (err != ERR_OK)
    {
        dbg_printf("[NET] subscribe %s failed: %d", TEL_TOPIC_CMD_FILTER, (int)err);
    }
}

static void on_mqtt_connection(mqtt_client_t *p_client, void *p_arg, mqtt_connection_status_t status)
{
    (void)p_arg;
    s_last_status = (int32_t)status;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        static uint8_t const k_online[1] = { (uint8_t)TEL_STATUS_ONLINE[0] };

        s_mqtt_up = true;
        s_stage = NET_STAGE_UP;
        (void)mqtt_subscribe(p_client, TEL_TOPIC_CMD_FILTER, 1U, on_subscribed, NULL);
        (void)mqtt_publish(p_client, TEL_TOPIC_STATUS, k_online, 1U, 1U, 1U, NULL, NULL);
        dbg_puts("[NET] MQTT session up");
    }
    else
    {
        s_mqtt_up = false;
        if ((s_stage == NET_STAGE_MQTT) || (s_stage == NET_STAGE_UP))
        {
            s_stage = NET_STAGE_FAILED;
        }
        dbg_printf("[NET] MQTT closed: status %d", (int)status);
    }
}

static void on_incoming_publish(void *p_arg, char const *p_topic, u32_t tot_len)
{
    size_t const n = strlen(p_topic);

    (void)p_arg;
    if (n < NET_RX_TOPIC_MAX)
    {
        (void)memcpy(s_rx_cur.topic, p_topic, n + 1U);
    }
    else
    {
        s_rx_cur.topic[0] = '\0';   /* too long: rejected later as BAD_TOPIC */
    }
    s_rx_cur.len = (uint32_t)tot_len;
    s_rx_fill = 0U;
}

static void on_incoming_data(void *p_arg, u8_t const *p_data, u16_t len, u8_t flags)
{
    uint32_t i;

    (void)p_arg;
    for (i = 0U; (i < len) && (s_rx_fill < NET_RX_DATA_MAX); i++)
    {
        s_rx_cur.data[s_rx_fill] = p_data[i];
        s_rx_fill++;
    }
    if ((flags & MQTT_DATA_FLAG_LAST) == 0U)
    {
        return;     /* more fragments follow */
    }
    if ((s_rx_head - s_rx_tail) >= NET_RX_SLOTS)
    {
        s_rx_dropped++;
        return;
    }
    s_rx[s_rx_head & (NET_RX_SLOTS - 1U)] = s_rx_cur;
    s_rx_head++;
}

static void on_dns_found(char const *p_name, ip_addr_t const *p_addr, void *p_arg)
{
    (void)p_name;
    (void)p_arg;
    s_dns_pending = false;
    if (p_addr != NULL)
    {
        s_broker = *p_addr;
        s_dns_done = true;
    }
    else
    {
        s_dns_failed = true;
    }
}

/* ---------------------------------------------------------------------------
 * Stages
 * ------------------------------------------------------------------------- */

static void start_mqtt(void)
{
    struct mqtt_connect_client_info_t ci;
    err_t                      err;

    (void)memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASS;
    ci.keep_alive = NET_KEEPALIVE_S;
    ci.will_topic = TEL_TOPIC_STATUS;
    ci.will_msg = TEL_STATUS_OFFLINE;
    ci.will_qos = 1U;
    ci.will_retain = 1U;

    /* The CONNECT packet is built inside this call, so ci may live on the stack. */
    err = mqtt_client_connect(&s_client, &s_broker, (u16_t)MQTT_BROKER_PORT, on_mqtt_connection, NULL, &ci);
    if (err != ERR_OK)
    {
        s_last_status = (int32_t)err;
        s_stage = NET_STAGE_FAILED;
        dbg_printf("[NET] MQTT connect start failed: %d", (int)err);
        return;
    }
    /* mqtt_client_connect() wipes the client, so set the receive callbacks after it. */
    mqtt_set_inpub_callback(&s_client, on_incoming_publish, on_incoming_data, NULL);
    s_stage = NET_STAGE_MQTT;
}

static void step_resolving(void)
{
    if (!s_dns_done && !s_dns_pending && !s_dns_failed)
    {
        if (ipaddr_aton(MQTT_BROKER_HOST, &s_broker) != 0)
        {
            s_dns_done = true;      /* an IP literal */
        }
        else
        {
            err_t const err = dns_gethostbyname(MQTT_BROKER_HOST, &s_broker, on_dns_found, NULL);

            if (err == ERR_OK)
            {
                s_dns_done = true;  /* cached */
            }
            else if (err == ERR_INPROGRESS)
            {
                s_dns_pending = true;
            }
            else
            {
                s_dns_failed = true;
            }
        }
    }
    if (s_dns_failed)
    {
        s_stage = NET_STAGE_FAILED;
        dbg_printf("[NET] cannot resolve %s", MQTT_BROKER_HOST);
    }
    else if (s_dns_done)
    {
        start_mqtt();
    }
    else
    {
        /* waiting for the DNS answer */
    }
}

/* ---------------------------------------------------------------------------
 * API (drv_wifi.h)
 * ------------------------------------------------------------------------- */

int32_t net_init(void)
{
    if (s_init)
    {
        return NET_OK;
    }
    if (cyw43_arch_init_with_country(WIFI_COUNTRY) != 0)
    {
        return NET_E_INIT;
    }
    cyw43_arch_enable_sta_mode();
    /* Light power saving: commands still arrive within tens of ms. */
    (void)cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    s_init = true;
    return NET_OK;
}

bool net_is_initialised(void)
{
    return s_init;
}

void net_poll(uint32_t now_ms)
{
    int32_t link;

    (void)now_ms;
    if (!s_init)
    {
        return;
    }
    cyw43_arch_poll();

    link = (int32_t)cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    s_wifi_up = (link == CYW43_LINK_UP);
    s_mqtt_up = s_mqtt_up && (mqtt_client_is_connected(&s_client) != 0U);

    switch (s_stage)
    {
        case NET_STAGE_JOINING:
            s_last_status = link;
            if (s_wifi_up)
            {
                dbg_printf("[NET] WiFi up, IP %s", ip4addr_ntoa(netif_ip4_addr(netif_default)));
                s_stage = NET_STAGE_RESOLVING;
            }
            else if (link < 0)
            {
                s_stage = NET_STAGE_FAILED;     /* FAIL, NONET or BADAUTH */
                dbg_printf("[NET] WiFi join failed: %ld", (long)link);
            }
            else
            {
                /* JOIN / NOIP: still associating or waiting for DHCP */
            }
            break;

        case NET_STAGE_RESOLVING:
            step_resolving();
            break;

        case NET_STAGE_UP:
            if (!s_mqtt_up)
            {
                s_stage = NET_STAGE_FAILED;
            }
            break;

        default:
            /* IDLE, MQTT (waiting for the callback) and FAILED: nothing to do */
            break;
    }
}

int32_t net_connect_begin(void)
{
    if (!s_init)
    {
        return NET_E_INIT;
    }
    s_mqtt_up = false;
    s_dns_pending = false;
    s_dns_done = false;
    s_dns_failed = false;

    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP)
    {
        s_stage = NET_STAGE_RESOLVING;
        return NET_OK;
    }
    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH) != 0)
    {
        s_stage = NET_STAGE_FAILED;
        return NET_E_LWIP;
    }
    s_stage = NET_STAGE_JOINING;
    return NET_OK;
}

void net_teardown(bool leave_wifi)
{
    if (!s_init)
    {
        return;
    }
    /* Closes TCP without the MQTT DISCONNECT packet, so the broker publishes
     * the last will (car/status = '0') and the dashboard sees the drop. */
    mqtt_disconnect(&s_client);
    s_mqtt_up = false;
    s_dns_pending = false;
    if (leave_wifi)
    {
        (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        s_wifi_up = false;
    }
    s_stage = NET_STAGE_IDLE;
}

bool net_wifi_up(void)
{
    return s_wifi_up;
}

bool net_mqtt_up(void)
{
    return s_mqtt_up;
}

uint32_t net_stage(void)
{
    return s_stage;
}

int32_t net_last_status(void)
{
    return s_last_status;
}

int32_t net_publish(char const *p_topic, uint8_t const *p_buf, uint32_t len, bool retain)
{
    err_t err;

    if ((p_topic == NULL) || (p_buf == NULL) || (len > 0xFFFFU))
    {
        return NET_E_ARG;
    }
    if (!s_mqtt_up)
    {
        return NET_E_STATE;
    }
    err = mqtt_publish(&s_client, p_topic, p_buf, (u16_t)len, 0U, retain ? 1U : 0U, NULL, NULL);
    if (err == ERR_OK)
    {
        return NET_OK;
    }
    return (err == ERR_MEM) ? NET_E_FULL : NET_E_LWIP;
}

bool net_rx_pop(net_rx_msg_t *p_out)
{
    if ((p_out == NULL) || (s_rx_head == s_rx_tail))
    {
        return false;
    }
    *p_out = s_rx[s_rx_tail & (NET_RX_SLOTS - 1U)];
    s_rx_tail++;
    return true;
}

uint32_t net_rx_dropped(void)
{
    return s_rx_dropped;
}
