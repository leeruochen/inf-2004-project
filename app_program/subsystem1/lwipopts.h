/*
 *  lwipopts.h
 *  Subsystem 1 -- lwIP configuration for the Telemetry WiFi backend
 *  (TEL_BACKEND_WIFI=ON only; found by lwIP through the include path the
 *  top-level CMakeLists.txt adds for this folder).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Pure macros.  Sized for this job, not for general networking: one MQTT
 *  connection that sends about 45 small frames per second and receives a
 *  few 3-byte commands.  Based on the Pico SDK examples' NO_SYS settings,
 *  with the buffers cut down (resource efficiency) and the MQTT limits
 *  raised so a busy 20 ms wake is never refused.
 *
 *  Threading: NO_SYS=1 with pico_cyw43_arch_lwip_poll.  lwIP runs only
 *  inside cyw43_arch_poll(), called by the Telemetry task (assumption A9).
 */

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* --- System ---------------------------------------------------------------- */
#define NO_SYS                          1
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define LWIP_CHKSUM_ALGORITHM           3

/* --- Memory (RAM budget about 16 KB, measured in the WiFi build's map) ------ */
#define MEM_SIZE                        4096    /* heap: TCP segments, DNS */
#define MEMP_NUM_TCP_SEG                16
#define MEMP_NUM_ARP_QUEUE              4
#define PBUF_POOL_SIZE                  8       /* receive buffers (about 1.5 KB each) */
#define MEMP_NUM_TCP_PCB                2       /* the MQTT connection + one closing */
#define MEMP_NUM_TCP_PCB_LISTEN         1       /* lwIP needs one; we never listen */
#define DNS_TABLE_SIZE                  2       /* one broker name */
#define DNS_MAX_NAME_LENGTH             64

/* --- Protocols ------------------------------------------------------------- */
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1       /* answer ping: handy when testing */
#define LWIP_RAW                        0
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_UDP                        1       /* DHCP, DNS */
#define LWIP_TCP                        1       /* MQTT */
#define LWIP_DHCP                       1
#define LWIP_DNS                        1       /* the broker may be a host name */
#define DHCP_DOES_ARP_CHECK             0
#define LWIP_DHCP_DOES_ACD_CHECK        0

#define TCP_MSS                         1460
#define TCP_WND                         (2 * TCP_MSS)
#define TCP_SND_BUF                     (2 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_TCP_KEEPALIVE              1

/* --- Netif ----------------------------------------------------------------- */
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_TX_SINGLE_PBUF       1

/* --- MQTT (lwip/apps/mqtt_opts.h) ------------------------------------------ */
/* Every publish, QoS 0 included, holds one request slot until TCP reports it
 * sent; one wake can publish up to 8 buffered events + motion + sensor +
 * heartbeat, plus subscribe and the online status. */
#define MQTT_REQ_MAX_IN_FLIGHT          16
/* Output ring: about 20 frames of topic + 18 B payload. */
#define MQTT_OUTPUT_RINGBUF_SIZE        1024
/* Incoming topic + header; our command topics are under 32 bytes. */
#define MQTT_VAR_HEADER_BUFFER_LEN      64
/* The MQTT client's cyclic timer needs one extra lwIP timeout. */
#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 1)

/* --- Diagnostics: OFF ------------------------------------------------------ */
/* lwIP must never print: printf from the Telemetry task would contend for
 * the stdio lock with the debug drain (rule R3a).  Statistics are off to
 * save RAM. */
#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0
#define MEM_STATS                       0
#define SYS_STATS                       0
#define MEMP_STATS                      0
#define LINK_STATS                      0

#endif /* LWIPOPTS_H */
