/*
 *  tel_link.h
 *  Subsystem 1 -- WiFi / MQTT connection state machine (design.md
 *  statechart 6.5) with exponential reconnect backoff (FR3).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  PURE logic: <stdbool.h> and <stdint.h> only.  No kernel, no SDK, no
 *  network calls.  The Telemetry task feeds it what it observed this wake
 *  (link up?  did a publish fail?  is the outbox empty?) and carries out the
 *  action it returns.  This keeps every transition host-testable
 *  (tests/host/test_tel.c).
 *
 *  States (3 bits, docs/DATA_REPRESENTATION.md section 4, WiFi FSM):
 *
 *    DISCONNECTED --start--> CONNECTING --net up--> CONNECTED
 *        CONNECTING --attempt failed or connect timeout--> RECONNECTING (teardown)
 *        CONNECTED  --net down or publish failed--> DEGRADED_BUFFERING
 *        DEGRADED   --net up, publish ok, outbox empty--> CONNECTED
 *        DEGRADED   --still bad after T_ka--> RECONNECTING (teardown)
 *        RECONNECTING --backoff expired--> CONNECTING (connect)
 *
 *  Backoff: the first wait is backoff_min_ms (1 s), then it doubles on every
 *  failed attempt (2 s, 4 s, ...) up to backoff_max_ms (30 s).  A successful
 *  connection resets it to backoff_min_ms.
 *
 *  All times are kernel milliseconds (uint32_t) and compared wrap-safely.
 */

#ifndef TEL_LINK_H
#define TEL_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* States (DREP_FSM_WIFI_STATES = 5, 3 bits). */
#define TEL_LINK_DISCONNECTED       (0U)
#define TEL_LINK_CONNECTING         (1U)
#define TEL_LINK_CONNECTED          (2U)
#define TEL_LINK_DEGRADED           (3U)    /**< DEGRADED_BUFFERING */
#define TEL_LINK_RECONNECTING       (4U)
#define TEL_LINK_STATE_COUNT        (5U)

/* Actions returned by tel_link_step() (bit mask). */
#define TEL_LINK_ACT_NONE           (0U)
#define TEL_LINK_ACT_CONNECT        (1U << 0)   /**< start a connection attempt */
#define TEL_LINK_ACT_TEARDOWN       (1U << 1)   /**< drop the MQTT session (and WiFi) */

/** Timing parameters (config/timing.h values in the firmware). */
typedef struct
{
    uint32_t connect_timeout_ms;    /**< CONNECTING longer than this = failed attempt */
    uint32_t degrade_timeout_ms;    /**< T_ka: DEGRADED longer than this = reconnect */
    uint32_t backoff_min_ms;        /**< first reconnect wait */
    uint32_t backoff_max_ms;        /**< reconnect wait cap */
} tel_link_cfg_t;

/** What the Telemetry task observed during this wake. */
typedef struct
{
    bool net_up;            /**< WiFi joined AND MQTT session accepted */
    bool attempt_failed;    /**< the connection attempt failed for certain (bad password, broker refused) */
    bool publish_failed;    /**< at least one publish was refused this wake */
    bool outbox_empty;      /**< no buffered event frame is waiting */
} tel_link_in_t;

typedef struct
{
    tel_link_cfg_t cfg;
    uint32_t       state;           /**< TEL_LINK_* */
    uint32_t       entered_ms;      /**< time the current state was entered */
    uint32_t       backoff_ms;      /**< wait used by the NEXT reconnect */
    uint32_t       retry_at_ms;     /**< RECONNECTING: time to try again */
    uint32_t       last_wait_ms;    /**< RECONNECTING: the wait chosen (for logs/tests) */
    bool           ever_connected;  /**< at least one CONNECTED since boot */
    bool           last_net_up;     /**< net_up seen by the last step */
    uint32_t       connects;        /**< entries into CONNECTED */
    uint32_t       failures;        /**< failed attempts / sessions torn down */
} tel_link_t;

/** Reset to DISCONNECTED at time now_ms. */
void tel_link_init(tel_link_t *p_link, tel_link_cfg_t const *p_cfg, uint32_t now_ms);

/**
 * Advance the state machine by one wake.  Returns TEL_LINK_ACT_* bits the
 * caller must carry out (TEARDOWN before CONNECT when both are set).
 */
uint32_t tel_link_step(tel_link_t *p_link, uint32_t now_ms, tel_link_in_t const *p_in);

/** Current state (TEL_LINK_*). */
uint32_t tel_link_state(tel_link_t const *p_link);

/**
 * True while the link counts as LOST for the EVF_SAFETY FAULT_WIFI_LOSS bit:
 * it was connected at least once and the network is not up now.  A full
 * MQTT output buffer with the network still up is not a loss.
 */
bool tel_link_is_lost(tel_link_t const *p_link);

/** True while frames may be handed to the network (CONNECTED or DEGRADED with the network up). */
bool tel_link_can_send(tel_link_t const *p_link);

/** Short upper-case name for logs ("CONNECTED", ...). */
char const *tel_link_state_name(uint32_t state);

#endif /* TEL_LINK_H */
