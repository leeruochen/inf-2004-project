/*
 *  tel_wire.h
 *  Subsystem 1 -- MQTT topic tree and the command receive check.
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  PURE: <stdbool.h>, <stdint.h> and src/common/data_repr.h (itself pure)
 *  only, so it builds on the host (tests/host/test_tel.c).
 *
 *  Payload format: the binary frames of docs/DATA_REPRESENTATION.md
 *  section 8 (ver u8 | type u8 | len u8 | payload), built by the drep_frame_*
 *  functions.  This file only adds WHERE each frame goes (its topic) and the
 *  extra check that a command arrived on the topic that matches it.
 *
 *  Topic tree (app_program/subsystem1/MQTT.md):
 *    car/telemetry/motion      frame type 1, every 200 ms       QoS 0
 *    car/telemetry/sensor      frame type 2, every 200 ms       QoS 0
 *    car/telemetry/barcode     frame type 3, event              QoS 0
 *    car/telemetry/hump        frame type 4, event              QoS 0
 *    car/telemetry/obstacle    frame type 5, event              QoS 0
 *    car/telemetry/fault       frame type 6, event              QoS 0
 *    car/telemetry/heartbeat   frame type 7, every 1 s          QoS 0
 *    car/status                1 byte '1' online / '0' offline (MQTT last will), retained
 *    car/cmd/start|stop|reset  frame types 8/9/10 (dashboard -> car), subscribed QoS 1
 */

#ifndef TEL_WIRE_H
#define TEL_WIRE_H

#include <stdbool.h>
#include <stdint.h>

#include "src/common/data_repr.h"

#define TEL_TOPIC_STATUS            "car/status"
#define TEL_TOPIC_CMD_FILTER        "car/cmd/#"
#define TEL_TOPIC_CMD_PREFIX        "car/cmd/"
#define TEL_STATUS_ONLINE           "1"
#define TEL_STATUS_OFFLINE          "0"

/** Longest topic string we publish or accept, without the NUL. */
#define TEL_TOPIC_MAX               (31U)

/* Extra reject reasons on top of DREP_FRAME_* (data_repr.h). */
#define TEL_WIRE_BAD_TOPIC          (-6)    /**< not car/cmd/start|stop|reset */
#define TEL_WIRE_TOPIC_MISMATCH     (-7)    /**< e.g. a STOP frame on car/cmd/start */

/** Topic for a telemetry frame type (DREP_FT_TELEMETRY_MOTION .. DREP_FT_HEARTBEAT), or NULL. */
char const *tel_wire_topic(uint32_t frame_type);

/** True for the frame types sent periodically (motion, sensor, heartbeat): never buffered. */
bool tel_wire_is_periodic(uint32_t frame_type);

/**
 * Receive check for one MQTT message on the command topics.  Accepts it only
 * if the frame passes drep_cmd_to_nav() (version, type, length) AND the
 * topic is the one for that command.  On success fills *p_nav and returns
 * DREP_FRAME_OK; otherwise returns the reject reason (DREP_FRAME_* or
 * TEL_WIRE_*) and leaves *p_nav unchanged, so a rejected frame can never
 * reach the Vehicle Controller.
 */
int32_t tel_wire_cmd(char const *p_topic, uint8_t const *p_buf, uint32_t n, nav_input_t *p_nav);

/** Short name of a reject reason for logs ("BAD_VERSION", ...). */
char const *tel_wire_reject_name(int32_t rc);

/**
 * Lower-case hex of n bytes into p_out (2n characters + NUL), cut to fit
 * out_bytes.  Returns the number of characters written (without the NUL).
 */
uint32_t tel_wire_hex(char *p_out, uint32_t out_bytes, uint8_t const *p_buf, uint32_t n);

#endif /* TEL_WIRE_H */
