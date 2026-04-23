#pragma once
#include <Arduino.h>
#include "fsd_handler.h"
#include "can_driver.h"

/*
 * uds_poller.h — passive UDS/OBD-II diagnostic poller
 *
 * Cycles through a poll table at 1 request/second (only when TX is allowed).
 * Targets BMS, HV bus, and thermal data.
 *
 * Responses on 0x7E8..0x7EF are intercepted in process_frame() and forwarded
 * here for decoding.  All requests + responses are logged via can_dump_log().
 * All frames continue to candump via the normal can_dump_record() path.
 *
 * ISO-TP: single frames are decoded inline; on a First Frame we send a Flow
 * Control (ContinueToSend) so consecutive frames are delivered, then log each
 * one with its sequence tag.
 */

#define UDS_INTERVAL_MS  1000u   // one request per second

// First CAN ID (inclusive) that carries UDS diagnostic responses
#define UDS_RESP_ID_LO   0x7E8u
// Last CAN ID (inclusive) — covers ECU slots 0..7 (0x7E0..0x7E7 → 0x7E8..0x7EF)
#define UDS_RESP_ID_HI   0x7EFu

struct UDSState {
    int      poll_index;       // next entry in poll table to send
    uint32_t last_tx_ms;       // timestamp of last sent request
    uint32_t requests_sent;
    uint32_t responses_rx;
    CanDriver *can;            // back-pointer for ISO-TP flow control replies
};

/** Initialise state.  Must be called after the CAN driver is ready. */
void uds_poller_init(UDSState *state, CanDriver *can);

/** Call from loop().  Sends next poll entry if interval has elapsed and
 *  fsd_can_transmit() is true.  No-op in Listen-Only or OTA mode. */
void uds_poller_tick(UDSState *state, const FSDState *fsd, uint32_t now_ms);

/** Returns true if this CAN ID is a UDS diagnostic response (0x7E8..0x7EF). */
bool uds_is_response_id(uint32_t id);

/** Decode and log a UDS/OBD-II response frame.  Sends ISO-TP flow control if
 *  needed so consecutive frames arrive.  Should be called from process_frame()
 *  for any frame where uds_is_response_id() returns true. */
void uds_handle_response(UDSState *state, const CanFrame &frame);
