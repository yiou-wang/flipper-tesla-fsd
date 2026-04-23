/*
 * uds_poller.cpp — UDS / OBD-II diagnostic poller
 *
 * Sends one diagnostic request per second when Active mode is on.
 * Logs all requests and all received responses to Serial + can_dump_log().
 * Handles ISO-TP single frames inline and sends Flow Control on First Frames
 * so the ECU delivers consecutive frames.
 */

#include "uds_poller.h"
#include "can_dump.h"
#include <string.h>
#include <stdio.h>

// ── Poll table ────────────────────────────────────────────────────────────────
// ISO-TP single-frame layout:
//   byte[0]   = PCI length (number of payload bytes that follow)
//   byte[1]   = service ID
//   byte[2..] = parameters
//   padding   = 0xAA  (ISO 15765-2)
//
// OBD-II Mode 1/9 (broadcast to all ECUs via 0x7DF):
//   {0x02, 0x01, PID, 0xAA, ...}
//
// UDS Read Data By Identifier — Service 0x22 (ISO 14229):
//   {0x03, 0x22, DID_hi, DID_lo, 0xAA, ...}

struct UDSEntry {
    uint32_t   tx_id;
    uint8_t    payload[8];
    const char *label;
};

static const UDSEntry k_poll_table[] = {
    // ── OBD-II Mode 1 broadcast ────────────────────────────────────────────
    { 0x7DF, {0x02, 0x01, 0x01, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "OBD1-01 monitor_status" },
    { 0x7DF, {0x02, 0x01, 0x05, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "OBD1-05 coolant_temp" },
    { 0x7DF, {0x02, 0x01, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "OBD1-0D vehicle_speed" },
    { 0x7DF, {0x02, 0x01, 0x46, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "OBD1-46 ambient_temp" },
    { 0x7DF, {0x02, 0x01, 0x5C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "OBD1-5C motor_temp" },

    // ── OBD-II Mode 9 broadcast (vehicle info) ─────────────────────────────
    { 0x7DF, {0x02, 0x09, 0x02, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, "OBD9-02 VIN" },

    // ── UDS RDBI (0x22) → BMS module 0x7E7 / responds on 0x7EF ───────────
    // Standard identifiers
    { 0x7E7, {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS F190 VIN" },
    { 0x7E7, {0x03, 0x22, 0xF1, 0x8C, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS F18C serial" },
    { 0x7E7, {0x03, 0x22, 0xF1, 0x86, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS F186 active_session" },
    // Tesla-specific BMS data identifiers (undocumented — log what the ECU returns)
    { 0x7E7, {0x03, 0x22, 0x02, 0xB0, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS 02B0 pack_voltage" },
    { 0x7E7, {0x03, 0x22, 0x02, 0xB1, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS 02B1 pack_current" },
    { 0x7E7, {0x03, 0x22, 0x02, 0xB2, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS 02B2 soc" },
    { 0x7E7, {0x03, 0x22, 0x02, 0xB3, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS 02B3 cell_temp" },
    { 0x7E7, {0x03, 0x22, 0xDD, 0x00, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS DD00 block0" },
    { 0x7E7, {0x03, 0x22, 0xDD, 0x01, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS DD01 block1" },
    // HV bus status identifiers
    { 0x7E7, {0x03, 0x22, 0x03, 0x01, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS 0301 hv_bus_status" },
    { 0x7E7, {0x03, 0x22, 0x03, 0x02, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-BMS 0302 hv_contactors" },

    // ── UDS RDBI → Gateway 0x7E0 / responds on 0x7E8 ──────────────────────
    { 0x7E0, {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-GTW F190 VIN" },
    { 0x7E0, {0x03, 0x22, 0xF1, 0x8C, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-GTW F18C serial" },
    { 0x7E0, {0x03, 0x22, 0xF1, 0x86, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-GTW F186 session" },

    // ── UDS RDBI → module 0x7E1 (VCFRONT / body) ──────────────────────────
    { 0x7E1, {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-M1 F190 VIN" },

    // ── UDS RDBI → module 0x7E2 (thermal/cooling suspected) ───────────────
    { 0x7E2, {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-TMS F190 VIN" },
    { 0x7E2, {0x03, 0x22, 0xDD, 0x00, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-TMS DD00 block0" },

    // ── UDS RDBI → module 0x7E4 (VCLEFT / alt BMS path) ──────────────────
    { 0x7E4, {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-M4 F190 VIN" },
    { 0x7E4, {0x03, 0x22, 0xDD, 0x00, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-M4 DD00 block0" },

    // ── UDS RDBI → module 0x7E6 (DAS / FSD compute) ───────────────────────
    { 0x7E6, {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-DAS F190 VIN" },
    { 0x7E6, {0x03, 0x22, 0xF1, 0x8C, 0xAA, 0xAA, 0xAA, 0xAA}, "UDS-DAS F18C serial" },
};

static const int k_poll_count = (int)(sizeof(k_poll_table) / sizeof(k_poll_table[0]));

// ── Helpers ───────────────────────────────────────────────────────────────────

static const char *uds_nrc_str(uint8_t nrc) {
    switch (nrc) {
        case 0x10: return "generalReject";
        case 0x11: return "serviceNotSupported";
        case 0x12: return "subFnNotSupported";
        case 0x13: return "invalidLength";
        case 0x22: return "conditionsNotCorrect";
        case 0x24: return "requestSequenceError";
        case 0x31: return "requestOutOfRange";
        case 0x33: return "securityAccessDenied";
        case 0x35: return "invalidKey";
        case 0x78: return "responsePending";
        case 0x7E: return "subFnNotSupportedInSession";
        case 0x7F: return "serviceNotSupportedInSession";
        default:   return "unknown";
    }
}

// Format raw bytes as hex string into buf (up to max_bytes bytes shown)
static int fmt_hex(char *buf, int buf_size, const uint8_t *data, int len) {
    int pos = 0;
    for (int i = 0; i < len && pos + 3 < buf_size; i++) {
        pos += snprintf(buf + pos, buf_size - pos, "%02X ", data[i]);
    }
    if (pos > 0 && buf[pos - 1] == ' ') buf[pos - 1] = '\0';  // trim trailing space
    return pos;
}

// ── Init ──────────────────────────────────────────────────────────────────────

void uds_poller_init(UDSState *state, CanDriver *can) {
    memset(state, 0, sizeof(UDSState));
    state->can = can;
    Serial.printf("[UDS] Poller ready — %d entries, 1 req/s in Active mode\n", k_poll_count);
}

// ── Tick: send next request ───────────────────────────────────────────────────

void uds_poller_tick(UDSState *state, const FSDState *fsd, uint32_t now_ms) {
    if (!fsd_can_transmit(fsd)) return;
    if ((now_ms - state->last_tx_ms) < UDS_INTERVAL_MS) return;
    state->last_tx_ms = now_ms;

    const UDSEntry &e = k_poll_table[state->poll_index];

    CanFrame f = {};
    f.id  = e.tx_id;
    f.dlc = 8;
    memcpy(f.data, e.payload, 8);

    state->can->send(f);
    state->requests_sent++;

    char hex[32];
    fmt_hex(hex, sizeof(hex), f.data, 4);  // show first 4 bytes (PCI + service + 1–2 params)
    Serial.printf("[UDS] TX [%02d/%02d] 0x%03X  %-28s  %s\n",
                  state->poll_index + 1, k_poll_count,
                  e.tx_id, e.label, hex);
    can_dump_log("UDS TX [%02d/%02d] 0x%03X %s %s",
                 state->poll_index + 1, k_poll_count, e.tx_id, e.label, hex);

    state->poll_index = (state->poll_index + 1) % k_poll_count;
}

// ── Response ID filter ────────────────────────────────────────────────────────

bool uds_is_response_id(uint32_t id) {
    return (id >= UDS_RESP_ID_LO && id <= UDS_RESP_ID_HI);
}

// ── Response decoder ──────────────────────────────────────────────────────────

void uds_handle_response(UDSState *state, const CanFrame &frame) {
    if (!uds_is_response_id(frame.id)) return;
    if (frame.dlc < 2) return;

    state->responses_rx++;

    uint8_t pci    = frame.data[0];
    uint8_t pci_type = (pci >> 4) & 0x0F;
    char    hex[72];

    // ── Single Frame (PCI type 0) ─────────────────────────────────────────
    if (pci_type == 0x00) {
        uint8_t sf_len = pci & 0x0F;  // number of payload bytes
        if (sf_len == 0 || sf_len > 7) {
            fmt_hex(hex, sizeof(hex), frame.data, frame.dlc);
            Serial.printf("[UDS] RX 0x%03X SF malformed  %s\n", frame.id, hex);
            can_dump_log("UDS RX 0x%03X SF malformed %s", frame.id, hex);
            return;
        }

        uint8_t svc = frame.data[1];

        // Negative Response (0x7F)
        if (svc == 0x7F && sf_len >= 3) {
            uint8_t req_svc = frame.data[2];
            uint8_t nrc     = frame.data[3];
            Serial.printf("[UDS] RX 0x%03X NR  svc=0x%02X nrc=0x%02X (%s)\n",
                          frame.id, req_svc, nrc, uds_nrc_str(nrc));
            can_dump_log("UDS RX 0x%03X NR svc=%02X nrc=%02X %s",
                         frame.id, req_svc, nrc, uds_nrc_str(nrc));
            return;
        }

        // OBD-II Mode 1 positive response (0x41)
        if (svc == 0x41 && sf_len >= 3) {
            uint8_t pid  = frame.data[2];
            // Show the data bytes (after PID)
            int data_len = (int)sf_len - 2;  // subtract service + pid
            fmt_hex(hex, sizeof(hex), frame.data + 3, data_len > 5 ? 5 : data_len);

            int32_t decoded = 0;
            const char *desc = "";
            switch (pid) {
                case 0x05:
                    decoded = (int32_t)frame.data[3] - 40;
                    snprintf(hex, sizeof(hex), "%d°C (raw=%02X)", (int)decoded, frame.data[3]);
                    desc = "coolant_temp";  break;
                case 0x0D:
                    decoded = frame.data[3];
                    snprintf(hex, sizeof(hex), "%d km/h", (int)decoded);
                    desc = "vehicle_speed"; break;
                case 0x46:
                    decoded = (int32_t)frame.data[3] - 40;
                    snprintf(hex, sizeof(hex), "%d°C (raw=%02X)", (int)decoded, frame.data[3]);
                    desc = "ambient_temp";  break;
                case 0x5C:
                    decoded = (int32_t)frame.data[3] - 40;
                    snprintf(hex, sizeof(hex), "%d°C (raw=%02X)", (int)decoded, frame.data[3]);
                    desc = "motor_temp";    break;
                default:
                    fmt_hex(hex, sizeof(hex), frame.data + 3, data_len > 5 ? 5 : data_len);
                    desc = ""; break;
            }
            Serial.printf("[UDS] RX 0x%03X OBD1 PID=%02X  %s  %s\n",
                          frame.id, pid, hex, desc);
            can_dump_log("UDS RX 0x%03X OBD1 PID=%02X %s %s",
                         frame.id, pid, hex, desc);
            return;
        }

        // OBD-II Mode 9 positive response (0x49)
        if (svc == 0x49 && sf_len >= 3) {
            uint8_t pid = frame.data[2];
            fmt_hex(hex, sizeof(hex), frame.data + 3, sf_len - 2 > 5 ? 5 : sf_len - 2);
            Serial.printf("[UDS] RX 0x%03X OBD9 PID=%02X  %s\n", frame.id, pid, hex);
            can_dump_log("UDS RX 0x%03X OBD9 PID=%02X %s", frame.id, pid, hex);
            return;
        }

        // UDS RDBI positive response (0x62)
        if (svc == 0x62 && sf_len >= 4) {
            uint16_t did      = ((uint16_t)frame.data[2] << 8) | frame.data[3];
            int      data_len = (int)sf_len - 3;  // subtract svc + DID_hi + DID_lo
            fmt_hex(hex, sizeof(hex), frame.data + 4, data_len > 4 ? 4 : data_len);
            Serial.printf("[UDS] RX 0x%03X RDBI DID=%04X  [%d B]  %s\n",
                          frame.id, did, data_len, hex);
            can_dump_log("UDS RX 0x%03X RDBI DID=%04X [%dB] %s",
                         frame.id, did, data_len, hex);
            return;
        }

        // Unknown positive service
        fmt_hex(hex, sizeof(hex), frame.data, sf_len + 1 > 8 ? 8 : sf_len + 1);
        Serial.printf("[UDS] RX 0x%03X SF svc=0x%02X len=%d  %s\n",
                      frame.id, svc, sf_len, hex);
        can_dump_log("UDS RX 0x%03X SF svc=%02X len=%d %s",
                     frame.id, svc, sf_len, hex);
        return;
    }

    // ── First Frame (PCI type 1) ──────────────────────────────────────────
    if (pci_type == 0x01) {
        uint16_t total_len = ((uint16_t)(pci & 0x0F) << 8) | frame.data[1];
        uint8_t  svc       = frame.data[2];
        fmt_hex(hex, sizeof(hex), frame.data + 2, 6);  // first 6 data bytes

        Serial.printf("[UDS] RX 0x%03X FF total=%d svc=0x%02X  %s\n",
                      frame.id, total_len, svc, hex);
        can_dump_log("UDS RX 0x%03X FF total=%d svc=%02X %s",
                     frame.id, total_len, svc, hex);

        // RDBI: log the DID so we know what this long response is for
        if (svc == 0x62 && frame.dlc >= 5) {
            uint16_t did = ((uint16_t)frame.data[3] << 8) | frame.data[4];
            Serial.printf("[UDS] RX 0x%03X FF RDBI DID=%04X total=%d bytes\n",
                          frame.id, did, total_len);
            can_dump_log("UDS RX 0x%03X FF RDBI DID=%04X total=%d",
                         frame.id, did, total_len);
        }

        // Send ISO-TP Flow Control — ContinueToSend, no block limit, 10 ms ST
        // FC goes on the corresponding request ID (response_id - 8)
        if (state->can && frame.id >= 0x008u) {
            CanFrame fc = {};
            fc.id       = frame.id - 0x008u;   // e.g. 0x7EF → 0x7E7
            fc.dlc      = 8;
            fc.data[0]  = 0x30;  // FC PCI, ContinueToSend
            fc.data[1]  = 0x00;  // block size: 0 = no limit
            fc.data[2]  = 0x0A;  // STmin = 10 ms
            fc.data[3]  = fc.data[4] = fc.data[5] = fc.data[6] = fc.data[7] = 0xAA;
            state->can->send(fc);
            can_dump_log("UDS TX 0x%03X FC (flow-ctrl for 0x%03X)", fc.id, frame.id);
        }
        return;
    }

    // ── Consecutive Frame (PCI type 2) ────────────────────────────────────
    if (pci_type == 0x02) {
        uint8_t sn = pci & 0x0F;
        fmt_hex(hex, sizeof(hex), frame.data + 1, 7);
        Serial.printf("[UDS] RX 0x%03X CF[%d]  %s\n", frame.id, sn, hex);
        can_dump_log("UDS RX 0x%03X CF[%d] %s", frame.id, sn, hex);
        return;
    }

    // ── Other / malformed ─────────────────────────────────────────────────
    fmt_hex(hex, sizeof(hex), frame.data, frame.dlc);
    Serial.printf("[UDS] RX 0x%03X unknown PCI=0x%02X  %s\n", frame.id, pci, hex);
    can_dump_log("UDS RX 0x%03X unknown PCI=%02X %s", frame.id, pci, hex);
}
