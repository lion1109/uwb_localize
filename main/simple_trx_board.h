#pragma once

#include "uwb_board.h"

// This header defines data structures and allows access by unit tests.

#ifdef UNIT_TEST
#pragma message "compiling TEST, non production"
#define STATIC 
#define EXTERN extern 
#else
#pragma message "compiling non TEST, production"
#define STATIC static
#define EXTERN static 
#endif

#define RADIO_WAVE_SPEED 299792458.0

// types for twr calculations
typedef int64_t timediff_t; // the difference between two timestamps
typedef int16_t drift_t;    // the drift in ppb, assitional tics per billion

// initialize by board_attributes:
extern uint16_t board_id;            // 16 bit network device address 
extern uint64_t timestamp_frequency; // board timestamp frequency

#define STATISTIC 1
#define USE_FLOAT_MATH 1

#define STAMPS 4
inline uint8_t successor_stamp_idx(uint8_t idx) { return (idx < STAMPS-1) ? idx + 1  : 0;       }
inline uint8_t predessor_stamp_idx(uint8_t idx) { return (idx == 0      ) ? STAMPS-1 : idx - 1; }
typedef struct {
    portMUX_TYPE mux;                   // mutex for task access

    // values set by radio task
    uint16_t     device_id;             // netword address of anchor
    uint8_t      consecutive;           // number of consecutive stamps: 0 -> no stamps recorded, 1 -> no consecutive
    uint8_t      last_idx;              // last_recorded index (if consecutive)
    uint8_t      last_seq_no;           // seq_no of last received frame   
    uint8_t      last_req_seq_no;       // seq_no of last send (request) frame    
    timestamp_t  tx_timestamp[STAMPS];  // frames remote tx timestamp
    timestamp_t  rx_timestamp[STAMPS];  // frames local rx timestamp
    timestamp_t  req_tx_ts[STAMPS];     // requesting frame local tx timestamp
    timestamp_t  req_rx_ts[STAMPS];     // requesting frame remote rx timestamp
#ifndef NDEBUG
    timestamp_t  rx_esp_ts[STAMPS];     // just for debugging
#endif

    // values set by ranging task
#if USE_FLOAT_MATH
    float        drift;    // [delta remote ticks / local tics], relative drift of remote clock
#endif
    timestamp_t  offset;   // [ticks], offset of remote board at last rx_timestamp (t6)
    float        distance; // [m], distance of antennas while last DS-TWR
    float        distance_ema;

#if STATISTIC
    float        drift_ema;
    float        drift_var_ema;
    float        frequency;
#endif
} anchor_data_t;



/* Frame type definitions:
 *
 * Blink frame:
 *   uint8_t     FRAME_TYPE_BLINK
 *   uint8_t     seq_no 
 *   uint8_t     text[8] 
 *
 * Test frame:
 *   uint8_t     FRAME_TYPE_TEST
 *   uint8_t     seq_no 
 *   timestamp_t tx_timestamp // remote tx timestamp of this frame
 *   uint8_t     cmd          // 0 or FRAME_CMD_REQ4RESPONSE
 *   timestamp_t req_rx_ts    // remote rx timestamp of request frape
 *
 * Anchor frame:
 *   uint8_t     FRAME_TYPE_ANCHOR
 *   uint8_t     seq_no 
 *   timestamp_t tx_timestamp // remote tx timestamp of this frame
 *   uint8_t     flags        // 0 or combination of FRAME_ANCHOR_FLAG_*
 *
 *   // data of requesting frame if flags & FRAME_ANCHOR_FLAG_ISRESP
 *   uint8_t     req_seq_no   // seq_no
 *   timestamp_t req_tx_ts    // local tx timestamp of requesting frame, equal to last_transmit_timestamp?
 *   timestamp_t req_rx_ts    // remote rx timestamp of requesting frame
 */
#define FRAME_TYPE_BLINK  197 // taken from simple_tx
#define FRAME_TYPE_TEST   198
#define FRAME_TYPE_ANCHOR 199

#define FRAME_TEST_REQ4RESPONSE 99

#define FRAME_ANCHOR_REQ4TWR 0x1 // request a response for a twr
#define FRAME_ANCHOR_ISRESP  0x2 // this is a response frame


#ifdef UNIT_TEST
extern anchor_data_t other_anchor;
extern uint8_t next_transmit_seq_no;
extern uint64_t last_transmit_timestamp;
#endif

static inline void update_anchor_by_rx_data(anchor_data_t *anchor, uint8_t idx, uint8_t seq_no, timestamp_t tx_ts, timestamp_t rx_ts, uint8_t consecutive) {
    anchor->last_idx = idx;
    anchor->last_seq_no = seq_no;
    anchor->tx_timestamp[idx] = tx_ts;
    anchor->rx_timestamp[idx] = rx_ts;
#ifndef NDEBUG
    anchor->rx_esp_ts[idx] = uwb_board_get_last_isr_esp_ts();
#endif
    if (consecutive) {
	if (anchor->consecutive < 255)
            anchor->consecutive++;
    } else
	anchor->consecutive = 1;
}

STATIC void board_init(); 

STATIC uint16_t fill_anchor_frame(
    uint8_t *frame, uint8_t seq_no, timestamp_t tx_ts,
    uint8_t req_seq_no, timestamp_t req_tx_ts, timestamp_t req_rx_ts);

STATIC void calculate_drift(anchor_data_t *other_anchor);

STATIC void handle_received_frame(const uint8_t *frame, uint16_t frame_len, timestamp_t rx_ts);

STATIC void ranging(uint16_t device_id);

