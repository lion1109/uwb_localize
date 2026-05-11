#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_SPEED 299702547UL // [m/s], ITU-Standard for air (15°C, 1013 hPa)

void fatal_error(const char* error);

#ifndef NDEBUG
uint64_t uwb_board_get_last_isr_esp_ts(); // just for performance debugging! (not NDEBUG!)
uint64_t uwb_board_get_last_spin_rx_esp_ts(); // just for performance debugging! (not NDEBUG!)
#endif


// Timestamp

typedef uint64_t timestamp_t; 

#define TIMESTAMP_BYTES 5
#define TIMESTAMP_NONE  0x10000000000
#define TIMESTAMP_MASK  0x0FFFFFFFFFFULL

static inline int64_t uwb_board_timestamp_diff(timestamp_t t1, timestamp_t t0)
{
    // Calculate signed difference of two 40-bit wrapping timestamps.
    //
    // Computes:
    //   (t1 - t0) mod 2^40
    //
    // and interprets the result as signed two's complement value
    // in the range:
    //
    //   [-2^39, 2^39 - 1]
    //
    // Result is correct as long as the true time difference
    // is smaller than +/- 2^39 ticks (~8.6 s for DW3000).

    const uint64_t MASK40 = (1ULL << 40) - 1;
    const uint64_t SIGN40 = 1ULL << 39;

    uint64_t d = (t1 - t0) & MASK40;

    // manual 40-bit sign extension to int64_t
    if (d & SIGN40)
        d |= ~MASK40;

    return (int64_t)d;
}


// Board and init

typedef struct {
  uint8_t  name[32];
  uint8_t  timestamp_bits; 
  uint64_t crystal_frequency;
  uint32_t pll_factor;
  uint64_t timestamp_frequency;
  uint32_t tx_antenna_delay;
  uint32_t rx_antenna_delay;
  uint64_t max_spi_frequency;
  uint16_t min_delay_us;
} uwb_board_attributes_t;

const uwb_board_attributes_t *uwb_board_get_attributes();
void uwb_board_attributes_out(const uwb_board_attributes_t *attrs);
/* board_init checks if timestamp_bits fit in TIMESTAMP_BYTES */

void uwb_board_set_spi_frequency(const uint32_t freq);
void uwb_board_init(void);

void uwb_board_wakeup(void); // start transceiving devices
void uwb_board_sleep(void); // go to sleep (save power)
void uwb_board_lowpower(void); // got to low power mode (led off)

uint16_t uwb_board_get_id(uint16_t start = 0xFFFF); // modify start to get different values 

timestamp_t uwb_board_get_systime();      // current board time
timestamp_t uwb_board_get_tx_min_delay(); // min number of ticks delay before board_transmit_delayed() 
uint32_t uwb_board_get_ticks_per_ns();    // number of timestamp ticks per ns


// Asynchronous functions

void uwb_board_spin(TickType_t timeout);      // wait and handle events (receive) until uwb_board_spin_abort() is called
void uwb_board_spin_once(TickType_t timeout); // wait and handle one event (receive), uwb_board_spin_abort() may abort
void uwb_board_spin_abort();                  // abort uwb_board_spin_once() or uwb_board_spin()

#define UWB_DELAY_NONE        0
#define UWB_DELAY_ABSOLUTE    1
#define UWB_DELAY_RX_RELATIVE 2
#define UWB_DELAY_TX_RELATIVE 3
#define UWB_DELAY_MASK        0x3

#define TRX_CB_TIMESTAMP      0x10
#define TRX_AUTO_RESTART      0x20

#define TX_STAT_SUCESS 0x0
#define TX_STAT_ERROR 0x1

typedef void (*uwb_board_receive_cb_t)(const uint8_t *frame, uint16_t frame_len, timestamp_t rx_timestamp); // rx callback, timestamp if TRX_CB_TIMESTAMP
typedef void (*uwb_board_transmit_cb_t)(const uint8_t status, timestamp_t tx_timestamp); // tx callback, timestamp if TRX_CB_TIMESTAMP was set
typedef uint8_t cb_flags_t;
void uwb_board_receive_async(uwb_board_receive_cb_t rx_cb, cb_flags_t flags);
#if 0
void uwb_board_transmit_async(uint8_t *frame, uint16_t frame_len, uwb_board_transmit_cb_t tx_cb, cb_flags_t flags);
void uwb_board_transmit_async_delayed(uint8_t *frame, uint16_t frame_len, uwb_board_transmit_cb_t tx_cb, cb_flags_t flags, timestamp_t tx_timestamp);
#endif


// Synchronous functions

const uint8_t *uwb_board_receive(uint16_t *frame_len_ptr, timestamp_t *rx_ts_ptr ); // receive frame and optionally get rx_timestamp

/* The uwb_board_transmit function can start the transmission at a specified future time by using a delay_type.
 * The specified time has to be at least uwb_board_get_tx_min_delay() ticks in the future. If the specified time is to early,
 * then uwb_board_transmit_delayed() returns an error.
 * The bord may be not able to use any future time, so uwb_board_transmit_delayed(frame,len,planned_time)
 * may return a tx_timestamp which may be different from the planned_time.
 * To assure the returned tx_timestamp is equal to the planned_time, the planned_time can be calculated as: 
 * 
 *   planned_time = uwb_board_plan_delayed(wished_tx_time) // to get a planned_time near the wished_tx_time, with tx_timestamp == planned_time
 *
 * Example code:
 * 
 * wished_time = uwb_board_get_systime() + uwb_board_get_tx_min_delay() + extra_time_for_complex_calculations_creating_frame;
 * planned_time = uwb_board_plan_delayed(wished_tx_time);
 * frame[] = ... create frame, frame may optionally include planned_time to send exactly tx_timestamp in the frame itself
 * if (uwb_board_transmit_delayed(frame, len, planned_time)) {
 *   sucess ...
 * } else {
 *   failure ...
 * } 
 *
 * To startup rx directly after tx set the flag UWB_RX_AFTER_TX
 */

void uwb_board_prepare_transmit(); // may be called to early prepare for a transmit makes uwb_board_transmit() a bit faster 
bool uwb_board_transmit( // transmit frame near specified tx_timestamp 
    uint8_t               *frame,
    uint16_t               frame_len,
    timestamp_t           *tx_timestamp,
    uint8_t                flags = UWB_DELAY_NONE,
    uwb_board_receive_cb_t rx_cb = nullptr, // calls uwb_board_receive_async(rx_cb, rx_cb_flags) after successfull transmit
    cb_flags_t		   rx_cb_flags = TRX_CB_TIMESTAMP|TRX_AUTO_RESTART
); 
timestamp_t uwb_board_plan_delayed(timestamp_t wished_tx_time); // uwb_board_transmit_delayed just can use planned_time to exactly hit tx_timestamp




typedef struct {
	uint8_t type;  // C5 = 197 = blink 
	uint8_t serial;
	uint8_t info[10];
} uwb_board_frame_simple_t;


// Helper functions for frame data generation/interpretation

static inline uint8_t *put_u8(uint8_t *p, uint8_t v) {
    p[0] = (uint8_t)(v >> 0);
    return p + 1;
}
static inline uint8_t get_u8(const uint8_t *p) {
    return ((uint8_t)p[0] << 0);
}

static inline uint8_t *put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 0);
    p[1] = (uint8_t)(v >> 8);
    return p + 2;
}
static inline uint16_t get_u16(const uint8_t *p) {
    return ((uint16_t)p[0] << 0) |
           ((uint16_t)p[1] << 8);
}

static inline uint8_t *put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >>  0);
    p[1] = (uint8_t)(v >>  8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    return p + 4;
}
static inline uint32_t get_u32(const uint8_t *p) {
    return ((uint32_t)p[0] <<  0)  |
           ((uint32_t)p[1] <<  8)  |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline uint8_t *put_u64(uint8_t *p, uint64_t v) {
    p[0] = (uint8_t)(v >> 0);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    p[4] = (uint8_t)(v >> 32);
    p[5] = (uint8_t)(v >> 40);
    p[6] = (uint8_t)(v >> 48);
    p[7] = (uint8_t)(v >> 56);
    return p + 8;
}
static inline uint64_t get_u64(const uint8_t *p) {
    return ((uint64_t)p[0] << 0)  |
           ((uint64_t)p[1] << 8)  |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
} 
static inline uint8_t *put_u64_5(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)(v >>  0);
    p[1] = (uint8_t)(v >>  8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    p[4] = (uint8_t)(v >> 32);
    return p + 5;
}
static inline uint64_t get_u64_5(const uint8_t *p)
{
    return ((uint64_t)p[0] <<  0)  |
           ((uint64_t)p[1] <<  8)  |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32);
}


// IEEE 802,15,4 network functions

void uwb_board_set_pan_shortaddress(uint16_t pan, uint16_t uwb_board_addr);

float uwb_board_get_carrier_freq_offset(); // after rx, device freq difference [Hz]
float uwb_board_get_clock_freq_offset(); // after rx, system clock freq difference [ppm] 



#ifdef __cplusplus
}
#endif

