#include "build.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

//#include "dw3000.h"
#include "uwb_board.h"
#include "esp_timer.h"


#define APP_NAME "SIMPLE TRX UWB BOARD v0.1"
#define TAG "simple_rx_board"


static QueueHandle_t twr_queue = NULL; // contains device_id of twr to process


#define STATISTIC 1
#if STATISTIC
#define SLIDING_MEAN 20 
static float ema_p = 1.0/SLIDING_MEAN;
#define UPDATE_EMA(var, val) (var) += ema_p * (val-(var)) 
#endif


// initialize by board_attributes:
static uint16_t board_id = 0;            // 16 bit network device address 
static uint64_t timestamp_frequency = 0; // board timestamp frequency

#define USE_FLOAT_MATH 1

#define STAMPS 4
inline uint8_t successor_stamp_idx(uint8_t idx) { return (idx < STAMPS-1) ? idx + 1  : 0;       }
inline uint8_t predessor_stamp_idx(uint8_t idx) { return (idx == 0      ) ? STAMPS-1 : idx - 1; }
typedef struct {
    portMUX_TYPE mux;                   // mutex for task access
    uint16_t     device_id;             // netword address of anchor
    uint8_t      consecutive;           // number of consecutive stamps: 0 -> no stamps recorded, 1 -> no consecutive
    uint8_t      last_idx;              // last_recorded index (if consecutive)
    uint8_t      last_seq_no;           
    timestamp_t  tx_timestamp[STAMPS];  // frames remote tx timestamp
    timestamp_t  rx_timestamp[STAMPS];  // frames local rx timestamp
    timestamp_t  req_tx_ts[STAMPS];     // requesting frame local tx timestamp
    timestamp_t  req_rx_ts[STAMPS];     // requesting frame remote rx timestamp
#ifndef NDEBUG
    timestamp_t  rx_esp_ts[STAMPS];     // just for debugging
#endif

    // values for twr
    uint32_t     drift_per_second;
#if USE_FLOAT_MATH
    float        drift;
#endif

#if STATISTIC
    // statistical values 
    float        drift_ema;
    float        drift_var_ema;
    float        frequency;
#endif
} anchor_data_t;

static anchor_data_t other_anchor = {
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .device_id = 0xFFFF,
    .consecutive = 0,
    .last_idx  = 0,
    .last_seq_no = 0,
    .tx_timestamp = { 0, 0, 0, 0 },
    .rx_timestamp = { 0, 0, 0, 0 },
    .req_tx_ts = { 0, 0, 0, 0 },
    .req_rx_ts = { 0, 0, 0, 0 },
#ifndef NDEBUG
    .rx_esp_ts = { 0, 0 },
#endif

    .drift_per_second = 0,

#if STATISTIC
    .drift = 0,
    .drift_ema = 0.0,
    .drift_var_ema = 0.0,
    .frequency = 0.0,
#endif
};


/* Output of debug data sometimes takes to much performance. 
 * monitor(...) reacts like printf, but drops most output.
 */
static uint16_t monitor_max = 0;
static uint16_t monitor_cnt = 0;
void set_monitor_limit() {
    if (other_anchor.frequency > 8.0) {
        uint16_t mm = ((uint16_t)(2.0 * other_anchor.frequency)) | 1;
	if (abs(mm - monitor_max) > 4) {
	    monitor_max = mm;
	    if (!monitor_cnt || monitor_cnt > monitor_max)
	        monitor_cnt = 1;
	    printf("monitor max: %d\n", monitor_max);
	}
    } else {
	monitor_max = 0;
        monitor_cnt = 0;
    }
}
#define monitor(fmt, ...) if (!monitor_max || (--monitor_cnt < 1)) { printf(fmt, ##__VA_ARGS__); monitor_cnt = monitor_max; }


static void calculate_drift(anchor_data_t *other_anchor)
{
    LOGD(TAG, "calculate_drift");

    uint8_t idx_new = other_anchor->last_idx;
    uint8_t idx_old = predessor_stamp_idx(idx_new);
    int64_t d_tx = uwb_board_timestamp_diff(other_anchor->tx_timestamp[idx_new], other_anchor->tx_timestamp[idx_old]);
    int64_t d_rx = uwb_board_timestamp_diff(other_anchor->rx_timestamp[idx_new], other_anchor->rx_timestamp[idx_old]);

    /* d_tx is 512 bit quantized */

#if STATISTIC
    int32_t delta = (int32_t)(d_tx - d_rx);

    float drift = (float)delta / (float)d_rx; 
    other_anchor->drift = drift;

    float d_mean;
    if (other_anchor->drift_ema == 0.0)
    {
	other_anchor->drift_ema = drift;
        d_mean = 0;
    } else {
        UPDATE_EMA(other_anchor->drift_ema, drift);
        d_mean = drift - other_anchor->drift_ema;
        UPDATE_EMA(other_anchor->drift_var_ema, d_mean * d_mean);
    }

    float freq = timestamp_frequency / d_rx;
    UPDATE_EMA(other_anchor->frequency, freq);

    LOGD(TAG, "d_tx: %lld, d_rx: %lld, delta: %ld, drift: %e, d_mean: %e, drift_ema: %e, drift_var_ema: %e, freq: %e [Hz], freq_ema %e [Hz]",
         d_tx, d_rx, delta, drift, d_mean, other_anchor->drift_ema, other_anchor->drift_var_ema, freq, other_anchor->frequency);
#endif
}


// transmit infos defined global to access in handle_received_frame:
static uint8_t next_transmit_seq_no = 0;
static uint64_t last_transmit_timestamp;
static bool send_response_to_last_frame = false; // handle_receive_frame set this to true if response is requested
static uint8_t last_received_frame_type = 0;

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

#ifndef NDEBUG
uint64_t handler_rx_esp_ts = 0;
#endif

inline void update_other_anchor_by_rx_data(uint8_t idx, uint8_t seq_no, timestamp_t tx_ts, timestamp_t rx_ts, uint8_t consecutive) {
    other_anchor.last_idx = idx;
    other_anchor.last_seq_no = seq_no;
    other_anchor.tx_timestamp[idx] = tx_ts;
    other_anchor.rx_timestamp[idx] = rx_ts;
#ifndef NDEBUG
    other_anchor.rx_esp_ts[idx] = uwb_board_get_last_isr_esp_ts();
#endif
    other_anchor.consecutive = consecutive ? other_anchor.consecutive + 1 : 1;
}

static void handle_received_frame(const uint8_t *frame, uint16_t frame_len, timestamp_t rx_ts)
{
#ifndef NDEBUG
    handler_rx_esp_ts = esp_timer_get_time();
#endif

    // LOGI(TAG, "handle_received_frame: %p, %d, %lld\n", frame, frame_len, rx_ts);
    if (frame) 
    {
  	uint8_t frame_type = frame[0];
  	uint8_t seq_no = frame[1];

	if (FRAME_TYPE_ANCHOR == frame_type) {  
  	    timestamp_t tx_ts = get_u64_5(frame+2);
  	    uint8_t flags = get_u8(frame+7);
            
	    uint8_t idx = successor_stamp_idx(other_anchor.last_idx);
            bool consecutive = seq_no == other_anchor.last_seq_no + 1; // reading access for this task allowed
            LOGD(TAG, "consecutive: %d", consecutive);
	    
	    // get requesting frame's data
	    uint8_t req_seq_no = get_u8(frame+8);
	    timestamp_t req_tx_ts = get_u64_5(frame+9);
	    timestamp_t req_rx_ts = get_u64_5(frame+14);
  	    
            // update achor_data
    	    taskENTER_CRITICAL(&(other_anchor.mux));
	    {   update_other_anchor_by_rx_data(idx, seq_no, tx_ts, rx_ts, consecutive);
	        if (req_seq_no + 1 == next_transmit_seq_no && req_tx_ts == last_transmit_timestamp) {
    		    // this frame is a response to my last transmitted frame
	       	    other_anchor.req_tx_ts[idx] = req_tx_ts;
		    other_anchor.req_rx_ts[idx] = req_rx_ts;
	        } else {
		    other_anchor.req_tx_ts[idx] = TIMESTAMP_NONE;
		    other_anchor.req_rx_ts[idx] = TIMESTAMP_NONE;
	        }
            }
    	    taskEXIT_CRITICAL(&(other_anchor.mux));
	    
	    if (consecutive) calculate_drift(&other_anchor);
  	    
	    set_monitor_limit(); 
	    
            float scl_offset = (uwb_board_get_clock_freq_offset() / 1000000) / 5; 
	    monitor("RX @ %lld test (%d): %d, %d, %lld, flags:%x, %.4e, %.4e, %.4e, %.4f [Hz], %.4e, %.4e, cs:%d,%d\n",
	           rx_ts, frame_len, frame[0], seq_no, tx_ts, flags, 
		   other_anchor.drift, other_anchor.drift_ema, sqrt(other_anchor.drift_var_ema),
		   other_anchor.frequency, scl_offset, scl_offset-other_anchor.drift,
		   consecutive, other_anchor.consecutive);

	    if (flags & FRAME_ANCHOR_REQ4TWR) {
		send_response_to_last_frame = true;
		xQueueSend(twr_queue, &(other_anchor.device_id), 0); // drop if queue is full
	        uwb_board_spin_abort();
	    }

	} else if (FRAME_TYPE_TEST == frame_type) {
  	    timestamp_t tx_ts = get_u64_5(frame+2);
  	    uint8_t cmd = frame[7];
            
	    uint8_t idx = successor_stamp_idx(other_anchor.last_idx);
            bool consecutive = seq_no == other_anchor.last_seq_no + 1;
            LOGD(TAG, "consecutive: %d", consecutive);
            
	    update_other_anchor_by_rx_data(idx, seq_no, tx_ts, rx_ts, consecutive);

	    if (consecutive) calculate_drift(&other_anchor);

	    set_monitor_limit(); 
#if STATISTIC
            //float ccl_offset = uwb_board_get_carrier_freq_offset();
            float scl_offset = (uwb_board_get_clock_freq_offset() / 1000000) / 5; 
	    monitor("RX @ %lld test (%d): %d, %d, %lld, cmd:%d, %.4e, %.4e, %.4e, %.4f [Hz], %.4e, %.4e, cs:%d,%d\n",
	           rx_ts, frame_len, frame[0], seq_no, tx_ts, cmd, 
		   other_anchor.drift, other_anchor.drift_ema, sqrt(other_anchor.drift_var_ema),
		   other_anchor.frequency, scl_offset, other_anchor.drift - scl_offset, consecutive, other_anchor.consecutive);
#else
	    monitor("RX @%lld test (%d): %d, %d, %lld, cmd:%d\n",
	           rx_ts, frame_len, frame[0], seq_no, tx_ts, cmd);
#endif
	    if (FRAME_TEST_REQ4RESPONSE == cmd) {
		send_response_to_last_frame = true;
	        uwb_board_spin_abort();
	    }

	} else if (FRAME_TYPE_BLINK == frame_type) {  
  	    printf("RX @ %lld blink frame received (%d): %d, %d, %.8s\n", rx_ts, frame_len, frame[0], frame[1], frame+2);
            if (true) {
		send_response_to_last_frame = true;
	        // uwb_board_spin_abort();
	    }
        } else {
  	    printf("other frame received (%d): %d, %d\n", frame_len, frame[0], frame[1]);
	}
    
	last_received_frame_type = frame_type;

    } else {
        printf("no frame received\n");
    }
}


static timestamp_t min_delay = uwb_board_get_tx_min_delay();
static uint32_t ticks_per_ns = uwb_board_get_ticks_per_ns();

#ifndef NDEBUG
uint64_t anchor_esp_ts = 0;
#endif

static void transmit_one(uint8_t frame_type = FRAME_TYPE_BLINK, anchor_data_t *other_anchor = nullptr, uwb_board_receive_cb_t = nullptr)
{
    uint8_t frame[19];
    uint8_t *p = frame;

    p = put_u8(p, frame_type);
    p = put_u8(p, next_transmit_seq_no++);

    uint16_t frame_len;
    if (frame_type == FRAME_TYPE_ANCHOR) {
        frame_len = 2 + 5 + 1 + 1 + 5 + 5; // 19 bytes
       
        uwb_board_prepare_transmit();
       
        int64_t esp_now = esp_timer_get_time();
	timestamp_t now = uwb_board_get_systime();
        
	// for tx_ts planning, frame generation
	timestamp_t tx_ts;
	timestamp_t planned;
        uint8_t delay_type;

	// data of requesting frame last received
        uint8_t req_seq_no;
	timestamp_t req_tx_ts; // remote tx timestamp
	timestamp_t req_rx_ts; // local rx timestamp

	// tx_ts planning
	if (other_anchor) {
	    uint8_t last_idx = other_anchor->last_idx;
            req_seq_no = other_anchor->last_seq_no;
            req_tx_ts = other_anchor->tx_timestamp[last_idx];
            req_rx_ts = other_anchor->rx_timestamp[last_idx];
#if 0
            // using UWB_DELAY_RX_RELATIVE fails in 10%
	    static timestamp_t resp_delay = uwb_board_plan_delayed(min_delay << 5);
	    delay_type = UWB_DELAY_RX_RELATIVE;
	    planned = uwb_board_plan_delayed(last_rx + resp_delay);
	    tx_ts = (planned - last_rx) & TIMESTAMP_MASK;
	    planned = (planned + 62 * 256) & TIMESTAMP_MASK;
#else
	    timestamp_t resp_delay = uwb_board_plan_delayed(min_delay + uwb_board_timestamp_diff(now, req_rx_ts));
            delay_type = UWB_DELAY_ABSOLUTE;
	    tx_ts = uwb_board_plan_delayed(req_rx_ts + resp_delay);
	    planned = tx_ts;
#endif
	} else {
            req_seq_no = 0xFF;
            req_tx_ts = 0;
            req_rx_ts = 0;
            delay_type = UWB_DELAY_ABSOLUTE;
	    tx_ts = uwb_board_plan_delayed(now + min_delay);
	    planned = tx_ts;
	}

	// frame generation
        p = put_u64_5(p, planned);
        p = put_u8(p, FRAME_ANCHOR_REQ4TWR);
        p = put_u8(p, req_seq_no);
        p = put_u64_5(p, req_tx_ts);
        p = put_u64_5(p, req_rx_ts);

        int64_t esp_mid = esp_timer_get_time();

	uint8_t res = uwb_board_transmit(frame, frame_len, &tx_ts, delay_type);

        timestamp_t ts_end = uwb_board_get_systime();
        timestamp_t ts_diff = ts_end - now;
        uint32_t ts_diff_us = (uint32_t)(ts_diff / ticks_per_ns / 1000);
        uint32_t rx_now_us = (uint32_t)((now-req_rx_ts) / ticks_per_ns / 1000);
        int64_t esp_end = esp_timer_get_time();
        int64_t esp_diff_us = esp_end - esp_now;
        if ( res and tx_ts == planned) {
	    last_transmit_timestamp = tx_ts;
            monitor("TX anchor %s @%llx, diff: %lld, end: %lld, diff: %lld = %ld µs, t_esp: %lld µs, t_esp prepare %lld µs, rx-now: %ld µs, rx-now-esp: %lld µs rx_spin_esp: %lld, rx_handler_esp: %lld, anchor: %lld\n",
	        (other_anchor ? "rel" : "abs"), tx_ts, tx_ts-planned, ts_end, ts_diff, ts_diff_us,
	       	esp_diff_us, esp_mid - esp_now, rx_now_us, esp_now - uwb_board_get_last_isr_esp_ts(),
		esp_now - uwb_board_get_last_spin_rx_esp_ts(), esp_now - handler_rx_esp_ts, esp_now - anchor_esp_ts);
        } else {
	    last_transmit_timestamp = TIMESTAMP_NONE;
	    if (!res) tx_ts = ts_end;
            LOGE(TAG, "TX anchor %s @ %lld=%llx, planned was: %llx, diff: %lld, end: %lld, diff: %lld = %ld µs, esp_us: %lld",
	        (other_anchor ? "rel" : "abs"), tx_ts, tx_ts, planned, tx_ts-planned, ts_end, ts_diff, ts_diff_us, esp_diff_us);
        }

    } else if (frame_type == FRAME_TYPE_TEST) {
        frame_len = 2 + 5 + 1 + 5;
       
        uwb_board_prepare_transmit();
       
        int64_t esp_now = esp_timer_get_time();
	timestamp_t now = uwb_board_get_systime();
        
	// tx_ts planning, frame generation
	
	timestamp_t tx_ts;
	timestamp_t planned;
        uint8_t delay_type;
	timestamp_t req_rx_ts;

	// tx_ts planning
	if (other_anchor) {
            req_rx_ts = other_anchor->rx_timestamp[other_anchor->last_idx];
#if 0
            // using UWB_DELAY_RX_RELATIVE fails in 10%
	    static timestamp_t resp_delay = uwb_board_plan_delayed(min_delay << 5);
	    delay_type = UWB_DELAY_RX_RELATIVE;
	    planned = uwb_board_plan_delayed(last_rx + resp_delay);
	    tx_ts = (planned - last_rx) & TIMESTAMP_MASK;
	    planned = (planned + 62 * 256) & TIMESTAMP_MASK;
#else
	    timestamp_t resp_delay = uwb_board_plan_delayed(min_delay + uwb_board_timestamp_diff(now, req_rx_ts));
            delay_type = UWB_DELAY_ABSOLUTE;
	    tx_ts = uwb_board_plan_delayed(req_rx_ts + resp_delay);
	    planned = tx_ts;
#endif
	} else {
            req_rx_ts = 0;
            delay_type = UWB_DELAY_ABSOLUTE;
	    tx_ts = uwb_board_plan_delayed(now + min_delay);
	    planned = tx_ts;
	}

	// frame generation
        p = put_u64_5(p,planned);
        p = put_u8(p, FRAME_TEST_REQ4RESPONSE);
        p = put_u64_5(p, req_rx_ts);
     
	uint64_t esp_mid = esp_timer_get_time();
	
	uint8_t res = uwb_board_transmit(frame, frame_len, &tx_ts, delay_type);

        timestamp_t ts_end = uwb_board_get_systime();
        timestamp_t ts_diff = ts_end - now;
        uint32_t ts_diff_us = (uint32_t)(ts_diff / ticks_per_ns / 1000);
        uint32_t rx_now_us = (uint32_t)((now-req_rx_ts) / ticks_per_ns / 1000);
        int64_t esp_end = esp_timer_get_time();
        int64_t esp_diff_us = esp_end - esp_now;
        if ( res and tx_ts == planned) {
	    last_transmit_timestamp = tx_ts;
            monitor("TX test %s @%llx, diff: %lld, end: %lld, diff: %lld = %ld µs, t_esp: %lld µs, t_esp prepare %lld µs, rx-now: %ld µs, rx-now-esp: %lld µs rx_spin_esp: %lld, rx_handler_esp: %lld, anchor: %lld\n",
	        (other_anchor ? "rel" : "abs"), tx_ts, tx_ts-planned, ts_end, ts_diff, ts_diff_us,
	       	esp_diff_us, esp_mid - esp_now, rx_now_us, esp_now - uwb_board_get_last_isr_esp_ts(),
		esp_now - uwb_board_get_last_spin_rx_esp_ts(), esp_now - handler_rx_esp_ts, esp_now - anchor_esp_ts);
        } else {
	    last_transmit_timestamp = TIMESTAMP_NONE;
	    if (!res) tx_ts = ts_end;
            LOGE(TAG, "TX test %s @ %lld=%llx, planned was: %llx, diff: %lld, end: %lld, diff: %lld = %ld µs, esp_us: %lld",
	        (other_anchor ? "rel" : "abs"), tx_ts, tx_ts, planned, tx_ts-planned, ts_end, ts_diff, ts_diff_us, esp_diff_us);
        }
    
    } else if (frame_type == FRAME_TYPE_BLINK) {
        sprintf((char*)(frame+2), "GODESOFT");
	frame_len = 11; //strlen(frame+2);
        
        timestamp_t tx_ts;	
	uwb_board_transmit(frame, frame_len, &tx_ts);
        printf("TX blink frame send\n");
    }

}


static void transmit(TickType_t timeout = portMAX_DELAY, uint8_t frame_type = FRAME_TYPE_BLINK, anchor_data_t *other_anchor = nullptr)
{
    TickType_t t_end = (timeout == portMAX_DELAY) ? portMAX_DELAY : xTaskGetTickCount() + timeout;
    while (true) {
	transmit_one(frame_type, other_anchor);
	if (timeout < portMAX_DELAY && xTaskGetTickCount() >= t_end)
            break;
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 Hz
    }
}



static void receive(TickType_t timeout = portMAX_DELAY) {
    //LOGI(TAG, "receive calls uwb_board_receive_async");
    uwb_board_receive_async(&handle_received_frame, TRX_CB_TIMESTAMP|TRX_AUTO_RESTART);
    //LOGI(TAG, "receive calls uwb_board_spin(timeout: %ld)", timeout);
    uwb_board_spin(timeout);

#ifndef NDEBUG
    anchor_esp_ts = esp_timer_get_time();
#endif
}


//#define pdUS_TO_TICKS(us) ((TickType_t)(((uint64_t)(us) * configTICK_RATE_HZ) / 1000000ULL))
static void anchor() {
    uint16_t long_listen = 400 + (board_id >> 8); // use board_id to get different time slots

    uint16_t reply_delay     = 22'000; // [µs]
    uint16_t reply_delay_max = 40'000; // [µs]
    uint16_t reply_delay_min = 10'000; // [µs]

    while (true) {
	static uint16_t count = 0;
        static uint8_t state_init = true;

        int32_t listen_time = 200;

        if (state_init) {
 	    listen_time = long_listen;
 	    state_init = false;
        }

        //monitor("anchor: receive for %ld ms\n", listen_time);
        receive(pdMS_TO_TICKS(listen_time));
        if (send_response_to_last_frame) {

	    LOGD(TAG, "received a message so send a response");
	    do { // response loop, make performant to hold downtime short
                switch (last_received_frame_type) {
		    case FRAME_TYPE_TEST:   reply_delay += 2000; break;
		    case FRAME_TYPE_ANCHOR: reply_delay -=    1; break;
		}
		if (reply_delay < reply_delay_min) reply_delay = reply_delay_min;
		else if (reply_delay > reply_delay_max) reply_delay = reply_delay_max;

		send_response_to_last_frame = false;
            	vTaskDelay(pdMS_TO_TICKS(reply_delay/1000)); // wait for remote device listening
            	transmit_one(FRAME_TYPE_ANCHOR, &other_anchor);
                receive(pdMS_TO_TICKS(listen_time)); // listen as fast as possible after responding!
	    } while (send_response_to_last_frame);

        } else {
	    LOGW(TAG, "nothing received send test frame to check for other anchors (reply_delay: %d,[%d,%d])",
	        reply_delay, reply_delay_min, reply_delay_max);
            if (reply_delay_min + 100 < reply_delay_max) reply_delay_min += 10;
	    transmit_one(FRAME_TYPE_TEST, nullptr);
	    state_init = true;
        }
        
	if (!(count++ % 100)) {
	    uwb_board_sleep();
	    uwb_board_lowpower();
	    vTaskDelay(pdMS_TO_TICKS(5000));
	    uwb_board_wakeup();
        }
    }
}


#define sender_id 0x3480
#define receiver_id 0xdeb3

static void uwb_task(void* arg) {
    LOGI(TAG,"uwb_task started, initializing radio board.");

    const uwb_board_attributes_t *attrs = uwb_board_get_attributes();
    uwb_board_attributes_out(attrs);
    
    // initialize globals
    timestamp_frequency = attrs->timestamp_frequency;
    board_id = uwb_board_get_id();

    // initialize boards transsceiver
    uwb_board_set_spi_frequency(4'000'000UL); // 4.8 MHz seems to be fastest
    //uwb_board_set_spi_frequence(attrs->max_spi_frequency); // 4.8 MHz seems to be fastest
    uwb_board_init();

    if (board_id == sender_id) {
	LOGI(TAG, "Running as transmitter/anchor %x", board_id);
        uwb_board_wakeup();
        //transmit(portMAX_DELAY,FRAME_TYPE_TEST,nullptr);
	anchor();
    } else {
	LOGI(TAG, "Running as receiver/anchor %x", board_id);
        uwb_board_wakeup();
        //receive();
	anchor();
    }
    fatal_error("uwb_task end");
}



/* **********
 * Range Task
 * ********** */

#define RTAG "ranger"

#define UPDATE_EMA2(var, val, p, diff) if (abs((var)-(val)) < (diff)) (var) += (p) * (val-(var)) 
static void calculate_range(timestamp_t *ts, float drift) {
    /* T_round1 = Roundtrip-Zeit bei Gerät A
     * T_reply1 = Antwortverzögerung bei Gerät B
     * T_round2 = Roundtrip-Zeit bei Gerät B
     * T_reply2 = Antwortverzögerung bei Gerät A
     *
     * T_tof = ( (T_round1−T_reply1) + (T_round2−T_reply2) ) / 4
     *
     * more accurate handling drift:
     * T_tof = ( T_round1*T_round2 - T_reply1*T_reply2 ) / ( T_round1 + T_round2 + T_reply1 + T_reply2 )
     */
    
     int64_t T_round1 = uwb_board_timestamp_diff(ts[4], ts[1]);
     int64_t T_reply1 = uwb_board_timestamp_diff(ts[3], ts[2]);
     int64_t T_round2 = uwb_board_timestamp_diff(ts[6], ts[3]);
     int64_t T_reply2 = uwb_board_timestamp_diff(ts[5], ts[4]);

     if (T_round1 < T_reply1 || T_reply1 < 1 )
	 LOGW(RTAG, "T_round1=%lld < T_reply1=%lld || T_reply1 < 1", T_round1, T_reply1);
     if (T_round2 < T_reply2 || T_reply2 < 1 )
	 LOGW(RTAG, "T_round2=%lld < T_reply2=%lld || T_reply2 < 1", T_round2, T_reply2);

     int64_t tof_4 = ( T_round1 - T_reply1 ) + ( T_round2 - T_reply2 );

     double  tof_n = (double)T_round1 * (double)T_round2 - (double)T_reply1 * (double)T_reply2;
     int64_t tof_d = T_round1 + T_round2 + T_reply1 + T_reply2;

     // factors to multiply by speed of light and devide by frequency
     static double c_f   =        299792458.0 / timestamp_frequency; // [m/s], the distance the light takes in one timestamp tick
     static double c_f_4 = 0.25 * 299792458.0 / timestamp_frequency; // [m/s]

     double d1 = (double)tof_4 * c_f_4;
     double d2 = tof_n / tof_d * c_f;

     static double d1_ema = d1;
     static double d2_ema = d2;

     // dynamic starting ema
     static double ema_p = 0.02;
     static double diff = 1.00;
     static uint16_t count = 0;
     if (false && (++count <= 1000)) {
         ema_p = 10.0 / count;
         diff = 100.0 / count;
     }

     UPDATE_EMA2(d1_ema, d1, ema_p, diff);
     UPDATE_EMA2(d2_ema, d2, ema_p, diff);
	
     LOGI(RTAG, "cf: %.4f, d1: %7.4f, d2: %7.4f, ema: d1: %7.4f, d2: %7.4f", c_f, d1, d2, d1_ema, d2_ema);
}


static void ranging(uint16_t device_id) {
    // lookup device in anchor_table
    
    // pick device
    uint8_t consecutive;
    timestamp_t ts[6];
    float drift;

    // access global anchor_data table
    taskENTER_CRITICAL(&(other_anchor.mux));
    {	consecutive = other_anchor.consecutive;
	if (consecutive >= 2) {
	    uint8_t idx = other_anchor.last_idx;
            uint8_t idx0 = predessor_stamp_idx(idx);
	    // poll frame from remote device, tx remote, rx local
    	    ts[1] = other_anchor.tx_timestamp[idx0];
    	    ts[2] = other_anchor.rx_timestamp[idx0];
    	    
	    // response frame from this device, tx local, rx remote
    	    ts[3] = other_anchor.req_tx_ts[idx];
    	    ts[4] = other_anchor.req_rx_ts[idx];
	    
	    // final frame from remote device, tx remote, rx local
    	    ts[5] = other_anchor.tx_timestamp[idx];
	    ts[6] = other_anchor.rx_timestamp[idx];

    	    drift = other_anchor.drift;
	}
    }
    taskEXIT_CRITICAL(&(other_anchor.mux));

    if (consecutive < 2) {
        LOGW(RTAG,"less then 2 consecutive entries in anchor_data");
        return;
    }

    calculate_range(ts, drift);
}

static void range_task(void* arg) {
    LOGI(TAG,"range_task started");

    while (true) {
	uint16_t device_id;
	xQueueReceive(twr_queue, &device_id, portMAX_DELAY);
	// LOGI(RTAG, "got data of %x", device_id);
        ranging(device_id);
    }
    fatal_error("range_task end");
}


extern "C" void app_main() {
    printf("\nStart FreeRTOS implementation: %s\n", APP_NAME);
  
    // initialize inter task communication stuff
    twr_queue = xQueueCreate(8, sizeof(uint16_t)); // contains device_ids for twr to process

    // create tasks
    xTaskCreatePinnedToCore(
      uwb_task,    // Funktion für UWB-Empfang
      "uwb_trx",   // Name Task
      4096,        // Stack
      NULL,        // Parameter
      12,          // Priorität
      NULL,        // Task Handle
      0            // Core 0
    );
  
    xTaskCreatePinnedToCore(
      range_task,  // Funktion für UWB-Empfang
      "range",     // Name Task
      4096,        // Stack
      NULL,        // Parameter
      5,           // Priorität
      NULL,        // Task Handle
      0            // Core 0
    );
}

