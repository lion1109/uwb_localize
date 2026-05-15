#include "build.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

//#include "dw3000.h"
#include "uwb_board.h"
#include "esp_timer.h"
#include "esp_random.h"

#include "simple_trx_board.h"


#define APP_NAME "SIMPLE TRX UWB BOARD v0.1"
#define TAG "simple_rx_board"

/* [1] The mathematics of two-way ranging,
 *     https://forum.qorvo.com/uploads/short-url/toATZSWYOalHWElTUi8x4nUkw0m.pdf
 */


static QueueHandle_t twr_queue = NULL; // contains device_id of twr to process


#if STATISTIC
#define SLIDING_MEAN 20 
static float drift_ema_p = 1.0/SLIDING_MEAN;
#define UPDATE_EMA(var, val) ((var) += drift_ema_p * ((val)-(var))) 
#endif


// initialize by board_attributes:
uint16_t board_id = 0;            // 16 bit network device address 
uint64_t timestamp_frequency = 0; // board timestamp frequency


STATIC anchor_data_t other_anchor = {
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

    .drift = 0.0,
    .offset = 0,
    .distance = 0.0,
    .distance_ema = 0.0,
#if STATISTIC
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


// transmit infos defined global to access in handle_received_frame:
STATIC uint8_t next_transmit_seq_no = 0;
STATIC uint64_t last_transmit_timestamp = TIMESTAMP_NONE;
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

#ifndef NDEBUG
uint64_t handler_rx_esp_ts = 0;
#endif


STATIC void handle_received_frame(const uint8_t *frame, uint16_t frame_len, timestamp_t rx_ts) {
#ifndef NDEBUG
    handler_rx_esp_ts = esp_timer_get_time();
#endif

    // LOGI(TAG, "handle_received_frame: %p, %d, %lld\n", frame, frame_len, rx_ts);
    if (frame) {
  	uint8_t frame_type = frame[0];
  	uint8_t seq_no = frame[1];

	if (FRAME_TYPE_ANCHOR == frame_type) {  
  	    timestamp_t tx_ts = get_u64_5(frame+2);
  	    uint8_t flags = get_u8(frame+7);
            
	    uint8_t idx = successor_stamp_idx(other_anchor.last_idx);
            bool consecutive = seq_no == (uint8_t)(other_anchor.last_seq_no + 1); // reading access for this task allowed
            LOGD(TAG, "consecutive: %d", consecutive);
	    
	    // get requesting frame's data
	    uint8_t req_seq_no = get_u8(frame+8);
	    timestamp_t req_tx_ts = get_u64_5(frame+9);
	    timestamp_t req_rx_ts = get_u64_5(frame+14);
  	    
            // update achor_data
    	    taskENTER_CRITICAL(&(other_anchor.mux));
	    {   update_anchor_by_rx_data(&other_anchor, idx, seq_no, tx_ts, rx_ts, consecutive);
	        if ((uint8_t)(req_seq_no + 1) == next_transmit_seq_no && req_tx_ts == last_transmit_timestamp) {
    		    // this frame is a response to my last transmitted frame
	       	    other_anchor.last_req_seq_no = req_seq_no;
	       	    other_anchor.req_tx_ts[idx] = req_tx_ts;
		    other_anchor.req_rx_ts[idx] = req_rx_ts;
	        } else {
	       	    other_anchor.last_req_seq_no = 0;
		    other_anchor.req_tx_ts[idx] = TIMESTAMP_NONE;
		    other_anchor.req_rx_ts[idx] = TIMESTAMP_NONE;
	        }
            }
    	    taskEXIT_CRITICAL(&(other_anchor.mux));
	    
	    //set_monitor_limit(); 
#if 0
            float scl_offset = (uwb_board_get_clock_freq_offset() / 1000000) / 5; // dw3000 meaning of drift is 5 times higher 
	    monitor("RX @ %llu anchor (%d): %d, %d, tx: %llu, req_tx: %llu, req_rx: %llu, flags:%x, %.4e, %.4e, %.4e, %.2f [Hz], %.4e, %.4e, cs:%d,%d\n",
	           rx_ts, frame_len, frame[0], seq_no, tx_ts, req_tx_ts, req_rx_ts, flags, 
		   other_anchor.drift, other_anchor.drift_ema, sqrt(other_anchor.drift_var_ema),
		   other_anchor.frequency, scl_offset, scl_offset-other_anchor.drift,
		   consecutive, other_anchor.consecutive);
#endif

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
            
	    update_anchor_by_rx_data(&other_anchor, idx, seq_no, tx_ts, rx_ts, consecutive);

	    set_monitor_limit(); 
#if STATISTIC
            //float ccl_offset = uwb_board_get_carrier_freq_offset();
            float scl_offset = (uwb_board_get_clock_freq_offset() / 1000000) / 5; 
	    printf("RX @ %lld test (%d): %d, %d, %lld, cmd:%d, %.4e, %.4e, %.4e, %.4f [Hz], %.4e, %.4e, cs:%d,%d\n",
	           rx_ts, frame_len, frame[0], seq_no, tx_ts, cmd, 
		   other_anchor.drift, other_anchor.drift_ema, sqrt(other_anchor.drift_var_ema),
		   other_anchor.frequency, scl_offset, other_anchor.drift - scl_offset, consecutive, other_anchor.consecutive);
#else
	    LOGI("RX @%lld test (%d): %d, %d, %lld, cmd:%d",
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


STATIC uint16_t fill_anchor_frame(
    uint8_t *frame, uint8_t seq_no, timestamp_t tx_ts,
    uint8_t req_seq_no, timestamp_t req_tx_ts, timestamp_t req_rx_ts ) {
    uint8_t *p = frame;
    p = put_u8(p, FRAME_TYPE_ANCHOR);
    p = put_u8(p, seq_no);

    p = put_u64_5(p, tx_ts);
    
    p = put_u8(p, FRAME_ANCHOR_REQ4TWR);
    p = put_u8(p, req_seq_no);
    p = put_u64_5(p, req_tx_ts);
    p = put_u64_5(p, req_rx_ts);
    return p - frame;
}

STATIC uint16_t fill_test_frame(uint8_t *frame, uint8_t seq_no, timestamp_t tx_ts, timestamp_t req_rx_ts ) {
    uint8_t *p = frame;
    p = put_u8(p, FRAME_TYPE_TEST);
    p = put_u8(p, seq_no);

    p = put_u64_5(p, tx_ts);
   
    p = put_u8(p, FRAME_TEST_REQ4RESPONSE);
    p = put_u64_5(p, req_rx_ts);
    return p - frame;
}

STATIC uint16_t fill_blink_frame(uint8_t *frame, uint8_t seq_no, const char * text) {
    uint8_t *p = frame;
    p = put_u8(p, FRAME_TYPE_BLINK);
    p = put_u8(p, seq_no);

    sprintf((char*)p, "%8s", text);
    return (p - frame) + strlen(text);
}
    
static void transmit_one(
    uint8_t frame_type = FRAME_TYPE_BLINK,
    anchor_data_t *anchor = nullptr,
    uwb_board_receive_cb_t rx_cb = nullptr )
{
    uint8_t frame[19];
    uint8_t seq_no = next_transmit_seq_no++;

    uint16_t frame_len;
    if (frame_type == FRAME_TYPE_ANCHOR) {
       
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
	if (anchor) {
	    uint8_t last_idx = anchor->last_idx;
            req_seq_no = anchor->last_seq_no;
            req_tx_ts = anchor->tx_timestamp[last_idx];
            req_rx_ts = anchor->rx_timestamp[last_idx];
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
            req_tx_ts = TIMESTAMP_NONE;
            req_rx_ts = TIMESTAMP_NONE;
            delay_type = UWB_DELAY_ABSOLUTE;
	    tx_ts = uwb_board_plan_delayed(now + min_delay);
	    planned = tx_ts;
	}

	// frame generation
	frame_len = fill_anchor_frame(frame, seq_no, planned, req_seq_no, req_tx_ts, req_rx_ts);
	//ASSERT(frame_len == 19);

	int64_t esp_mid = esp_timer_get_time();

        uint8_t rx_flags = rx_cb ? TRX_CB_TIMESTAMP|TRX_AUTO_RESTART : 0;
	bool res = uwb_board_transmit(frame, frame_len, &tx_ts, delay_type,
	 			      rx_cb, rx_flags);

        timestamp_t ts_end = uwb_board_get_systime();
        timestamp_t ts_diff = ts_end - now;
        uint32_t ts_diff_us = (uint32_t)(ts_diff / ticks_per_ns / 1000);
        uint32_t rx_now_us = (uint32_t)((now-req_rx_ts) / ticks_per_ns / 1000);
        int64_t esp_end = esp_timer_get_time();
        int64_t esp_diff_us = esp_end - esp_now;
        if ( res and tx_ts == planned) {
	    last_transmit_timestamp = tx_ts;
#if 0
	    monitor("TX anchor %s @%llu, diff: %lld, end: %lld, diff: %lld = %ld µs, req seq_no %d, tx %llu, rx %llu,  t_esp: %lld µs, t_esp prepare %lld µs, rx-now: %ld µs, rx-now-esp: %lld µs rx_spin_esp: %lld, rx_handler_esp: %lld, anchor: %lld\n",
	        (anchor ? "rel" : "abs"), tx_ts, tx_ts-planned, ts_end, ts_diff, ts_diff_us,
		req_seq_no, req_tx_ts, req_rx_ts, 
	       	esp_diff_us, esp_mid - esp_now, rx_now_us, esp_now - uwb_board_get_last_isr_esp_ts(),
		esp_now - uwb_board_get_last_spin_rx_esp_ts(), esp_now - handler_rx_esp_ts, esp_now - anchor_esp_ts);
#endif
	} else {
	    last_transmit_timestamp = TIMESTAMP_NONE;
	    if (!res) tx_ts = ts_end;
            LOGE(TAG, "TX anchor %s @ %lld=%llx, planned was: %llx, diff: %lld, end: %lld, diff: %lld = %ld µs, esp_us: %lld",
	        (anchor ? "rel" : "abs"), tx_ts, tx_ts, planned, tx_ts-planned, ts_end, ts_diff, ts_diff_us, esp_diff_us);
        }

    } else if (frame_type == FRAME_TYPE_TEST) {
       
        uwb_board_prepare_transmit();
       
        int64_t esp_now = esp_timer_get_time();
	timestamp_t now = uwb_board_get_systime();
        
	// tx_ts planning, frame generation
	
	timestamp_t tx_ts;
	timestamp_t planned;
        uint8_t delay_type;
	timestamp_t req_rx_ts;

	// tx_ts planning
	if (anchor) {
            req_rx_ts = anchor->rx_timestamp[anchor->last_idx];
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

	frame_len = fill_test_frame(frame, seq_no, planned, req_rx_ts);
     
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
	        (anchor ? "rel" : "abs"), tx_ts, tx_ts-planned, ts_end, ts_diff, ts_diff_us,
	       	esp_diff_us, esp_mid - esp_now, rx_now_us, esp_now - uwb_board_get_last_isr_esp_ts(),
		esp_now - uwb_board_get_last_spin_rx_esp_ts(), esp_now - handler_rx_esp_ts, esp_now - anchor_esp_ts);
        } else {
	    last_transmit_timestamp = TIMESTAMP_NONE;
	    if (!res) tx_ts = ts_end;
            LOGE(TAG, "TX test %s @ %lld=%llx, planned was: %llx, diff: %lld, end: %lld, diff: %lld = %ld µs, esp_us: %lld",
	        (anchor ? "rel" : "abs"), tx_ts, tx_ts, planned, tx_ts-planned, ts_end, ts_diff, ts_diff_us, esp_diff_us);
        }
    
    } else if (frame_type == FRAME_TYPE_BLINK) {
	frame_len = fill_blink_frame(frame, seq_no, "GODESOFT");
        
        timestamp_t tx_ts;
	uwb_board_transmit(frame, frame_len, &tx_ts);
        printf("TX blink frame send\n");
    }

}


static void transmit(TickType_t timeout = portMAX_DELAY, uint8_t frame_type = FRAME_TYPE_BLINK, anchor_data_t *other_anchor = nullptr) {
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


static void anchor() {
    while (true) {
	static uint16_t count = 0;
        static bool state_init = true;
        static bool listening = false;
        
	uint32_t listen_time = pdMS_TO_TICKS(200);
        if (state_init) {
 	    state_init = false;
            listen_time = pdMS_TO_TICKS(5000);
        }

	if (listening) {
	    listening = false;
            uwb_board_spin(listen_time);
	} else {
            receive(listen_time);
	}
	if (send_response_to_last_frame) {

	    LOGD(TAG, "received a message so send a response");
	    do { // response loop, make performant to hold downtime short
		send_response_to_last_frame = false;
            	vTaskDelay(pdMS_TO_TICKS(21)); // wait for remote listening
                
		// listen as fast as possible after responding!
		transmit_one(FRAME_TYPE_ANCHOR, &other_anchor,
			     &handle_received_frame);
    		uwb_board_spin(listen_time);
	        listening = true;
	    } while (send_response_to_last_frame);

        } else {
	    LOGW(TAG, "nothing received, send request to any remote");
	    state_init = true;
	    transmit_one(FRAME_TYPE_TEST, nullptr); // , &handle_received_frame);
        }
	taskYIELD();
    }
}

static void anchor_old() {
    //uint16_t long_listen = 400 + (board_id >> 8); // use board_id to get different time slots

    while (true) {
	static uint16_t count = 0;
        static bool state_init = true;
        static bool listening = false;

        int32_t listen_time = 200 + (esp_random() & 0x3F); // 6 bits random

        if (state_init) {
 	    listen_time = 2 * listen_time;
 	    state_init = false;
        }

        //monitor("anchor: receive for %ld ms\n", listen_time);
	if (listening) {
	    listening = false;
            uwb_board_spin(listen_time);
	} else {
            receive(pdMS_TO_TICKS(listen_time));
	}
	if (send_response_to_last_frame) {

	    LOGD(TAG, "received a message so send a response");
	    do { // response loop, make performant to hold downtime short
		send_response_to_last_frame = false;
            	vTaskDelay(pdMS_TO_TICKS(19)); // wait for remote listening
                
		// listen as fast as possible after responding!
		transmit_one(FRAME_TYPE_ANCHOR, &other_anchor,
			     &handle_received_frame);
    		uwb_board_spin(listen_time);
	        taskYIELD();
	    } while (send_response_to_last_frame);

        } else {
	    LOGW(TAG, "nothing received, send to any remote");
	    state_init = true;
	    // listening = true;
	    transmit_one(FRAME_TYPE_TEST, nullptr); // , &handle_received_frame);
	    taskYIELD();
        }
        
	if (!(count++ % 100)) {
	    uwb_board_sleep();
	    uwb_board_lowpower();
	    vTaskDelay(pdMS_TO_TICKS(5000));
	    uwb_board_wakeup();
        }
    }
}


STATIC void board_init() {
    LOGI(TAG,"initializing radio board");

    // initialize inter task communication stuff
    twr_queue = xQueueCreate(8, sizeof(uint16_t)); // contains device_ids for twr to process

    const uwb_board_attributes_t *attrs = uwb_board_get_attributes();
    uwb_board_attributes_out(attrs);
    
    // initialize globals
    timestamp_frequency = attrs->timestamp_frequency;
    board_id = uwb_board_get_id();

    // initialize boards transsceiver

    uwb_board_set_spi_frequency(12'000'000); // 4.8 MHz seems to be fastest
    //uwb_board_set_spi_frequence(attrs->max_spi_frequency); // 4.8 MHz seems to be fastest
/* spi-freq benchmark
 * [MHz]    [us]
 * 20	    error by uwb_board_init: WRONG device id
 * 16	    error by uwb_board_init: WRONG device id
 * 12	     849614
 * 10	     870815
 * 8	     947151 
 * 6	     967034
 * 4,8	    1019411
 * 4	    1042147 
 * */
    uwb_board_init();

    uwb_board_benchmark();
}


#define sender_id 0x3480
#define receiver_id 0xdeb3

static void uwb_task(void* arg) {
    LOGI(TAG,"uwb_task started");

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
inline float update_ema(float old, float val, float p, float diff) { float d = abs(old-val); return (d < diff) ? p * d : 0.0 ; } 
static void calculate_range(const timestamp_t *ts, uint8_t consecutive, anchor_data_t *anchor, uint8_t last_seq_no, uint8_t last_req_seq_no) {
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
 
     timediff_t T_round1 = uwb_board_timestamp_diff(ts[3], ts[0]); // t4 - t1
     timediff_t T_reply1 = uwb_board_timestamp_diff(ts[2], ts[1]); // t3 - t2
     timediff_t T_round2 = uwb_board_timestamp_diff(ts[5], ts[2]); // t6 - t3
     timediff_t T_reply2 = uwb_board_timestamp_diff(ts[4], ts[3]); // t5 - t4

     timediff_t total_remote = T_round1 + T_reply2; // equal to t5 - t1
     timediff_t total_local  = T_reply1 + T_round2; // equal to t6 - t2
     double drift_c = (double)(total_local - total_remote) / (double)total_local; // drift compensation = -drift
     
     // for check and alternate tof formula
     double T_round1_c = drift_c * T_round1 + T_round1;   
     double T_reply2_c = drift_c * T_reply2 + T_reply2;
     
     // intermediate results with drift compensation
     double T_delta1_c = T_round1_c - (double)T_reply1;
     double T_delta2_c = (double)T_round2 - (double)T_reply2_c;

     // check data consistence
     bool ignore = false;
     static uint8_t show_logs = 0;
     if (T_delta1_c < 0.0 || T_delta2_c < 0.0) {
	 ignore = true;
	 show_logs = 3;
     }
     if (show_logs) {
         show_logs--;
	 LOGW(RTAG, "last_idx: %d, t1, %lld, t2: %lld, t3: %lld, t4: %lld, t5: %lld, t6: %lld", anchor->last_idx, ts[0], ts[1], ts[2], ts[3], ts[4], ts[5]);
         LOGW(RTAG, "T_round1: %lld(%.1f), T_reply1: %lld, T_round2: %lld, T_reply2: %lld(%.1f), TOF1: %.1f, TOF2: %.1f",
		    T_round1, T_round1_c, T_reply1, T_round2, T_reply2, T_reply2_c, T_delta1_c/2, T_delta2_c/2);
         if (T_delta1_c < 0.0)
	     LOGW(RTAG, "T_round1_c=%.1f, T_reply1=%lld, T_round1_c - T_reply1: %.1f", T_round1_c, T_reply1, T_delta1_c);
         if (T_delta2_c < 0.0)
	     LOGW(RTAG, "T_round2_c=%lld, T_reply2=%.1f, T_round2 - T_reply2_c: %.1f", T_round2, T_reply2_c, T_delta2_c);
     }

     if (ignore) return;

     double  tof_n = (double)T_round1 * (double)T_round2 - (double)T_reply1 * (double)T_reply2;
     int64_t tof_d = T_round1 + T_round2 + T_reply1 + T_reply2;

     // distance the radio wave moves in one timestamp tick
     static float c_f = RADIO_WAVE_SPEED / timestamp_frequency; // [m/tic]
     static float c_f_4 = c_f / 4;

     float tof = tof_n / tof_d;
     float d1 = tof * c_f;
     float d2 = (T_delta1_c + T_delta2_c) * c_f_4; // alternate formula

     // for range ema:
     static float ema_p = 0.08;
     static float diff = 10.00;

     timestamp_t t6_remote = (timestamp_t)(drift_c * tof + tof + 0.5) + ts[4]; // t5 + drift compensated tof
     timediff_t offset = uwb_board_timestamp_diff(t6_remote, ts[5]); // t6_remote - t6

     // write results without radio/ranger mutex
     anchor->distance = d1;
     anchor->drift = -drift_c; // sign !
     anchor->offset = offset;

#if STATISTIC
     static float d2_ema;
     UPDATE_EMA2(anchor->distance_ema, d1, ema_p, diff);
     UPDATE_EMA2(d2_ema, d2, ema_p, diff);

     static float T_delta1_c_ema = 0.0;
     static float T_delta2_c_ema = 0.0;
     UPDATE_EMA2(T_delta1_c_ema, T_delta1_c, 0.01, 1.e9);
     UPDATE_EMA2(T_delta2_c_ema, T_delta2_c, 0.01, 1.e9);

     static float drift_ema = 0.0;
     static float drift_var_ema = 0.0;
     double drift_std = (anchor->drift - drift_ema);
     UPDATE_EMA(drift_ema, anchor->drift); 
     UPDATE_EMA(drift_var_ema, drift_std * drift_std); 

     float frame_freq = (float)timestamp_frequency / total_local;
     float r_reply21 = (T_reply2_c / T_reply1) - 1;
     static timediff_t old_offset = 0;
     LOGI(RTAG, "%3d %3d %3d d1:%4.3f,%6.3f, d2:%6.3f,%4.3f, "
		"drift:%.3e,%.3e, f:%.2f[Hz], Td12:%.2f,%.2f,%9.2e, o:%6lld, r:%.4e",
	        consecutive, last_seq_no, last_req_seq_no,
		d1, anchor->distance_ema, d2, d2_ema, 
                drift_ema, sqrt(drift_var_ema), frame_freq,
		T_delta1_c_ema, T_delta2_c_ema, (T_delta1_c_ema/T_delta2_c_ema)-1,
		offset-old_offset, r_reply21);
     old_offset = offset;
#else
     LOGI(RTAG, "d1: %6.3f, d2: %6.3f, d3: %6.3f, ema: d1: %6.3f, d2: %6.3f",
	        d1, d2, d3, anchor->d1_ema, anchor->d2_ema);
#endif
     if (!last_seq_no)
         LOGI(RTAG, "timestamp_frequency: %llu, c_f: %f, RADIO_WAVE_SPEED: %f, 1/f: %e",
		timestamp_frequency, c_f, RADIO_WAVE_SPEED, 1.0/timestamp_frequency);
}


STATIC void ranging(uint16_t device_id) {
    // lookup device in anchor_table
    
    // pick device
    uint8_t consecutive;
    timestamp_t ts[6];
    uint8_t last_seq_no; // for debugging
    uint8_t last_req_seq_no; // for debugging

    // access global anchor_data table
    taskENTER_CRITICAL(&(other_anchor.mux));
    {	consecutive = other_anchor.consecutive;
	if (consecutive >= 2) {
	    uint8_t idx = other_anchor.last_idx;
            uint8_t idx0 = predessor_stamp_idx(idx);
	    // poll frame from remote device, tx remote, rx local
    	    ts[0] = other_anchor.tx_timestamp[idx0]; // t1 
    	    ts[1] = other_anchor.rx_timestamp[idx0]; // t2
    	    
	    // response frame from this device, tx local, rx remote
    	    ts[2] = other_anchor.req_tx_ts[idx]; // t3
    	    ts[3] = other_anchor.req_rx_ts[idx]; // t4
	    
	    // final frame from remote device, tx remote, rx local
    	    ts[4] = other_anchor.tx_timestamp[idx]; // t5
	    ts[5] = other_anchor.rx_timestamp[idx]; // t6

    	    last_seq_no = other_anchor.last_seq_no;
    	    last_req_seq_no = other_anchor.last_req_seq_no;
	}
    }
    taskEXIT_CRITICAL(&(other_anchor.mux));

    if (consecutive < 2) {
        LOGW(RTAG,"less then 2 consecutive entries in anchor_data");
        return;
    }

    calculate_range(ts, consecutive, &other_anchor, last_seq_no, last_req_seq_no);
}

static void range_task(void* arg) {
    LOGI(TAG,"range_task started");

    while (true) {
	uint16_t device_id;
	xQueueReceive(twr_queue, &device_id, portMAX_DELAY);
	// LOGI(RTAG, "got data of %x", device_id);
        ranging(device_id);
	taskYIELD();
    }
    fatal_error("range_task end");
}


#ifndef UNIT_TEST
extern "C" void app_main() {
    printf("\nStart FreeRTOS implementation: %s\n", APP_NAME);
  
    // initialize inter task communication and radio board
    board_init();

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
      1            // Core 0
    );
}
#endif

