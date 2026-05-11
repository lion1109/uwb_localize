#include "unity.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_task_wdt.h"

#include "debug.h"
#include "simple_trx_board.h"

static void test_init() {
    static bool initialized = false;
    if ( initialized ) return;
    initialized = true;

    board_init();	
    printf("board_id: %x\n", board_id);  
    printf("timestamp_frequency: %llu\n", timestamp_frequency);  
}

inline timediff_t drift_tics(timediff_t t, drift_t drift) {
    return t * drift / 1000'000'000LL;
}

inline timediff_t add_drift(timediff_t t, drift_t drift) {
    return t + drift_tics(t, drift);
}


static const char *TAG = nullptr;


static void perform_test(bool use_callback) {
    uint8_t frame[128];

    other_anchor.last_idx = 3;

    timestamp_t tics_per_ns = timestamp_frequency / 1000'000'000ULL;
    timestamp_t tics_per_m = timestamp_frequency / (uint64_t)RADIO_WAVE_SPEED;

    uint8_t idx_1 = successor_stamp_idx(other_anchor.last_idx);
    
    uint8_t remote_seq_no = 8;
    uint8_t local_seq_no = 128;

    drift_t drift_remote = 3000; // ppb; remote clock is 3 ppm faster 

    timediff_t tof = 1 * tics_per_m; // 1 meter distance 
    timediff_t tof_local = tof; 
    timediff_t tof_remote = add_drift(tof, drift_remote); // use drifted tof to calculate remote times
  
    timediff_t t_relay23_local  = 500 * tics_per_ns;
    timediff_t t_relay45_local  = 505 * tics_per_ns;
    timediff_t t_relay23_remote = add_drift(t_relay23_local, drift_remote);
    timediff_t t_relay45_remote = add_drift(t_relay45_local, drift_remote);

    timestamp_t local_1  = 0x00'FFFF'FFFF; // timestamp for local clock at t1
    timestamp_t remote_1 = 0xEE'FFFF'FFFF; // timestamp for remote clock at t1     
    timestamp_t t1 = remote_1 & TIMESTAMP_MASK; // remote clock
    timestamp_t t2 = (local_1 + tof_local) & TIMESTAMP_MASK; // local clock

    if (use_callback) {
        uint16_t frame_len = fill_anchor_frame(frame, remote_seq_no, t1, local_seq_no, TIMESTAMP_NONE, TIMESTAMP_NONE);
        handle_received_frame(frame, frame_len, t2);
    } else {
        update_anchor_by_rx_data(&other_anchor, idx_1, remote_seq_no, t1, t2, 0);
        other_anchor.req_tx_ts[idx_1] = 0ULL;
        other_anchor.req_rx_ts[idx_1] = 0ULL;
    }

    TEST_ASSERT_EQUAL(other_anchor.last_idx, idx_1);
    TEST_ASSERT_EQUAL(other_anchor.tx_timestamp[idx_1], t1);
    TEST_ASSERT_EQUAL(other_anchor.rx_timestamp[idx_1], t2);

    while (true) {
	// increment sequenc numbers before next round
	remote_seq_no++;
	local_seq_no++;
        
	timestamp_t t3 = (t2 + t_relay23_local) & TIMESTAMP_MASK; // local clock
        timestamp_t t4 = (t1 + tof_remote + t_relay23_remote + tof_remote) & TIMESTAMP_MASK; 
        timestamp_t t5 = (t4 + t_relay45_remote) & TIMESTAMP_MASK; 
        timestamp_t t6 = (t3 + tof_local + t_relay45_local + tof_local) & TIMESTAMP_MASK; 

        uint8_t idx_2 = successor_stamp_idx(idx_1);
        if (use_callback) {
	    // !handle_received_frame checks last_transmit_timestamp and next_transmit_seq_no
	    last_transmit_timestamp = t3;
	    next_transmit_seq_no = local_seq_no + 1;
            // LOGI(TAG, "idx: %d, seq_no: %d: tx_seq_no: %d, t3: %lld, t4: %lld, t5: %lld, t6: %lld", idx_2, remote_seq_no, local_seq_no, t3, t4, t5, t6);
            
	    uint16_t frame_len = fill_anchor_frame(frame, remote_seq_no, t5, local_seq_no, t3, t4);
            handle_received_frame(frame, frame_len, t6);
        } else {
            // LOGI(TAG, "idx: %d, seq_no: %d: t3: %lld, t4: %lld, t5: %lld, t6: %lld", idx_2, remote_seq_no, t3, t4, t5, t6);
	    update_anchor_by_rx_data(&other_anchor, idx_2, remote_seq_no, t5, t6, 1);
	    calculate_drift(&other_anchor);
            other_anchor.req_tx_ts[idx_2] = t3;
            other_anchor.req_rx_ts[idx_2] = t4;
	}

        TEST_ASSERT_EQUAL(other_anchor.last_idx, idx_2);
        TEST_ASSERT_EQUAL(other_anchor.tx_timestamp[idx_2], t5);
        TEST_ASSERT_EQUAL(other_anchor.rx_timestamp[idx_2], t6);
        TEST_ASSERT_EQUAL(other_anchor.req_tx_ts[idx_2], t3);
        TEST_ASSERT_EQUAL(other_anchor.req_rx_ts[idx_2], t4);

        ranging(other_anchor.device_id);
        TEST_ASSERT(0.990 < other_anchor.d1 && other_anchor.d1 < 1.01);
        TEST_ASSERT(0.990 < other_anchor.d2 && other_anchor.d2 < 1.01);

	// prepare for next round
	t1 = t5;
        t2 = t6;
	idx_1 = idx_2;
    }
}


TEST_CASE("update_other_anchor", "[update_other_anchor]") {
    TAG = "TEST update_other_anchor";

    test_init();
    perform_test(false);
}


TEST_CASE("read_callback", "[callback]") {
    TAG = "TEST read_callback";

    test_init();
    perform_test(true);
}


extern "C" void app_main(void) {
    esp_task_wdt_deinit();

    /*
    UNITY_BEGIN();
    RUN_TEST(test_callback);
    UNITY_END();
    */

    unity_run_menu();
}
