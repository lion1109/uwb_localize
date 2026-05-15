#include "build.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>

#include "driver/gpio.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>

#include <dw3000_device_api.h>
#include <dw3000.h>
//#include <dw3000_regs.h>

#include "uwb_board.h"

/* [1] DW3xxx_Driver_API_Guide_6.0.14.pdf 
 * [2] 
 * */

static const char * const TAG = "uwb_board";

/* additional convenient functions */
void fatal_error(const char* error) {
    LOGE(TAG, "fatal error: %s\r\n", error);
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}



static void print_partitions() {
    printf("\n--- Partitions ---\n");

    esp_partition_iterator_t it = esp_partition_find(
        ESP_PARTITION_TYPE_ANY,
        ESP_PARTITION_SUBTYPE_ANY,
        NULL
    );

    while (it != NULL)
    {
        const esp_partition_t *part = esp_partition_get(it);

        printf("Name: %-10s Type: %d Sub: %d Addr: 0x%x Size: %d\n",
               part->label,
               part->type,
               part->subtype,
               (unsigned int)part->address,
               (int)part->size);

        it = esp_partition_next(it);
    }

    esp_partition_iterator_release(it);

    printf("------------------\n");
}


static void print_all_nvs() { printf("\nNVS Variables:\n");
    nvs_flash_init();

    nvs_iterator_t it = NULL;

    esp_err_t err = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);

    if (err != ESP_OK) {
        printf("nvs_entry_find failed\n\n");
        return;
    }
    
    if (it == NULL) {
        printf("--- no entry ---\n\n");
        return;
    }

    while (it != NULL) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        printf("%s, %s, Type %d: ",
               info.namespace_name,
               info.key,
               info.type);

	        nvs_handle_t h;
        nvs_open(info.namespace_name, NVS_READONLY, &h);

        switch (info.type) {
            case NVS_TYPE_U8: {
                uint8_t v;
                nvs_get_u8(h, info.key, &v);
                printf("%u", v);
                break; }

            case NVS_TYPE_I8: {
                int8_t v;
                nvs_get_i8(h, info.key, &v);
                printf("%d", v);
                break; }

            case NVS_TYPE_U16: {
                uint16_t v;
                nvs_get_u16(h, info.key, &v);
                printf("%u", v);
                break; }

            case NVS_TYPE_I16: {
                int16_t v;
                nvs_get_i16(h, info.key, &v);
                printf("%d", v);
                break; }

            case NVS_TYPE_U32: {
                uint32_t v;
                nvs_get_u32(h, info.key, &v);
                printf("%lu", v);
                break; }

            case NVS_TYPE_I32: {
                int32_t v;
                nvs_get_i32(h, info.key, &v);
                printf("%ld", v);
                break; }

            case NVS_TYPE_U64: {
                uint64_t v;
                nvs_get_u64(h, info.key, &v);
                printf("%llu", v);
                break; }

            case NVS_TYPE_I64: {
                int64_t v;
                nvs_get_i64(h, info.key, &v);
                printf("%lld", v);
                break; }

            case NVS_TYPE_STR: {
                size_t len = 0;
                nvs_get_str(h, info.key, NULL, &len);
                char* buf = (char*)malloc(len);
                nvs_get_str(h, info.key, buf, &len);
                printf("%s", buf);
                free(buf);
                break; }

            case NVS_TYPE_BLOB: {
                size_t len = 0;
                nvs_get_blob(h, info.key, NULL, &len);
                printf("<blob %d bytes>", (int)len);
                break; }

            default:
                printf("<unsupported type>");
                break;
        }
        nvs_close(h);

        printf("\n");

        nvs_entry_next(&it);
    }

    nvs_release_iterator(it);

    printf("\n");
}


static const char *NVS_NS = "uwb_board";

static void test_nvs() {
    nvs_handle_t handle;

    nvs_open(NVS_NS, NVS_READWRITE, &handle);
    nvs_set_u32(handle, "spi_frequency", 20'000'000);
    nvs_commit(handle);
    nvs_close(handle);
}

static void test_nvs2() {
    nvs_handle_t handle;

    nvs_open(NVS_NS, NVS_READWRITE, &handle);

    nvs_erase_key(handle, "spi_speed");
    nvs_commit(handle);
    nvs_close(handle);
}


/* This module abstracts an uwb transceiver board
 */

// raising the tx + rx antenna delay sum, lowers the distance measured
#define ad_mid 16380 // makerfabs value: 16436
#define ad_dif   300 // default? 500
/* ad_dif	real distance	calculated distance
 * 1500		1.38		1.68
 * 1500		1.00		1.41
 * 1000         1.38            1.53
 * 1000         1.00            1.41
 * 1250         1.38            1.63
 * 1250         1.00            1.38
 *  500		1.38		1.
 *  500		1.00		1.33
 *    0		1.00		1.30
 */
static uwb_board_attributes_t makerfabs_dw3000_esp32 = {
  .name                = "makerfabs_dw3000_esp32",
  .timestamp_bits      = 40,
  .crystal_frequency   = 38'400'000ULL,
  .pll_factor          = 13 * 128ULL,
  .timestamp_frequency = 38'400'000ULL * 13 * 128,

  // all following values have to be calibrated:
  .max_spi_frequency   = 8'000'000ULL,
  .min_delay_us        = 540, // 540 // 533 // 1250, 465
  .tx_antenna_delay    = ad_mid - ad_dif, // 16436 
  .rx_antenna_delay    = ad_mid + ad_dif
};

void saveValue()
{
    nvs_handle_t handle;

    nvs_open("config", NVS_READWRITE, &handle);

    nvs_set_i32(handle, "tx_delay", 16434);

    nvs_commit(handle);   // sehr wichtig!

    nvs_close(handle);
}

const uwb_board_attributes_t *uwb_board_get_attributes() {
    uwb_board_attributes_t *attrs = &makerfabs_dw3000_esp32;
    // optimizations for faster boards and calibration
#define ad_delta_tx -0
#define ad_delta_rx +0 // the higher, the 
    if (0x3480 == uwb_board_get_id()) {
        // attrs->max_spi_frequency = 40'000'000ULL;
        attrs->tx_antenna_delay += ad_delta_tx;
        attrs->rx_antenna_delay -= ad_delta_rx;
    } else if (0xdeb3 == uwb_board_get_id()) {
        // attrs->min_delay_us = 540; // 465;
        attrs->tx_antenna_delay -= ad_delta_tx;
        attrs->rx_antenna_delay += ad_delta_rx;
    }
    return attrs;
}

void uwb_board_attributes_out(const uwb_board_attributes_t *attrs) {
    printf("board name: %s\n", attrs->name);
    printf("board timestamp_bits: %d\n", attrs->timestamp_bits);
    printf("board crystal_frequency: %lld\n", attrs->crystal_frequency);
    printf("board pll_factor: %ld\n", attrs->pll_factor);
    printf("board timestamp_frequency: %lld\n", attrs->timestamp_frequency);
    printf("board tx_antenna_delay: %ld\n", attrs->tx_antenna_delay);
    printf("board rx_antenna_delay: %ld\n", attrs->rx_antenna_delay);
    printf("board max_spi_frequency: %lld\n", attrs->max_spi_frequency);
    printf("board min_delay_us: %d\n", attrs->min_delay_us);

    printf("board id: 0x%x\n", uwb_board_get_id());
    printf("ticks per ns: %ld\n", uwb_board_get_ticks_per_ns());
    printf("min delay: %lld\n", uwb_board_get_tx_min_delay());
}

/* implementation for makerfabs dw3000 esp32 board using FreeRTOS port of dw3000 arduino software */

#define DWT_INT_RXFCG_BIT_MASK SYS_STATUS_RXFCG_BIT_MASK
#define DWT_INT_TXFRS_BIT_MASK SYS_STATUS_TXFRS_BIT_MASK

// spi interface gpio pins: reset, irq, chip_select
#define PIN_RST (gpio_num_t)27
#define PIN_IRQ (gpio_num_t)34
#define PIN_CS  (gpio_num_t)4


uint16_t uwb_board_get_id(uint16_t start)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return esp_crc16_le(start, mac, 6);
}


/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t board_rx_buffer[FRAME_LEN_MAX];

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16_t board_frame_len;



// -------- ISR --------
static QueueHandle_t irq_queue = NULL;
enum board_event_t {
    EVT_IRQ = 1,
    EVT_ABORT = 2
};
#ifndef NDEBUG
static uint64_t isr_esp_ts = 0;
uint64_t uwb_board_get_last_isr_esp_ts() { return isr_esp_ts; }
#endif
static void IRAM_ATTR board_isr_handler(void* arg)
{
    board_event_t evt = EVT_IRQ;
    BaseType_t hp_task_woken = pdFALSE;

    xQueueSendFromISR(irq_queue, &evt, &hp_task_woken);
#ifndef NDEBUG
    isr_esp_ts = esp_timer_get_time();
#endif

    if (hp_task_woken) {
        portYIELD_FROM_ISR();
    }
}


uint8_t isr_started = false;
void check_start_isr()
{
  LOGD(TAG, "check_start_isr");
  if (isr_started) return;
  isr_started = true;

  // configure irq pin and install isr
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << PIN_IRQ),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE
  };
  gpio_config(&io_conf);
  LOGD(TAG, "gpio irq pin initialized");

  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  gpio_isr_handler_add(PIN_IRQ, board_isr_handler, NULL);
  LOGD(TAG, "gpio irq pin isr handler set");
}


/* switch DW3000 polling <-> irq */
#define DWT_INT_RXFCG_BIT_MASK  SYS_STATUS_RXFCG_BIT_MASK
#define DWT_INT_RXFCE_BIT_MASK  DWT_INT_RFCE
#define DWT_INT_RXRFTO_BIT_MASK DWT_INT_RFTO
#define DWT_INT_RXPTO_BIT_MASK  DWT_INT_RXPTO

static uint8_t mode_polling = true;
static void switch_to_polling()
{
    //LOGD(TAG, "switch to polling");
    
    dwt_forcetrxoff();   // important after irq
    
    if (!mode_polling) {
        dwt_setinterrupt(0, 0xFFFFFFFF, DWT_ENABLE_INT_ONLY); // switch of all IRQs
        mode_polling = true;
    }
    //LOGD(TAG, "dwt clear status register");
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF); // clear status register

    xQueueReset(irq_queue); // drop pending events
}
static void switch_to_irq()
{
    LOGD(TAG, "switch to irq");

    dwt_forcetrxoff();
    LOGD(TAG, "DW3000 dwt_forcetrxoff done");
    
    xQueueReset(irq_queue); // drop pending events
    
    if (!mode_polling) return;

    vTaskDelay(pdMS_TO_TICKS(1)); // oder taskYIELD()
  
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF); // clear pending interrupts
    LOGD(TAG, "DW3000 status register cleared");

    //vTaskDelay(pdMS_TO_TICKS(200));
    LOGD(TAG, "DW3000 dwt_setinterrupt\n");
    //dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR, 0, DWT_ENABLE_INT); // aktivate IRQ
    dwt_setinterrupt(
        DWT_INT_RXFCG_BIT_MASK |
        DWT_INT_RXFCE_BIT_MASK |
        DWT_INT_RXRFTO_BIT_MASK |
        DWT_INT_RXPTO_BIT_MASK |
        DWT_INT_TXFRS_BIT_MASK,
        0,
        DWT_ENABLE_INT_ONLY
    );
    LOGD(TAG, "DW3000 dwt_setinterrupt done");

    check_start_isr();

    mode_polling = false;
}


void uwb_board_set_spi_frequency(const uint32_t freq)
{
    // SPI rate, DW3000 supports up to 38 MHz, 12 MHz verified
    spiFastFrequency(freq);
}

static uint64_t check_spi_frequency(uint64_t freq) {
    spiFastFrequency(freq);
    int8_t i;
    for (i = 5; i>0; i--)
        dwt_read32bitreg(SYS_STATUS_ID);
    static uint64_t t0 = esp_timer_get_time();
    for (i = 100; i>0; i--) 
        dwt_read32bitreg(SYS_STATUS_ID);
    return esp_timer_get_time() - t0;
}

static const uwb_board_attributes_t *board_attrs = uwb_board_get_attributes();

void uwb_board_init() {
    LOGD(TAG, "uwb_board_init()");

    print_partitions();
    print_all_nvs();
    test_nvs();

    /* checks */
    if (board_attrs->timestamp_bits > 8 * TIMESTAMP_BYTES) fatal_error("uwb board config error: timestamp_bits do not fit into TIMESTAMP_BYTES");

    // Default communication configuration. We use default non-STS DW mode. 
    dwt_config_t config = {
      5,                // channel number
      DWT_PLEN_128,     // preamble length, used in TX only
      DWT_PAC8,         // preamble acquisition chunk size. used in RX only
      9,                // TX preamble code, used in TX only
      9,                // RX preamble code, used in RX only
      1,                // 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type 
      DWT_BR_6M8,       // Data rate
      DWT_PHRMODE_STD,  // PHY header mode
      DWT_PHRRATE_STD,  // PHY header rate
      (129 + 8 - 8),    // SFD timeout (preamble length + 1 + SFD length - PAC size), used in RX only
      DWT_STS_MODE_OFF, // STS disabled
      DWT_STS_LEN_64,   // STS length see allowed values in Enum dwt_sts_lengths_e 
      DWT_PDOA_M0       // PDOA mode off
    };

    dwt_txconfig_t txconfig = { // has to match chanal and frequency of config above
      .PGdly = 0x34,            // recommended PGdly, see DW3xxx_Driver_API_Guide_6.0.14.pdf 5.2.4
      //.power = 0x1F1F1F1F     // less power, just 1m los
      .power = 0xFDFDFDFD       // recommended TX power, chanal 5: 0xfdfdfdfd, chanal 9: 0xfefefefe
      //.PGcount = ?            // Pulse generator count value
    };

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_CS);
    vTaskDelay(pdMS_TO_TICKS(200)); // transition time from INIT_RC to IDLE_RC, or wait for SPIRDY event

    if (!dwt_checkidlerc()) fatal_error("IDLE FAILED"); // asure DW IC is in IDLE_RC

    dwt_softreset();
    vTaskDelay(pdMS_TO_TICKS(200));

    if (dwt_check_dev_id() == DWT_ERROR) fatal_error("WRONG device id");
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) fatal_error("INIT FAILED");

    // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) fatal_error("CONFIG FAILED"); // DWT_ERROR if either the PLL or RX calibration has failed the host should reset the device

    dwt_configuretxrf(&txconfig);

    dwt_enableautoack(0,0); // disable auto ACK

    dwt_setrxantennadelay(board_attrs->rx_antenna_delay);
    dwt_settxantennadelay(board_attrs->tx_antenna_delay);

    irq_queue = xQueueCreate(10, sizeof(uint32_t));
    LOGD(TAG, "irq_queue created");

    LOGD(TAG, "uwb_board_init() initialize and configure successfully done");

#define LED_PIN (gpio_num_t)2
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(LED_PIN, 1);

//#define SPI_AUTO_FREQ 1
#ifdef SPI_AUTO_FREQ
    uint64_t freq_min = 1'800'000ULL;
    uint64_t d_freq = 20'000ULL;
    uint64_t time_min = check_spi_frequency(freq_min);
    printf("check spi frequency: %llu [Hz], time %llu\n", freq_min, time_min);
    while (true) {
        uint64_t freq_check = freq_min + d_freq; 
        uint64_t time_check = check_spi_frequency(freq_check);
	printf("check spi frequency: %llu [Hz], time %llu\n", freq_check, time_check);
	if (time_check > time_min) break;
        freq_min = freq_check;
	time_min = time_check;
    }
    spiFastFrequency(freq_min);
#endif

#define DWT_TIMEOUT_MAX 0xFFFFF // nearly 1 second [1]
    dwt_setrxaftertxdelay(0); // wait 0 µs after transmit before going to rx mode
    dwt_setrxtimeout(DWT_TIMEOUT_MAX);
}


uint32_t uwb_board_benchmark() {
    
    uint32_t t_benchmark = dw3000_benchmark();

    return t_benchmark;
}


void uwb_board_wakeup() {
    uwb_board_transmit(nullptr,0,nullptr); // transmit empty frame to wake up
    //uwb_board_receive_async(nullptr, 0); // switch receiver on 
    //uwb_board_spin_once(pdMS_TO_TICKS(1000)); // for just one ms
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    //vTaskDelay(pdMS_TO_TICKS(1)); // wait until frame send
}
void uwb_board_sleep() {
    switch_to_polling();

}
void uwb_board_lowpower() {
    dwt_setleds(DWT_LEDS_DISABLE | DWT_LEDS_INIT_BLINK);
    gpio_set_level(LED_PIN, 0);
}

static uint16_t board_pan_id = 0x0000; // this boards pan id
static uint16_t board_dev_id = 0x0000; // this boards device address

void uwb_board_set_pan_shortaddress(uint16_t pan_id, uint16_t board_addr)
{
    board_pan_id = pan_id;
    board_dev_id = board_addr;
    dwt_setpanid(pan_id);
    dwt_setaddress16(board_addr);
}


timestamp_t uwb_board_get_systime()
{
    uint32_t ts = 0;
    dwt_write32bitreg(SYS_STATUS_ID, 0); // disable latch!
    dwt_readsystime((uint8_t*)&ts);
    return ((uint64_t)ts) << 8;
}


static uint8_t *read_received_frame(uint16_t *length_ptr)
{
    LOGD(TAG, "read_received_frame polling");
 
    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) fatal_error("read_received_frame without SYS_STATUS_RXFCG_BIT_MASK");
 
    LOGD(TAG, "read_received_frame polling done");

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        /* Clear local RX buffer to avoid having leftovers from previous receptions.
         * This is not necessary but is included here to aid reading the RX buffer.
         */
        memset(board_rx_buffer, 0, sizeof(board_rx_buffer));

        /* A frame has been received, copy it to our local buffer. */
        board_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
        if (board_frame_len <= FRAME_LEN_MAX)
        {
          dwt_readrxdata(board_rx_buffer, board_frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */
        }

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        LOGD(TAG, "Frame Received: length: %d", board_frame_len);
        if (length_ptr) *length_ptr = board_frame_len;
        return board_rx_buffer;
    } else {
        /* Clear RX error events in the DW IC status register. */
        LOGE(TAG, "Frame Received error");
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        LOGE(TAG, "status_reg: %x", (uint16_t)status_reg);
        fatal_error("Frame Rec Error");
        return NULL;
    }
}


/* synchronization stuff */
float uwb_board_get_clock_freq_offset() { // after rx, system clock frequency difference [ppm]
    static float clockoffset_to_ppm = ((float)10e6) / (float)(2ULL << 26);
    // after rx, return system freq difference
    int16_t co = dwt_readclockoffset();
    return clockoffset_to_ppm * co;
}

	
float uwb_board_get_carrier_freq_offset() { // after rx, device freq difference [Hz]
    int32_t ci = dwt_readcarrierintegrator() ; // Read carrier integrator value
    // at 6.81Mb/s data rate convert to clock offset in Hz.
    float clockOffsetHertz = ci * FREQ_OFFSET_MULTIPLIER;

    // On channel 5 convert this to clock offset in PPM.
    // float clockOffsetPPM = clockOffsetHertz * and HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
    return clockOffsetHertz;
}


/* sync stuff */

uint32_t uwb_board_get_ticks_per_ns() {
    return board_attrs->timestamp_frequency / 1000000000ULL;
}

timestamp_t uwb_board_get_tx_min_delay() { // min number of ticks delay before uwb_board_transmit_delayed()
    return 1'000ULL * (uint64_t)board_attrs->min_delay_us * (uint64_t)uwb_board_get_ticks_per_ns();
}

timestamp_t uwb_board_plan_delayed(timestamp_t tx_ts) { // uwb_board_transmit_delayed just can use timestamp values returned by this function 
    //turn (tx_ts & 0xFFFFFFFE00) + board_attrs->tx_antenna_delay + 512; // round up so lsb is set
    return ((tx_ts & 0xFFFFFFFC00) + board_attrs->tx_antenna_delay + 1536) & TIMESTAMP_MASK; // round up so lsb is set
}


// spin ###############################################################

// data for receiving
static uwb_board_receive_cb_t active_rx_cb;
static uint8_t active_rx_flags;

void uwb_board_spin_abort() {
    uint32_t evt = EVT_ABORT;
    xQueueSend(irq_queue, &evt, 0);
}


enum spin_result_t {
    SPIN_TIMEOUT,
    SPIN_ABORTED,
    SPIN_RX,
    SPIN_TX,
    SPIN_IDLE
};

#ifndef NDEBUG
uint64_t spin_rx_esp_ts = 0;
uint64_t uwb_board_get_last_spin_rx_esp_ts() { return spin_rx_esp_ts; }
#endif
static spin_result_t spin_once(TickType_t timeout)
{
    LOGD(TAG, "spin_once timeout: %ld", timeout);
    
    board_event_t evt;
    if (!xQueueReceive(irq_queue, &evt, timeout)) {
        LOGD(TAG, "Timeout");
        return SPIN_TIMEOUT;
    }
    LOGD(TAG, "spin_once event: %d", evt);

    if (evt == EVT_ABORT) return SPIN_ABORTED;

#ifndef NDEBUG
    spin_rx_esp_ts = esp_timer_get_time();    
#endif

    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

    if (status & SYS_STATUS_TXFRS_BIT_MASK) { // TRANSMIT success
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        return SPIN_TX;

    } else if (status & SYS_STATUS_RXFCG_BIT_MASK) { // RECEIVE success
        LOGD(TAG, "Status RXFCG");
        uint16_t len = dwt_read16bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;

        if (len > sizeof(board_rx_buffer)) {
            LOGE(TAG, "Frame received length %d bytes larger than frame buffer", len);
            fatal_error("Frame received length larger than frame buffer");
	} else {
            dwt_readrxdata(board_rx_buffer, len, 0);
            LOGD(TAG, "Frame received: %d bytes", len);
        }

        timestamp_t rx_ts;
        if (active_rx_flags & TRX_CB_TIMESTAMP) {
           dwt_readrxtimestamp((uint8_t*)&rx_ts);
           rx_ts &= TIMESTAMP_MASK;
        } else {
           rx_ts = TIMESTAMP_NONE;
        }

	//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
	
	// restart as early as possible !!!
	if (active_rx_flags & TRX_AUTO_RESTART) dwt_rxenable(DWT_START_RX_IMMEDIATE);

	if (active_rx_cb)
	    active_rx_cb(board_rx_buffer, len, rx_ts);

        return SPIN_RX;
    }
   
    // unknown state my be after uwb_board_spin_abort() ???
    if (status == (SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFR_BIT_MASK |
		   SYS_STATUS_RXPHD_BIT_MASK | SYS_STATUS_CIADONE_BIT_MASK | SYS_STATUS_RXSFDD_BIT_MASK | SYS_STATUS_RXPRD_BIT_MASK |
		   SYS_STATUS_IRQS_BIT_MASK)) {
        return SPIN_RX; // continue spinning
    } 

    // ERROR
    if (status & SYS_STATUS_ALL_RX_ERR) {
        LOGE(TAG, "Status ERROR (not RXFCG?) 0x%lx", status);
        if (SYS_STATUS_RXFCE_BIT_MASK & status) {
    	    LOGW(TAG, "STATUS RXFCE frame checksum error"); status &= ~SYS_STATUS_RXFCE_BIT_MASK; }
        if (SYS_STATUS_RXFR_BIT_MASK & status) {
    	    LOGW(TAG, "STATUS RXFR  frame"); status &= ~SYS_STATUS_RXFR_BIT_MASK; }
        if (SYS_STATUS_RXPHD_BIT_MASK & status) {
       	    LOGW(TAG, "STATUS RXPHD frame"); status &= ~SYS_STATUS_RXPHD_BIT_MASK; }
        if (SYS_STATUS_CIADONE_BIT_MASK & status) {
    	    LOGW(TAG, "STATUS CIADONE frame"); status &= ~SYS_STATUS_CIADONE_BIT_MASK; }
        if (SYS_STATUS_RXSFDD_BIT_MASK & status) {
    	    LOGW(TAG, "STATUS RXSFDD frame"); status &= ~SYS_STATUS_RXSFDD_BIT_MASK; }
        if (SYS_STATUS_RXPRD_BIT_MASK & status) {
    	    LOGW(TAG, "STATUS RXPRD frame"); status &= ~SYS_STATUS_RXPRD_BIT_MASK; }
        if (SYS_STATUS_IRQS_BIT_MASK & status) {
    	    LOGW(TAG, "STATUS IRQS frame"); status &= ~SYS_STATUS_IRQS_BIT_MASK; }
        LOGW(TAG, "other status bits 0x%lx", status);
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); // clear error
        
	if (active_rx_cb)
	    active_rx_cb(nullptr, 0, TIMESTAMP_NONE);

	if (active_rx_flags & TRX_AUTO_RESTART) dwt_rxenable(DWT_START_RX_IMMEDIATE);

	return SPIN_RX;
    }

    if (status & SYS_STATUS_RXFTO_BIT_MASK) {
	LOGW(TAG, "spin_once: unhandled RXFTO irq (status: %lx)", status);
    }
	
    LOGE(TAG, "spin_once: unhandled SPIN_IDLE state! (status: %lx)", status);
    //fatal_error("spin_once: unhandled SPIN_IDLE state!");
    return SPIN_IDLE; // may be an ERROR, TODO check
}


void uwb_board_spin_once(TickType_t timeout) {
    // xSemaphoreTake(lock, portMAX_DELAY);
    LOGD(TAG, "uwb_board_spin_once call spin_once(%ld)", timeout);
    if (SPIN_ABORTED == spin_once(timeout)) return;
}


void uwb_board_spin(TickType_t timeout) {
    TickType_t end_time = (timeout==portMAX_DELAY) ? portMAX_DELAY : xTaskGetTickCount() + timeout;
    while (true) {
        TickType_t t_delay;
	if (timeout==portMAX_DELAY)
	{
	    t_delay = portMAX_DELAY;
	} else {
	    TickType_t now = xTaskGetTickCount();
	    if (now >= end_time) return;
	    t_delay = end_time - now;
	}
	LOGD(TAG, "uwb_board_spin call spin_once(%ld)", t_delay);
	uint8_t res = spin_once(t_delay);
	LOGD(TAG, "uwb_board_spin spin_once returned %d", res);

#if 0
	// faster ABORT
 	uint32_t evt = 0;
	if (xQueuePeek(irq_queue, &evt, 0) == pdTRUE) {
            //printf("Next event is %d\n", evt);
	    if (evt == EVT_ABORT) {
		xQueueReceive(irq_queue, &evt, 0);
	        LOGD(TAG, "spin_once aborted, so break loop (fast)");
		break;
	    }
        }
#endif
	
	if (SPIN_ABORTED == res || SPIN_TIMEOUT == res)
	{
	    if (SPIN_TIMEOUT == res) LOGD(TAG, "spin_once timeout, so break loop");
	    if (SPIN_ABORTED == res) LOGD(TAG, "spin_once aborted, so break loop");
	    break;
	}
	//vTaskDelay(1);
    }
}


// RECEIVE ############################################################

void uwb_board_receive_async(uwb_board_receive_cb_t rx_cb, cb_flags_t flags) {
    active_rx_cb = rx_cb;
    active_rx_flags = flags;

    switch_to_irq();
    LOGD(TAG, "dwt_rxenable status_reg: %ld", dwt_read32bitreg(SYS_STATUS_ID));

    /*
    if (flags & TRX_AUTO_RESTART)
    	dwt_setautorxreenable(1);
    else
	dwt_setautorxreenable(0);
    */

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}


static struct { // data for synchronous receive
    const uint8_t *frame;
    uint16_t frame_len;
    timestamp_t rx_ts;
} cb_result;

static void board_receive_cb(const uint8_t *frame, uint16_t frame_len, timestamp_t rx_ts) {
    cb_result = { frame, frame_len, rx_ts};
}

const uint8_t *uwb_board_receive(uint16_t *frame_len_ptr, timestamp_t *rx_ts_ptr) {
    LOGD(TAG, "uwb_board_receive\n");

    cb_result = {0};

    active_rx_flags = rx_ts_ptr ? TRX_CB_TIMESTAMP : 0;
    active_rx_cb = board_receive_cb;
    
    switch_to_irq();
    LOGD(TAG, "dwt_rxenable status_reg: %ld", dwt_read32bitreg(SYS_STATUS_ID));
    //dwt_setautorxreenable(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uwb_board_spin_once(portMAX_DELAY);
    if (cb_result.frame) {
        if (frame_len_ptr) *frame_len_ptr = cb_result.frame_len;
        if (rx_ts_ptr)     *rx_ts_ptr     = cb_result.rx_ts;
    }
    return cb_result.frame;
}


// TRANSMIT ###########################################################

static uint8_t transmit_prepared = false;
void uwb_board_prepare_transmit() {
    switch_to_irq();
    transmit_prepared = true;
}


bool uwb_board_transmit(
    uint8_t *frame,
    uint16_t frame_len,
    timestamp_t *tx_ts_ptr,
    uint8_t delay_type,
    uwb_board_receive_cb_t rx_cb,
    cb_flags_t rx_cb_flags )
{
    if (!transmit_prepared)
        uwb_board_prepare_transmit();
    transmit_prepared = false;

    uint8_t ranging = 1;
    dwt_writetxdata(frame_len, frame, 0); // write frame into TX buffer
    dwt_writetxfctrl(frame_len + FCS_LEN, 0, ranging); // zero offset in TX buffer

    // optional set delay
    if (delay_type != UWB_DELAY_NONE) {
        if ((*tx_ts_ptr) & ~TIMESTAMP_MASK) {
	    fatal_error("illegal delay timestamp");
	}
    }

    timestamp_t planned_time;
    uint8_t start_cmd;
    switch (delay_type) {
        case UWB_DELAY_NONE:
	    start_cmd = DWT_START_TX_IMMEDIATE;
	    break;
        case UWB_DELAY_ABSOLUTE:
            dwt_setdelayedtrxtime((*tx_ts_ptr - board_attrs->tx_antenna_delay)>>8);
            planned_time = *tx_ts_ptr; 
	    start_cmd = DWT_START_TX_DELAYED;
	    break;
        case UWB_DELAY_RX_RELATIVE:
            dwt_setdelayedtrxtime((*tx_ts_ptr)>>8);
            dwt_readrxtimestamp((uint8_t*)&planned_time);
            planned_time = (planned_time + *tx_ts_ptr) & TIMESTAMP_MASK; 
	    start_cmd = DWT_START_TX_DLY_RS;
	    break;
        case UWB_DELAY_TX_RELATIVE:
            dwt_setdelayedtrxtime((*tx_ts_ptr)>>8);
            dwt_readtxtimestamp((uint8_t*)&planned_time);
            planned_time = (planned_time + *tx_ts_ptr) & TIMESTAMP_MASK; 
	    start_cmd = DWT_START_TX_DLY_TS;
	    break;
	default:
	    LOGE(TAG, "illegal delay type %d\n", delay_type);
	    fatal_error("illegal delay type");
            return false;
    }

    if (rx_cb) {
	active_rx_cb = rx_cb;
        active_rx_flags = rx_cb_flags;
	start_cmd |= DWT_RESPONSE_EXPECTED;
	//dwt_setrxaftertxdelay(0); // wait 0 µs after transmit before going to rx mode
        //dwt_setrxtimeout(DWT_TIMEOUT_MAX);
    }

    //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK|SYS_STATUS_HPDWARN_BIT_MASK);
    if (dwt_starttx(start_cmd) == DWT_ERROR) {
        LOGE(TAG, "TRANSMIT FAILED: dwt_starttx(0x%x) == DWT_ERROR", start_cmd);
        uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
        if (status & SYS_STATUS_HPDWARN_BIT_MASK) {
            LOGW(TAG, "after failed dwt_starttx -> SYS_STATUS: HPDWARN set");
        } else {
            LOGW(TAG, "after failed dwt_starttx -> SYS_STATUS: 0x%lx", status);
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_HPDWARN_BIT_MASK);
        return false;
    }

    spin_result_t res;
    do {
        res = spin_once(pdMS_TO_TICKS(100));
    } while (res == SPIN_IDLE);

    if (res == SPIN_TX) {
        if (tx_ts_ptr) {
            dwt_readtxtimestamp((uint8_t*)tx_ts_ptr);
            *tx_ts_ptr &= TIMESTAMP_MASK;
            if (delay_type == UWB_DELAY_ABSOLUTE) {
                if (planned_time != *tx_ts_ptr) {
	            LOGE(TAG, "uwb_board_transmit: wrong tx timestamp, diff: %llu",
	                uwb_board_timestamp_diff(planned_time, *tx_ts_ptr));
		    return false;
		}
            }
        }
    } else {
        LOGE(TAG, "TX timeout/error");
        return false;
    }

    return true;
}


// IEEE 802.15.4 MAC ##################################################

#include "ieee802_15_4.h"

static uint8_t board_mac_seq_no = 0;

typedef struct {
    timestamp_t t1;
    timestamp_t t2;
    timestamp_t t3;
    timestamp_t t4;
    timestamp_t t5;
    timestamp_t t6;
} timestamps_dstwr_t;

static const uint16_t mac_fcf = mac_make_fcf_data_short(); // MAC frame control format


#define DSTWR_FRAME_LEN (9 + 2 + 6 * 5)  // 9 bytes mac, 1 byte command (DS-TWR), twr frame no, 6 * 5 bytes timestamp
#define COMMAND_DSTWR   196
#define DSTWR_FRAME_POLL  0
#define DSTWR_FRAME_REPLY 1
#define DSTWR_FRAME_FINAL 2
#define DSTWR_FRAME_INFO  3

static void board_transmit_mac(uint8_t *frame, uint16_t frame_len, timestamp_t *delay, uint8_t delay_type) {
    uint8_t seq_no = mac_get_seq(frame) + 1;
    mac_set_seq(frame, seq_no);
    mac_set_src_short_addr(frame, board_dev_id);

    uwb_board_transmit(frame, frame_len, delay, delay_type);
}

bool uwb_board_receive_cb(const uint8_t *frame, uint16_t frame_len, timestamp_t rx_ts) {
    if (DSTWR_FRAME_LEN != frame_len || frame[0] != *((uint8_t*)&mac_fcf) || frame[0] != *(((uint8_t*)&mac_fcf)+1) )
	return false;
    if (COMMAND_DSTWR != frame[9])
	return false;
    if (DSTWR_FRAME_INFO <= frame[10])
	return false;

    // the received frame is a DS-TWR frame, so handle it by replying
    uint8_t source_id = mac_get_src_short_addr(frame);
    uint8_t dstwr_seq_no = frame[10];
    if (DSTWR_FRAME_POLL == dstwr_seq_no) {
        LOGD(TAG, "reply to a DS-TWR poll frame");
	uint8_t *p = (uint8_t *)frame; // remove const!
	p[10]++;
	mac_set_dst_short_addr(p, source_id);
	timestamp_t delay = uwb_board_get_tx_min_delay();
	timestamp_t planned = uwb_board_plan_delayed(rx_ts + delay);
        put_u64_5(p+(12+1*5), rx_ts);	
        put_u64_5(p+(12+2*5), planned);	
	board_transmit_mac((uint8_t *)frame, frame_len, &delay, UWB_DELAY_RX_RELATIVE);
        if (planned != delay) fatal_error("delay wrong");
        return true;
    }
    return false;
}

void uwb_board_dstwr(uint16_t dest_address, timestamps_dstwr_t *timestamps) {

    /* The DS-TWR runs in 6 steps
     *
     * 1. prepareing
     * 2. this device: sending the poll frame, t1 := tx_timestamp of poll frame.
     * 3. other device: receive poll frame and transmit back as reply frame.
     *    t2 := rx_timestamp of the poll frame, t3 := tx_timestamp of reply frame
     * 4. this device: receive reply frame and transmit back final frame.
     *    t4 := rx_timestamp of the poll frame, t5 := tx_timestamp of final frame
     * 5. other device: receive final frame and transmit back info frame about final rx_timestamp.
     *    t6 := rx_timestamp of the final frame
     * 6. return timestamps
     */
   
    uint16_t frame_len = DSTWR_FRAME_LEN;
    uint8_t frame[41];

    // 1: prepare frame and transmitting
    // switch on packet filtering
    if (9 != mac_header_length(mac_fcf))
        fatal_error("mach_header_length is not 9");

    mac_build_header_short(frame, mac_fcf, 0, board_pan_id, dest_address, board_dev_id);

    static uint8_t * const p = frame + mac_header_length(mac_fcf);

    // switch_to_irq();

    // 2: transmit poll, switch to receiveing
    
/*

        timestamp_t now = uwb_board_get_systime();
        timestamp_t planned = uwb_board_plan_delayed(now + min_delay);
        frame[0] = FRAME_TYPE_TEST;
        put_u64_5(frame+2,planned);
        frame[7] = FRAME_TYPE_TEST_CMD_REQ4RESPONSE;
        timestamp_t last_rx = other_anchor ? other_anchor->tx_timestamp[other_anchor->last_idx] : 0ULL;
        put_u64_5(frame+8,last_rx);

        tx_ts = planned;
        uwb_board_transmit_delayed(frame, frame_len, &tx_ts);

        timestamp_t ts_end = uwb_board_get_systime();
        timestamp_t ts_diff = ts_end - now;
        uint32_t ts_diff_us = (uint32_t)(ts_diff / ticks_per_ns / 1000);
        int64_t esp_end = esp_timer_get_time();
        int64_t esp_diff_us = esp_end - esp_now;
        if (tx_ts == planned)
        {
            printf("TX test frame send as planned @%llx, diff: %lld, end: %lld, diff: %lld = %ld usi, esp_us: %lld\n",
                   tx_ts, tx_ts-planned, ts_end, ts_diff, ts_diff_us, esp_diff_us);
        } else {
            printf("TX test frame send @ %lld=%llx, planned was: %llx, diff: %lld, end: %lld, diff: %lld = %ld usi, esp_us: %lld\n",
                   tx_ts, tx_ts, planned, tx_ts-planned, ts_end, ts_diff,ts_diff_us, esp_diff_us);
        }

*/
}

