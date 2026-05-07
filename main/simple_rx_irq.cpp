#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "dw3000.h"
#include "dw3000_device_api.h"

#define DWT_INT_RXFCG_BIT_MASK	SYS_STATUS_RXFCG_BIT_MASK
#define DWT_INT_RXFCE_BIT_MASK	DWT_INT_RFCE
#define DWT_INT_RXRFTO_BIT_MASK	DWT_INT_RFTO
#define DWT_INT_RXPTO_BIT_MASK	DWT_INT_RXPTO

// -------- Pin Config --------
#define PIN_IRQ GPIO_NUM_34
#define PIN_RST GPIO_NUM_27
#define PIN_CS  GPIO_NUM_4

static void fatal_error(const char* error)
{
    printf("fatal error: %s\r\n", error);
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}

// -------- Queue --------
static QueueHandle_t irq_queue;

// -------- ISR --------
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t dummy = 1;
    xQueueSendFromISR(irq_queue, &dummy, NULL);
}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    printf("receive sucess\n");
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    printf("receive error\n");
}


// -------- GPIO Init --------
static void gpio_init_irq()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_IRQ, gpio_isr_handler, NULL);
}


static void dw3000_init0() // sourci of void board_init()
{
  printf("board_setup()\n");


  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);

  vTaskDelay(pdMS_TO_TICKS(200)); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  if (!dwt_checkidlerc()) fatal_error("IDLE FAILED");// Need to make sure DW IC is in IDLE_RC before proceeding

  dwt_softreset();
  //vTaskDelay(pdMS_TO_TICKS(200));

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) fatal_error("INIT FAILED");

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Default communication configuration. We use default non-STS DW mode. */
  dwt_config_t config = {
    .chan = 5,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = 1,
    .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .sfdTO = (129 + 8 - 8),
    .stsMode = DWT_STS_MODE_OFF,
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0
  };

  if (dwt_configure(&config)) fatal_error("CONFIG FAILED");
  // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  // printf("DW3000 configured\n");
  //vTaskDelay(pdMS_TO_TICKS(200));

  // dwt_setrxtimeout(0); // unbegrenzt
  //switch_to_polling();
  printf("board_setup() initialise and configure successfully done\n");
}


static void dw3000_init()
{
    UART_init();
    printf("dw3000_init");

    dw3000_init0();
    printf("dw3000_init0 done");
    
    // vTaskDelay(pdMS_TO_TICKS(200));
    // dwt_setrxantennadelay(16436);
    //printf("DW3000 antenna delay set\n");

    //dwt_setcallbacks(NULL, rx_ok_cb, rx_err_cb, rx_err_cb, NULL, NULL);

    //dwt_setinterrupt(0, 0, DWT_ENABLE_INT_ONLY);

    // 5. Clear pending interrupts im DW3000
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);

    vTaskDelay(pdMS_TO_TICKS(200));
    // Interrupts im Chip aktivieren
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    printf("DW3000 dwt_setinterrupt\n");
    dwt_setinterrupt(
        DWT_INT_RXFCG_BIT_MASK |
        DWT_INT_RXFCE_BIT_MASK |
        DWT_INT_RXRFTO_BIT_MASK |
        DWT_INT_RXPTO_BIT_MASK,
        0,
        DWT_ENABLE_INT_ONLY
    );
    printf("DW3000 dwt_setinterrupt done\n");

    vTaskDelay(pdMS_TO_TICKS(200));
    // RX starten
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    printf("DW3000 dwt_rxenable done\n");
}

// -------- RX Task --------
static void dw3000_rx_task(void* arg)
{
    irq_queue = xQueueCreate(10, sizeof(uint32_t));

    dw3000_init();
    
    gpio_init_irq();

    uint32_t evt;

    while (1) {
        if (xQueueReceive(irq_queue, &evt, portMAX_DELAY)) {

            uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

            // -------- Frame OK --------
            if (status & DWT_INT_RXFCG_BIT_MASK) {

                uint16_t len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

                if (len > 0 && len < 1024) {
                    uint8_t buffer[1024];
                    dwt_readrxdata(buffer, len, 0);

                    printf("RX (%d): ", len);
                    for (int i = 0; i < len; i++) {
                        printf("%02X ", buffer[i]);
                    }
                    printf("\n");

                    // Optional: Blink erkennen
                    if (buffer[0] == 0xC5) {
                        printf("Blink frame detected\n");
                    }
                }

                dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_RXFCG_BIT_MASK);
            }

            // -------- Errors --------
            if (status & DWT_INT_RXFCE_BIT_MASK) {
                dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_RXFCE_BIT_MASK);
                printf("CRC error\n");
            }

            if (status & DWT_INT_RXRFTO_BIT_MASK) {
                dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_RXRFTO_BIT_MASK);
            }

            if (status & DWT_INT_RXPTO_BIT_MASK) {
                dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_RXPTO_BIT_MASK);
            }

            // Wichtig: RX neu starten
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    }
}


// -------- Main --------
extern "C" void app_main(void)
{
    //xTaskCreate(dw3000_rx_task, "dw_rx", 4096, NULL, 5, NULL);
    xTaskCreate(dw3000_rx_task, "dw_rx", 8192, NULL, 5, NULL);
}

