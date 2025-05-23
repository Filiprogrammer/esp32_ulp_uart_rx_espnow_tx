#include <esp_attr.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/rtc_io_reg.h>
#include <string.h>
#include "config.h"

RTC_DATA_ATTR bool ulp_running = false;
RTC_DATA_ATTR uint8_t rx_buffer[246];
RTC_DATA_ATTR uint32_t sequence_number = 0;
#define RTC_MEM_ULP_PROGRAM_ADDRESS 0x100
#define RTC_MEM_RECEIVE_BUFFER_ADDRESS 0x300
// BAUD_RATE 2400
// ULP_CLOCK_SPEED 8000000
// Should be 3333.33 clock cycles per bit

const uint8_t receiver_address[6] = RECEIVER_ADDRESS;

typedef struct __attribute__((packed)) DataPacket {
    uint32_t sequence_number;
    uint8_t data[246];
} DataPacket;

bool send_success;
SemaphoreHandle_t send_success_semaphore = NULL;

void on_data_sent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        #ifdef DEBUG
        puts("Delivery success");
        #endif
        send_success = true;
    } else {
        #ifdef DEBUG
        puts("Delivery failed");
        #endif
        send_success = false;
    }

    xSemaphoreGive(send_success_semaphore);
}

bool send_data_packet(const DataPacket packet_to_send) {
    #ifdef DEBUG
    printf("Sending Packet - Seq: %lu, Data: ", packet_to_send.sequence_number);
    for (uint8_t i = 0; i < 246; i++) {
        printf("%02X", packet_to_send.data[i]);
    }
    putchar('\n');
    #endif

    if (esp_now_send(receiver_address, (uint8_t*)&packet_to_send, sizeof(DataPacket)) != ESP_OK) {
        #ifdef DEBUG
        puts("ESP-NOW send command failed immediately");
        #endif
        return false;
    }

    xSemaphoreTake(send_success_semaphore, portMAX_DELAY);
    return send_success;
}

void setup_esp_now() {
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

    // Register the send callback
    esp_now_register_send_cb(on_data_sent);

    // Add receiver as a peer
    esp_now_peer_info_t peer_info;
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, receiver_address, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    #ifdef DEBUG
    puts("Sender ready");
    #endif
}

void setupULP() {
    gpio_config_t pin_config = {
        .pin_bit_mask = (1ULL << GPIO_RX),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pin_config);
    rtc_gpio_init(GPIO_RX);
    rtc_gpio_set_direction(GPIO_RX, RTC_GPIO_MODE_INPUT_ONLY);

    const ulp_insn_t ulp_task[] = {
        I_MOVI(R3, RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t)),
        M_LABEL(0),
        I_DELAY(4000),
        M_LABEL(1),
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX), // Read RX pin
        M_BGE(1, 1), // Wait for start bit

        // Start bit received, delay for one and a half of a baud thingie so we are in the middle
        I_DELAY(5000 - 6),

        // Read the 1. bit from RX into R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 2. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 1),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 3. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 2),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 4. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 3),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 5. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 4),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 6. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 5),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 7. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 6),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3333),

        // Read the 8. bit from RX, OR it together with R2 and place the result into R0
        I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX, RTC_GPIO_IN_NEXT_S + RTC_GPIO_RX),
        I_LSHI(R1, R0, 7),
        I_ORR(R0, R1, R2),

        // Store the received byte into RTC_SLOW_MEM
        I_ST(R0, R3, 0),

        I_ADDI(R0, R3, 1),
        I_MOVR(R3, R0),
        M_BL(0, RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t) + 246),
        I_WAKE(),
        I_MOVI(R3, RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t)),
        M_BX(0)
    };

    memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);
    size_t size = sizeof(ulp_task) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(RTC_MEM_ULP_PROGRAM_ADDRESS / sizeof(uint32_t), ulp_task, &size);
    ulp_run(RTC_MEM_ULP_PROGRAM_ADDRESS / sizeof(uint32_t));
    ulp_running = true;
}

void app_main() {
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
    bool is_first_run = !ulp_running;

    if (is_first_run) {
        puts("Setting up ULP...");
        setupULP();
        puts("ULP setup done");
    }

    if (!is_first_run) {
        send_success_semaphore = xSemaphoreCreateBinary();
        DataPacket packet_to_send;
        packet_to_send.sequence_number = sequence_number;
        memcpy(packet_to_send.data, rx_buffer, sizeof(rx_buffer));
        setup_esp_now();
        for (uint8_t retry_count = 0; retry_count <= MAX_RETRANSMITS; retry_count++) {
            if (send_data_packet(packet_to_send)) {
                // Successfully delivered, no need to retry anymore
                break;
            }
        }
        esp_wifi_stop();
        sequence_number++;
    }

    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}

void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    for (uint8_t i = 0; i < 246; i++) {
        rx_buffer[i] = (uint8_t)RTC_SLOW_MEM[RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t) + i];
    }
}
