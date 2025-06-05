#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <soc/rtc_cntl_reg.h>
#include <string.h>
#include "config.h"
#include "ulp_uart.h"

RTC_DATA_ATTR uint32_t sequence_number = 0;

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

void app_main() {
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
    bool is_first_run = !ulp_uart_is_running();

    if (is_first_run) {
        puts("Setting up ULP...");
        ulp_uart_setup(GPIO_RX);
        puts("ULP setup done");
    }

    if (!is_first_run) {
        send_success_semaphore = xSemaphoreCreateBinary();
        DataPacket packet_to_send;
        packet_to_send.sequence_number = sequence_number;
        memcpy(packet_to_send.data, ulp_uart_rx_buffer, sizeof(ulp_uart_rx_buffer));
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
