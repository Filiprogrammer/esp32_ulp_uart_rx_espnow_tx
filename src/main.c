#include <esp_attr.h>
#include <esp_sleep.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <sdkconfig.h>
#include <soc/rtc_io_reg.h>
#include <string.h>
#include "config.h"

RTC_DATA_ATTR bool ulp_running = false;
RTC_DATA_ATTR uint8_t rx_buffer[246];
#define RTC_MEM_ULP_PROGRAM_ADDRESS 0x100
#define RTC_MEM_RECEIVE_BUFFER_ADDRESS 0x300
// BAUD_RATE 2400
// ULP_CLOCK_SPEED 8000000
// Should be 3333.33 clock cycles per bit

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
    if (!ulp_running) {
        puts("Setting up ULP...");
        setupULP();
        puts("ULP setup done");
    }

    esp_sleep_enable_ulp_wakeup();

    for (uint32_t i = 0; i < 246; ++i) {
        uint8_t c = rx_buffer[i];
        putchar((char)c);
    }
    fflush(stdout);

    esp_deep_sleep_start();
}

void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    for (uint8_t i = 0; i < 246; i++) {
        rx_buffer[i] = (uint8_t)RTC_SLOW_MEM[RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t) + i];
    }
}
