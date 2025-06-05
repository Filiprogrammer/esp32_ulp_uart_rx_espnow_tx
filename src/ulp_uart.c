#include "ulp_uart.h"
#include <driver/rtc_io.h>
#include <esp32/ulp.h>
#include <esp_sleep.h>
#include <sdkconfig.h>
#include <soc/rtc_io_reg.h>
#include <string.h>

#define RTC_MEM_ULP_PROGRAM_ADDRESS 0x100
#define RTC_MEM_RECEIVE_BUFFER_ADDRESS 0x300
// BAUD_RATE 2400
// ULP_CLOCK_SPEED 8500000
// Should be 3541.66 clock cycles per bit

RTC_DATA_ATTR bool ulp_running = false;
RTC_DATA_ATTR uint8_t ulp_uart_rx_buffer[ULP_UART_RECEIVE_BUFFER_SIZE];

// 6.11 RTC_MUX Pin List https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
static const uint8_t gpio_to_rtc_gpio_conversion[] = {
    11,   0xFF,   12, 0xFF,   10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF,   15,   14,   16,   13, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,    6,    7,   17, 0xFF, 0xFF,
    0xFF, 0xFF,    9,    8,    4,    5,    0,    1,    2,    3
};

static uint8_t gpio_to_rtc_bit(gpio_num_t gpio_pin) {
    return RTC_GPIO_IN_NEXT_S + gpio_to_rtc_gpio_conversion[gpio_pin];
}

void ulp_uart_setup(gpio_num_t rx_pin) {
    gpio_config_t pin_config = {
        .pin_bit_mask = (1ULL << rx_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pin_config);
    rtc_gpio_init(rx_pin);
    rtc_gpio_set_direction(rx_pin, RTC_GPIO_MODE_INPUT_ONLY);

    uint8_t rtc_pin_bit = gpio_to_rtc_bit(rx_pin);

    const ulp_insn_t ulp_task[] = {
        I_MOVI(R3, RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t)),
        M_LABEL(0),
        I_DELAY(7333), // Wait out parity bit and stop bit from last frame
        M_LABEL(1),
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit), // Read RX pin
        M_BGE(1, 1), // Wait for start bit

        // Start bit received, delay for one and a half of a baud thingie so we are in the middle
        I_DELAY(5000 - 6),

        // Read the 1. bit from RX into R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6),

        // Read the 2. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 1),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6 - 6 - 6),

        // Read the 3. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 2),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6 - 6 - 6),

        // Read the 4. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 3),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6 - 6 - 6),

        // Read the 5. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 4),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6 - 6 - 6),

        // Read the 6. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 5),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6 - 6 - 6),

        // Read the 7. bit from RX and OR it onto R2
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 6),
        I_ORR(R0, R1, R2),
        I_MOVR(R2, R0),
        I_DELAY(3541 - 8 - 6 - 6 - 6),

        // Read the 8. bit from RX, OR it together with R2 and place the result into R0
        I_RD_REG(RTC_GPIO_IN_REG, rtc_pin_bit, rtc_pin_bit),
        I_LSHI(R1, R0, 7),
        I_ORR(R0, R1, R2),

        // Store the received byte into RTC_SLOW_MEM
        I_ST(R0, R3, 0),

        I_ADDI(R0, R3, 1),
        I_MOVR(R3, R0),
        M_BL(0, RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t) + ULP_UART_RECEIVE_BUFFER_SIZE),
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

bool ulp_uart_is_running(void) {
    return ulp_running;
}

void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    for (size_t i = 0; i < ULP_UART_RECEIVE_BUFFER_SIZE; i++) {
        ulp_uart_rx_buffer[i] = (uint8_t)RTC_SLOW_MEM[RTC_MEM_RECEIVE_BUFFER_ADDRESS / sizeof(uint32_t) + i];
    }
}
