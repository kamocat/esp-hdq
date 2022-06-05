/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD 17
#define ECHO_TEST_RXD 16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      2
#define ECHO_UART_BAUD_RATE     57600
#define ECHO_TASK_STACK_SIZE    4096

#define BUF_SIZE (1024)

uint8_t hdq_read(uint8_t cmd){
    const uint8_t hi = 0xFE;
    const uint8_t lo = 0xC0;
    uint8_t buf[32];
    int i = 0;
    for(; i < 8; ++i){
        buf[i] = cmd&1 ? hi : lo;
        cmd >>= 1;
    }
    uart_write_bytes(ECHO_UART_PORT_NUM, buf, 8);
    int len = uart_read_bytes(ECHO_UART_PORT_NUM, buf, 32, 1);
    uint8_t tmp = 0;
    for(i = len-8; i < len; ++i){
        tmp >>= 1;
        if(buf[i] > 0xF8)
            tmp |= 0x80;
    }
    return tmp;
}

uint16_t hdq_read16(uint8_t cmd){
    uint16_t result = hdq_read(cmd + 1);
    result <<= 8;
    result |= hdq_read(cmd);
    return result;
}

void hdq_reset(void){
    uint8_t buf = 0xFF;
    uart_write_bytes_with_break(ECHO_UART_PORT_NUM, &buf, 1, 20);
    vTaskDelay(1);
}

uint8_t hdq_name(char * buf){
    uint8_t len = hdq_read(0x62);
    for(int i = 0; i < len; ++i){
        buf[i] = hdq_read(i+0x63);
    }
    return len;
}

// Requires 20 bytes in buffer
void bin16(uint16_t x, char * buf){
    for(int i = 4; i; --i){
        for(int j = 4; j; --j){
            *buf++ = x&0x8000 ? '1' : '0';
            x <<= 1;
        }
        *buf++ = ' ';
    }
    *buf++ = 0; // null termination
}

      

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    //ESP_ERROR_CHECK(gpio_set_direction(ECHO_TEST_TXD, GPIO_MODE_OUTPUT_OD));
    //ESP_ERROR_CHECK(gpio_pullup_en(ECHO_TEST_RXD));

    while (1) {
        hdq_reset();
        ESP_LOGI("Charge", "%d%%", hdq_read16(0x2c));
        ESP_LOGI("Health", "%d%%", hdq_read16(0x2e));
        ESP_LOGI("Voltage", "%dmv", hdq_read16(8));
        float celcius = (hdq_read16(6) - 2731) * 0.1; // Kelvin to Celcius
        ESP_LOGI("Temperature", "%.1f C", celcius);
        ESP_LOGI("Full Charge Capacity", "%d mAh", hdq_read16(0x12));
        ESP_LOGI("Charge Remaining", "%d mAh", hdq_read16(0x10));
        ESP_LOGI("Cycle Count", "%d", hdq_read16(0x2a));
        ESP_LOGI("Depth of Discharge", "%d", hdq_read16(0x36));
        ESP_LOGI("Average Current", "%dmA", hdq_read16(0x14));
        ESP_LOGI("Stack used", "%d bytes", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
