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

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (1024)

esp_err_t hdq_read(uint8_t cmd, uint8_t * reply){
    const uint8_t hi = 0xFE;
    const uint8_t lo = 0xC0;
    uint8_t buf[20];
    int i = 0;
    // Flush buffer
    uart_read_bytes(ECHO_UART_PORT_NUM, buf, 16, 0);
    for(; i < 8; ++i){
        buf[i] = cmd&1 ? hi : lo;
        cmd >>= 1;
    }
    uart_write_bytes(ECHO_UART_PORT_NUM, buf, 8);
    int len = uart_read_bytes(ECHO_UART_PORT_NUM, buf, 16, 10 / portTICK_RATE_MS);
    uint8_t tmp = 0;
    for(; i < len; ++i){
        if(buf[i] > 0xF8)
            tmp |= 0x80;
        tmp >>= 1;
    }
    *reply = tmp;
    if(len < 17)
        return ESP_ERR_NOT_FINISHED;
    return ESP_OK;
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
        // Write data back to the UART
        uint8_t cmd, reply;
        uint16_t volts;
        cmd = 9;
        hdq_read(cmd, &reply);
        volts = reply;
        volts <<= 8;
        cmd = 8;
        hdq_read(cmd, &reply);
        volts |= reply;

        ESP_LOGI("Volts", "%dmv", volts);
        uart_write_bytes_with_break(ECHO_UART_PORT_NUM, &cmd, 1, 10);
        vTaskDelay(100/portTICK_RATE_MS);
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
