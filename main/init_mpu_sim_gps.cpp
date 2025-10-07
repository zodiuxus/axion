#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/uart.h"

#include "esp_log.h"

#include "logic.h"

/* Port and pin definitions */

// MPU6050
#define PIN_I2C_SDA 5
#define PIN_I2C_SCL 6
#define PIN_MPU_INT 7
#define I2C_MASTER_FREQ_HZ 100000

// A7670E
#define PIN_UART_TX 17
#define PIN_UART_RX 18
#define PIN_UART_RTS 15
#define PIN_UART_CTS 16
#define UART_TIMEOUT_MS 2000


void i2c_setup() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_I2C_SDA;
    conf.scl_io_num = PIN_I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_LOGI(TAG, "Param config");
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_LOGI(TAG, "Driver install");
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    vTaskDelete(nullptr);
}

void uart_setup() {
    constexpr uart_port_t port_uart = PORT_UART;
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    // QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(port_uart, BUF_UART, BUF_UART, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(port_uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port_uart, PIN_UART_TX, PIN_UART_RX, PIN_UART_RTS, PIN_UART_CTS));
    vTaskDelete(nullptr);
}

void mpu_setup() {
    mpu.initialize();
    mpu.dmpInitialize();

    if (mpu.testConnection()) ESP_LOGI(TAG, "Success");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);

    vTaskDelete(nullptr);
}

void setup_gnss() {
    if (send_at_command("AT+CGNSSPWR?", 1000, "CGNSSPWR: 0")) {
        send_at_command("AT+CGNSSPWR=1", 120000, "READY");
        vTaskDelay(60000/portTICK_PERIOD_MS);
        send_at_command("AT+CGNSSPORTSWITCH=1,1", 3000, "OK");
        send_at_command("AT+CGNSSCOLD", 9000, "OK");
        send_at_command("AT+CGNSSTST=1", 1000, "OK");
        send_at_command("AT+CGNSSMOD=?", 9000, "OK");
    }
    send_at_command("AT+CGNSSURC=0", 10000, "OK");
    if (send_at_command("AT+CGNSSINFO?", 9000, "OK")) xTaskCreate(TaskFunction_t(get_coords), "get_coords", 4096, nullptr, 2, nullptr);
    vTaskDelete(nullptr);
}


void at_init() {
    ESP_LOGW(TAG, "Setting up A7670E...");
    uart_flush(PORT_UART);
    send_at_command("AT+CGNSSPWR=1", 1000, "");
    send_at_command("AT+CMEE=2", 1000, "OK");
    // send_at_command("AT+CRESET", 15000, "READY");

    ESP_LOGW(TAG, "Device information...\n");
    send_at_command("AT+SIMCOMATI", 1000, "");

    ESP_LOGW(TAG, "Setting up connectivity...");
    if (!send_at_command("AT+COPS?", 45000, "29403")) {
        if (send_at_command("AT+CPIN?", 2000, "SIM PIN")) {
            send_at_command("AT+CPIN=8284", 1000, "READY");
        }
        if(send_at_command("AT+CEREG=?", 1000, "2")) send_at_command("AT+CEREG=2", 1000, "OK");
        else send_at_command("AT+CEREG=1", 1000, "OK");
        send_at_command("AT+CGATT=1", 1000, "OK");
    }

    ESP_LOGW(TAG, "Ensuring connectivity...");
    send_at_command("AT+CPAS", 1000, "OK");
    send_at_command("AT+CEREG?", 5000, "OK");
    send_at_command("AT+CNSMOD?", 2000, "8");
    send_at_command("AT+COPS?", 45000, "29403");

    setup_gnss();
    vTaskDelete(nullptr);
}
