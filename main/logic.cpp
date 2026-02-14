#include <driver/uart.h>
#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "cstring"
#include "logic.h"

bool send_at_command(const char* cmd, int max_timeout_ms, const char* expected_response) {
    char cmd_buf[64];
    int len = snprintf(cmd_buf, sizeof(cmd_buf), "%s\r", cmd);

    uart_write_bytes(PORT_UART, cmd_buf, len);
    uart_flush(PORT_UART);
    ESP_LOGI(TAG, "Sent AT command: %s\n", cmd);

    len = 0;
    bool response_found = false;
    int64_t start_time = esp_timer_get_time() / 1000;

    while ((esp_timer_get_time() / 1000) - start_time < max_timeout_ms) {
        int read_len = uart_read_bytes(PORT_UART, reinterpret_cast<uint8_t *>(response_buf) + len, BUF_UART - len - 1, pdMS_TO_TICKS(100));

        if (read_len > 0) {
            len += read_len;
            response_buf[len] = '\0';
        }

        if (strstr(response_buf, expected_response)) response_found = true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Response:\n%s", response_buf);
    if (response_found) return true;
    return false;
}

void get_rpy() {
    mpu.setDMPEnabled(true);
    while(true){
        mpuIntStatus = mpu.getIntStatus();
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();

            // otherwise, check for DMP data ready interrupt frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
            printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
            printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
        }

        //Best result is to match with DMP refresh rate
        // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
        // Now its 0x13, which means DMP is refreshed with 10Hz rate
        // vTaskDelay(5/portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}

void parse_gnss_info(char* response) {
    /*
    * [<mode>],[<GPS-SVs>],[<GLONASS-SVs>],[BEIDOU-SVs],
    * [<lat>],[<N/S>],[<log>],[<E/W>],[<date>],[<UTC-time>],[<alt>],
    * [<speed>],[<course>],[<PDOP>],[HDOP],[VDOP]
    * required indecies: 4, 5, 6, 7, 10, 11, 12 optional
     */
    char* split = strtok(response, ",");
    uint8_t counter = 0;
    while (split != nullptr) {
        switch (counter) {
            case 4: pos[0] = atof(split); break;
            case 5: if (split == "S") pos[0] = -pos[0]; break;
            case 6: pos[1] = atof(split); break;
            case 7: if (split == "W") pos[1] = -pos[1]; break;
            case 10: pos[2] = atof(split); break;
            case 11: speed = atof(split); break;
            default: break;
        }
        split = strtok(nullptr,",");
        counter++;
    }
}

void setup_thermo() {
    gpio_config_t therm_pin_config;
    therm_pin_config.mode = GPIO_MODE_INPUT;
    therm_pin_config.pull_up_en = GPIO_PULLUP_ENABLE;
    therm_pin_config.pin_bit_mask = (1ULL << GPIO_NUM_17);
    gpio_config(&therm_pin_config);
}

void get_thermometer_reading() {
    setup_thermo();
    while (true) {
        float level = gpio_get_level(GPIO_NUM_17);
        ESP_LOG_DEBUG()
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void health_check() {

    vTaskDelay(pdMS_TO_TICKS(5000));
}

void get_coords() {
    while (true) {
        send_at_command("AT+CGNSSINFO", 9000, "OK");
        parse_gnss_info(response_buf);
    }
    vTaskDelete(nullptr);
}
