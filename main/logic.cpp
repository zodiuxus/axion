#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string.h>
#include <driver/uart.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "cstring"
#include "logic.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "values.h"

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

void combine_sensors() {
  while (true) {
    ESP_LOGI(TAG, "Temperature info: %.2f", temp_values[0]);
    ESP_LOGI(TAG, "GPS info:\nPosition: %f, %f, %f\nSpeed: %.2f", lat, lon, alt, speed);
    ESP_LOGI(TAG, "RPY: %3.1f, %3.1f, %3.1f", ypr[2]*180/M_PI, ypr[1]*180/M_PI, ypr[0]*180/M_PI);
    


    // They get the data far faster than this
    // however, for simplicity, I will print
    // them once every second, as it is for
    // testing purposes, and nothing else
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(nullptr);
}

float convert_coords(float num) {
  uint8_t degrees = static_cast<int>(num/100);
  float minutes = num - (degrees*100);
  return degrees + (minutes / 60.0);
}

void parse_gnss_info(char* data) {
  /*
  * [<mode>],[<GPS-SVs>],[<GLONASS-SVs>],[BEIDOU-SVs],
  * [<lat>],[<N/S>],[<log>],[<E/W>],[<date>],[<UTC-time>],
  * [<alt>],[<speed>],[<course>],[<PDOP>],[HDOP],[VDOP]
  * Required positions:
  * 4 - lat, 5- N/S, 6 - log, 7 - E/W, 10 - alt, 11 - speed
  */
  
  char buffer[BUF_UART];
  strncpy(buffer, data, BUF_UART-1);
  buffer[BUF_UART-1] = '\0';

  uint8_t counter = 0;
  char* token;
  char* rest = nullptr;
  for (token = strtok_r(buffer, ",", &rest); token != nullptr; token = strtok_r(nullptr, ",", &rest)) {
    switch (counter) {
      case 4: lat = atof(token); break;
      case 5: if (strcmp(token, "S") == 0) lat = -lat; break;
      case 6: lon = atof(token); break;
      case 7: if (strcmp(token, "W") == 0) lon = -lon; break;
      case 10: alt = atof(token); break;
      case 11: speed = atof(token); break;
      default: break;
    }   
    counter++;
  }
  
  lat = convert_coords(lat);
  lon = convert_coords(lon);
  speed = speed * 0.51445;
  ESP_LOGI(TAG, "lat %f - lon %f - alt %f - speed %f", lat, lon, alt, speed);
}

void get_coords() {
    while (true) {
        send_at_command("AT+CGNSSINFO", 9000, "OK");
        parse_gnss_info(response_buf);
    vTaskDelay(5000/portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}

void get_rpy() {
  while(1){
	    mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        mpu.resetFIFO();

	    } else if (mpuIntStatus & 0x02) {
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
          printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
          printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
	    }

		vTaskDelay(5/portTICK_PERIOD_MS);
	}

	vTaskDelete(nullptr);
}
