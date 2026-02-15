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
    if (debugOutput) ESP_LOGI(TAG, "Sent AT command: %s\n", cmd);

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
    if (debugOutput) ESP_LOGI(TAG, "Response:\n%s", response_buf);
    if (response_found) return true;
    return false;
}

void combine_sensors() {
  xEventGroupWaitBits(sysReady, SYSTEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
  printf("\n\n\n\n");
  while (true) {
    printf("\033[4F");
    printf("\033[KTemperature info: %.2f\n", temp_values[0]);
    printf("\033[KPosition: %f, %f, %f\n", lat, lon, alt);
    printf("\033[KSpeed: %.2f\n", speed);
    printf("\033[KRPY: %3.2f, %3.2f, %3.2f\n", ypr[2]*180/M_PI, ypr[1]*180/M_PI, ypr[0]*180/M_PI);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(nullptr);
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
  
  speed = speed * 0.51445;
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
          if (debugOutput) {
            printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
            printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
            printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
          }
	    }

		vTaskDelay(5/portTICK_PERIOD_MS);
	}

	vTaskDelete(nullptr);
}
