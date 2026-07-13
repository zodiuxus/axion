/**
 * modem.cpp — A7670E bring-up + GNSS polling.
 *
 * Replaces init_mpu_sim_gps.cpp's at_init() + setup_gnss() + get_coords()
 * with a single bring-up task and a separate long-running GNSS poller.
 *
 * All UART access goes through at_command.* (mutex-protected).
 * SIM PIN and APN come from secrets.h (gitignored).
 */
#include "modem.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "at_command.h"
#include "config.h"
#include "secrets.h"
#include "state.h"

static const char *TAG = "axion.modem";

/* ---- NMEA / Simcom +CGNSSINFO parser -------------------------------- */
/* +CGNSSINFO: [<mode>],[<GPS-SVs>],[<GLONASS-SVs>],[BEIDOU-SVs],
 *            [<lat>],[<N/S>],[<lon>],[<E/W>],[<date>],[<UTC-time>],
 *            [<alt>],[<speed>],[<course>],[<PDOP>],[HDOP],[VDOP]
 * Required positions:
 *   4 = lat, 5 = N/S, 6 = lon, 7 = E/W, 10 = alt, 11 = speed (knots)
 */
static void parse_cgnssinfo(char *data, double *lat, double *lon,
                            float *alt, float *speed_knots)
{
    *lat = 0.0; *lon = 0.0; *alt = 0.0f; *speed_knots = 0.0f;

    /* Skip the "+CGNSSINFO:" header if present so the comma parser starts
     * at the first field. */
    char *start = strstr(data, "+CGNSSINFO:");
    if (start != nullptr) start += strlen("+CGNSSINFO:");
    else                  start = data;
    /* Skip leading colon/space if any (e.g. "+CGNSSINFO: 1,2,..."). */
    while (*start == ':' || *start == ' ' || *start == '\r' || *start == '\n') start++;

    char *saveptr = nullptr;
    char *token   = strtok_r(start, ",\r\n", &saveptr);
    int   idx     = 0;
    bool  lat_neg = false, lon_neg = false;
    double l_lat = 0.0, l_lon = 0.0;
    float  l_alt = 0.0f, l_spd = 0.0f;

    while (token != nullptr) {
        switch (idx) {
            case 4:  l_lat   = atof(token); break;
            case 5:  lat_neg = (token[0] == 'S' || token[0] == 's'); break;
            case 6:  l_lon   = atof(token); break;
            case 7:  lon_neg = (token[0] == 'W' || token[0] == 'w'); break;
            case 10: l_alt   = (float)atof(token); break;
            case 11: l_spd   = (float)atof(token); break;
            default: break;
        }
        token = strtok_r(nullptr, ",\r\n", &saveptr);
        idx++;
    }

    *lat         = lat_neg ? -l_lat : l_lat;
    *lon         = lon_neg ? -l_lon : l_lon;
    *alt         = l_alt;
    *speed_knots = l_spd;
}

/* ---- GNSS power-up sequence ----------------------------------------- */
static void gnss_power_up(void)
{
    if (at_command_send("AT+CGNSSPWR?", 1000, "CGNSSPWR: 0")) {
        ESP_LOGI(TAG, "GNSS powering up...");
        at_command_send("AT+CGNSSPWR=1", 120000, "READY");
        vTaskDelay(pdMS_TO_TICKS(3000));
        at_command_send("AT+CGNSSPORTSWITCH=1,1", 3000, "OK");
        at_command_send("AT+CGNSSCOLD", 9000, "OK");
        at_command_send("AT+CGNSSTST=1", 1000, "OK");
        at_command_send("AT+CGNSSMOD=?", 9000, "OK");
    }
    at_command_send("AT+CGNSSURC=0", 10000, "OK");
    /* It can take a while for a cold-start fix. Try once and let the
     * polling task keep asking. */
    at_command_send("AT+CGNSSINFO?", 9000, "OK");
    ESP_LOGI(TAG, "GNSS setup done; polling task will continue");
}

/* ---- Public tasks --------------------------------------------------- */
void modem_setup_task(void * /*arg*/)
{
    ESP_LOGW(TAG, "Bringing up A7670E...");
    at_modem_flush_rx();

    /* Nudge GNSS power on early so it has time to start acquiring. */
    at_command_send("AT+CGNSSPWR=1", 1000, "");
    at_command_send("AT+CMEE=2", 1000, "OK");

    ESP_LOGW(TAG, "Device information:");
    at_command_send("AT+SIMCOMATI", 1000, "");

    ESP_LOGW(TAG, "Setting up connectivity...");
    bool registered = at_command_send("AT+COPS?", 45000, "29403");
    if (!registered) {
        if (at_command_send("AT+CPIN?", 2000, "SIM PIN")) {
            char cpin_cmd[32];
            snprintf(cpin_cmd, sizeof(cpin_cmd), "AT+CPIN=%s", SIM_PIN);
            at_command_send(cpin_cmd, 2000, "READY");
        }
        if (at_command_send("AT+CEREG=?", 1000, "2")) {
            at_command_send("AT+CEREG=2", 1000, "OK");
        } else {
            at_command_send("AT+CEREG=1", 1000, "OK");
        }
        at_command_send("AT+CGATT=1", 1000, "OK");
    }

    ESP_LOGW(TAG, "Verifying connectivity...");
    at_command_send("AT+CPAS", 1000, "OK");
    at_command_send("AT+CEREG?", 5000, "OK");
    at_command_send("AT+CNSMOD?", 2000, "8");
    at_command_send("AT+COPS?", 45000, "29403");

    /* SMS text mode. */
    at_command_send("AT+CMGF=1", 9000, "OK");

    axion_state_set_ready(BIT_AT_READY);
    ESP_LOGI(TAG, "A7670E modem ready (SMS path available)");

    /* Power up GNSS now that the modem is registered. */
    gnss_power_up();
    axion_state_set_ready(BIT_GNSS_READY);

    /* Hand off to the polling task. */
    xTaskCreate(modem_gnss_task, "gnss_poll", 4096, nullptr, 2, nullptr);

    vTaskDelete(nullptr);
}

void modem_gnss_task(void * /*arg*/)
{
    /* Wait until the modem bring-up task says GNSS is configured. */
    axion_state_wait_all(BIT_GNSS_READY);

    char    local_buf[UART_BUF_SIZE];
    double  lat = 0.0, lon = 0.0;
    float   alt = 0.0f, spd_knots = 0.0f;

    while (true) {
        if (at_command_send("AT+CGNSSINFO", 3000, "+CGNSSINFO:")) {
            at_command_last_response(local_buf, sizeof(local_buf));
            parse_cgnssinfo(local_buf, &lat, &lon, &alt, &spd_knots);
            float speed_ms = spd_knots * 0.5144447f;
            axion_state_set_gnss(lat, lon, alt, speed_ms);
        }
        vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_MS));
    }
}
