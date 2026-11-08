/**
 * modem.cpp - A7670E bring-up + GNSS polling.
 *
 * All UART access goes through at_command.* (mutex-protected). SIM PIN,
 * carrier, and APN come from secrets.h as factory defaults, seeded into
 * NVS on first boot (NVS wins afterwards).
 *
 * The configured PLMN is a preference: manual selection is tried first,
 * then automatic, accepting home (CEREG stat=1) or roaming (stat=5) -
 * so SMS to 112 works from any registered EU network.
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
#include "calibration_store.h"

static const char *TAG = "axion.modem";

/* NVS keys for the carrier-secrets cache (see secrets.h.example for
 * the factory defaults and the rationale). */
#define NVS_KEY_CARRIER   "carrier"
#define NVS_KEY_APN_NAME  "apn_name"
#define NVS_KEY_APN_USER  "apn_user"
#define NVS_KEY_APN_PASS  "apn_pass"

/* ---- Carrier-secrets resolver ---------------------------------------- */
/* NVS first, secrets.h fallback; cached for the duration of the boot. */
static char s_carrier[8]   = "";   /* MCC+MNC, max 6 digits + NUL */
static char s_apn_name[32] = "";
static char s_apn_user[32] = "";
static char s_apn_pass[32] = "";
static bool s_secrets_resolved = false;

static void resolve_carrier_secrets(void)
{
    if (s_secrets_resolved) return;

    /* Carrier: NVS first, secrets.h fallback. */
    if (calibration_store_get_str(NVS_KEY_CARRIER, s_carrier, sizeof(s_carrier)) != ESP_OK) {
        strncpy(s_carrier, CARRIER_MCCMNC, sizeof(s_carrier) - 1);
        s_carrier[sizeof(s_carrier) - 1] = '\0';
        if (s_carrier[0] != '\0') {
            calibration_store_set_str(NVS_KEY_CARRIER, s_carrier);
        }
    }

    /* APN: same pattern. If APN_NAME is empty, the modem bring-up skips
     * the CGDCONT configuration. */
    if (calibration_store_get_str(NVS_KEY_APN_NAME, s_apn_name, sizeof(s_apn_name)) != ESP_OK) {
        strncpy(s_apn_name, APN_NAME, sizeof(s_apn_name) - 1);
        s_apn_name[sizeof(s_apn_name) - 1] = '\0';
        calibration_store_set_str(NVS_KEY_APN_NAME, s_apn_name);
    }
    if (calibration_store_get_str(NVS_KEY_APN_USER, s_apn_user, sizeof(s_apn_user)) != ESP_OK) {
        strncpy(s_apn_user, APN_USER, sizeof(s_apn_user) - 1);
        s_apn_user[sizeof(s_apn_user) - 1] = '\0';
        calibration_store_set_str(NVS_KEY_APN_USER, s_apn_user);
    }
    if (calibration_store_get_str(NVS_KEY_APN_PASS, s_apn_pass, sizeof(s_apn_pass)) != ESP_OK) {
        strncpy(s_apn_pass, APN_PASS, sizeof(s_apn_pass) - 1);
        s_apn_pass[sizeof(s_apn_pass) - 1] = '\0';
        calibration_store_set_str(NVS_KEY_APN_PASS, s_apn_pass);
    }

    ESP_LOGI(TAG, "carrier secrets: plmn='%s' apn='%s'",
             s_carrier, s_apn_name);
    s_secrets_resolved = true;
}

/* ---- NMEA / Simcom +CGNSSINFO parser -------------------------------- */
/* +CGNSSINFO: fields 4=lat 5=N/S 6=lon 7=E/W 10=alt 11=speed(knots).
 * No fix = every field empty (atof() would read 0.0): an empty <lat>
 * field is the no-fix signal.
 * @return true on fix (outputs filled); false = no fix (untouched). */
static bool parse_cgnssinfo(char *data, double *lat, double *lon,
                            float *alt, float *speed_knots)
{
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
    bool  have_lat = false;
    double l_lat = 0.0, l_lon = 0.0;
    float  l_alt = 0.0f, l_spd = 0.0f;

    while (token != nullptr) {
        switch (idx) {
            case 4:  l_lat = atof(token); have_lat = (token[0] != '\0'); break;
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

    if (!have_lat) return false;   /* no fix - caller keeps last known position */

    *lat         = lat_neg ? -l_lat : l_lat;
    *lon         = lon_neg ? -l_lon : l_lon;
    *alt         = l_alt;
    *speed_knots = l_spd;
    return true;
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
        at_command_send("AT+CGNSSTST=0", 1000, "OK");
        at_command_send("AT+CGNSSMOD=?", 9000, "OK");
    }
    at_command_send("AT+CGNSSURC=0", 10000, "OK");
    /* Cold-start fix may take a while; the poller keeps asking. */
    at_command_send("AT+CGNSSINFO?", 9000, "OK");
    ESP_LOGI(TAG, "GNSS setup done; polling task will continue");
}

/* ---- Cellular registration (home + roaming) ------------------------- */
/* Poll AT+CEREG? until registered:
 *   stat=1 home, stat=5 roaming; 0/2/3/4 = not registered.
 * Accepting stat=5 is what enables roaming (incl. SMS to 112 on any
 * registered EU network). */
static bool wait_for_registration(uint32_t timeout_ms)
{
    const uint32_t step_ms = 2000;
    char    buf[128];
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        if (at_command_send("AT+CEREG?", 2000, "+CEREG:")) {
            at_command_last_response(buf, sizeof(buf));
            /* +CEREG: <n>,<stat>[,...] - find the stat field. */
            char *p = strstr(buf, "+CEREG:");
            if (p) {
                p += strlen("+CEREG:");
                /* skip "<n>" */
                while (*p && *p != ',') p++;
                if (*p == ',') p++;
                int stat = atoi(p);
                if (stat == 1 || stat == 5) {
                    ESP_LOGI(TAG, "registered (CEREG stat=%d, %s)",
                             stat, (stat == 5) ? "roaming" : "home");
                    return true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        elapsed += step_ms;
    }
    ESP_LOGW(TAG, "no registration after %ums", (unsigned)timeout_ms);
    return false;
}

/* ---- Public tasks --------------------------------------------------- */
void modem_setup_task(void * /*arg*/)
{
    /* Install the UART driver before any at_command_* call. On failure
     * continue anyway: sensors still work, monitor never unblocks. */
    esp_err_t uart_err = at_modem_init();
    if (uart_err != ESP_OK) {
        ESP_LOGE(TAG, "at_modem_init failed: %s - modem unusable this boot",
                 esp_err_to_name(uart_err));
        /* Continue anyway: the sensor stack works without the modem, and
         * the monitor task gates on BIT_AT_READY which we never set. */
    }

    ESP_LOGW(TAG, "Bringing up A7670E...");
    at_modem_flush_rx();

    /* Resolve carrier/APN from NVS (or seed from secrets.h on first boot). */
    resolve_carrier_secrets();

    /* Nudge GNSS power on early so it has time to start acquiring. */
    at_command_send("AT+CGNSSPWR=1", 1000, "");
    at_command_send("AT+CMEE=2", 1000, "OK");

    ESP_LOGW(TAG, "Device information:");
    at_command_send("AT+SIMCOMATI", 1000, "");

    ESP_LOGW(TAG, "Setting up connectivity...");
    /* SIM PIN first - some SIMs refuse all other AT commands until
     * they're unlocked. */
    if (at_command_send("AT+CPIN?", 2000, "SIM PIN")) {
        char cpin_cmd[32];
        snprintf(cpin_cmd, sizeof(cpin_cmd), "AT+CPIN=%s", SIM_PIN);
        at_command_send(cpin_cmd, 2000, "READY");
    }

    /* Enable verbose CEREG URCs so we can poll the stat field reliably. */
    if (at_command_send("AT+CEREG=?", 1000, "2")) {
        at_command_send("AT+CEREG=2", 1000, "OK");
    } else {
        at_command_send("AT+CEREG=1", 1000, "OK");
    }

    /* Manual PLMN if configured, else automatic; registration accepts
     * home or roaming (see wait_for_registration). */
    if (s_carrier[0] != '\0') {
        char cops_cmd[24];
        snprintf(cops_cmd, sizeof(cops_cmd),
                 "AT+COPS=1,2,\"%s\"", s_carrier);
        if (!at_command_send(cops_cmd, 5000, "OK")) {
            ESP_LOGW(TAG, "manual PLMN '%s' rejected; falling back to automatic",
                     s_carrier);
            at_command_send("AT+COPS=0", 5000, "OK");
        }
    } else {
        at_command_send("AT+COPS=0", 5000, "OK");
    }

    /* Wait up to 45s for home (stat=1) OR roaming (stat=5) registration. */
    bool registered = wait_for_registration(45000);
    if (!registered) {
        ESP_LOGE(TAG, "no cellular registration - SMS path may fail");
        /* Continue anyway; the modem may recover on its own. */
    }

    /* PS attach: not strictly needed for SMS, set for future data use. */
    at_command_send("AT+CGATT=1", 2000, "OK");

    /* APN configuration: only if a non-empty APN name was provided.
     * CGDCONT defines a PDP context; we use CID 1. */
    if (s_apn_name[0] != '\0') {
        char cgdcont_cmd[80];
        snprintf(cgdcont_cmd, sizeof(cgdcont_cmd),
                 "AT+CGDCONT=1,\"IP\",\"%s\"", s_apn_name);
        at_command_send(cgdcont_cmd, 3000, "OK");
        /* Auth + username/password if provided (PAP, CID 1). */
        if (s_apn_user[0] != '\0' && s_apn_pass[0] != '\0') {
            char auth_cmd[96];
            snprintf(auth_cmd, sizeof(auth_cmd),
                     "AT+CGAUTH=1,1,\"%s\",\"%s\"", s_apn_user, s_apn_pass);
            at_command_send(auth_cmd, 3000, "OK");
        }
    }

    ESP_LOGW(TAG, "Verifying connectivity...");
    at_command_send("AT+CPAS", 1000, "OK");
    at_command_send("AT+CEREG?", 5000, "OK");
    at_command_send("AT+CNSMOD?", 2000, "8");

    /* SMS text mode. */
    at_command_send("AT+CMGF=1", 9000, "OK");

    axion_state_set_ready(BIT_AT_READY);
    /* Arm collision escalation. The monitor's alert loop additionally
     * waits for BIT_TEMP_CALIBRATED before it starts. */
    axion_state_set_armed(true);
    ESP_LOGI(TAG, "A7670E modem ready (SMS path available) - system armed");

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
    int     consecutive_no_fix = 0;

    while (true) {
        if (at_command_send("AT+CGNSSINFO", 3000, "+CGNSSINFO:")) {
            at_command_last_response(local_buf, sizeof(local_buf));
            if (parse_cgnssinfo(local_buf, &lat, &lon, &alt, &spd_knots)) {
                float speed_ms = spd_knots * 0.5144447f;
                axion_state_set_gnss(lat, lon, alt, speed_ms);
                if (consecutive_no_fix > 0) {
                    ESP_LOGI(TAG, "GNSS fix reacquired after %d poll(s)",
                             consecutive_no_fix);
                }
                consecutive_no_fix = 0;
            } else {
                /* No fix: keep last known position, mark stale,
                 * throttle the log. */
                axion_state_note_gnss_no_fix();
                consecutive_no_fix++;
                if (consecutive_no_fix == 1 || (consecutive_no_fix % 12) == 0) {
                    ESP_LOGW(TAG, "no GNSS fix (%d consecutive polls) - "
                                  "reporting last known position if alerted",
                             consecutive_no_fix);
                }
            }
        } else {
            /* AT command itself failed/timed out - also stale, same handling. */
            axion_state_note_gnss_no_fix();
        }
        vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_MS));
    }
}
