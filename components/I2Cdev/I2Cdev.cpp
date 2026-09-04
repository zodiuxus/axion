// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// EFM32 stub port by Nicolas Baldeck <nicolas@pioupiou.fr>
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-01-02 - Initial release
//      2026-09    - Migrated from the deprecated legacy command-link driver
//                   (driver/i2c.h) to the modern I2C master driver
//                   (driver/i2c_master.h). Failure semantics fixed: reads
//                   now return 0 on error instead of pretending success,
//                   so testConnection() and DMP write-verification actually
//                   detect a dead bus. The old code also redefined
//                   ESP_ERROR_CHECK to log "esp_err_t = -1" and continue,
//                   which produced hundreds of useless log lines per boot
//                   when the bus was miswired.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===============================================
*/

#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "sdkconfig.h"

#include "I2Cdev.h"

static const char *TAG = "I2Cdev";

/* ---- Shared bus + per-address device handle cache --------------------- */

static i2c_master_bus_handle_t s_bus = nullptr;
/* Bus frequency passed to installBus(); applied to every device handle
 * added afterwards. In the new driver there is no bus-wide speed -
 * scl_speed_hz lives in the per-device config. */
static uint32_t s_bus_hz = 400000;

struct I2cDevEntry {
    uint8_t                   addr;
    i2c_master_dev_handle_t   handle;
};
static I2cDevEntry s_devs[I2CDEV_MAX_DEVICES];
static size_t      s_dev_count  = 0;
static SemaphoreHandle_t s_lock = nullptr;   /* guards lazy device adds */

bool I2Cdev::installBus(int sda_gpio, int scl_gpio, uint32_t hz)
{
    if (s_bus) return true;   /* already installed */

    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port              = I2C_NUM_0;
    cfg.sda_io_num            = (gpio_num_t)sda_gpio;
    cfg.scl_io_num            = (gpio_num_t)scl_gpio;
    /* NOTE: i2c_master_bus_config_t.sda_pullup_en/scl_pullup_en were
     * REMOVED in ESP-IDF v5.5 - do not set them. Internal pull-ups are
     * enabled solely via flags.enable_internal_pullup below. At 400 kHz
     * the internal (~45k) pull-ups are weak; 4.7k external pull-ups on
     * SDA/SCL are strongly recommended. */
    cfg.clk_source            = I2C_CLK_SRC_DEFAULT;
    /* Glitch filter: recommended default from the IDF docs. */
    cfg.glitch_ignore_cnt     = 7;
    cfg.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        s_bus = nullptr;
        return false;
    }

    if (s_lock == nullptr) {
        s_lock = xSemaphoreCreateMutex();
    }
    s_bus_hz = hz;
    return true;
}

i2c_master_bus_handle_t I2Cdev::busHandle()
{
    return s_bus;
}

i2c_master_dev_handle_t I2Cdev::deviceHandle(uint8_t devAddr)
{
    if (s_bus == nullptr) {
        ESP_LOGE(TAG, "deviceHandle(0x%02x): bus not installed", devAddr);
        return nullptr;
    }

    /* Fast path: already cached. */
    for (size_t i = 0; i < s_dev_count; ++i) {
        if (s_devs[i].addr == devAddr) return s_devs[i].handle;
    }

    /* Slow path: register the device now. A mutex keeps two tasks from
     * adding the same address concurrently; after the first transactions
     * the fast path above is lock-free. */
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(I2CDEV_XFER_TIMEOUT_MS)) != pdTRUE) {
        return nullptr;
    }

    /* Re-check under the lock (another task may have added it). */
    for (size_t i = 0; i < s_dev_count; ++i) {
        if (s_devs[i].addr == devAddr) {
            if (s_lock) xSemaphoreGive(s_lock);
            return s_devs[i].handle;
        }
    }

    i2c_master_dev_handle_t handle = nullptr;
    if (s_dev_count < I2CDEV_MAX_DEVICES) {
        i2c_device_config_t dev = {};
        dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev.device_address  = devAddr;
        dev.scl_speed_hz    = s_bus_hz;   /* from installBus() */
        esp_err_t err = i2c_master_bus_add_device(s_bus, &dev, &handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add device 0x%02x failed: %s",
                     devAddr, esp_err_to_name(err));
        } else {
            s_devs[s_dev_count].addr   = devAddr;
            s_devs[s_dev_count].handle = handle;
            s_dev_count++;
        }
    } else {
        ESP_LOGE(TAG, "device cache full (0x%02x not added)", devAddr);
    }

    if (s_lock) xSemaphoreGive(s_lock);
    return handle;
}

bool I2Cdev::probe(uint8_t devAddr)
{
    if (s_bus == nullptr) return false;
    return i2c_master_probe(s_bus, devAddr, pdMS_TO_TICKS(I2CDEV_XFER_TIMEOUT_MS)) == ESP_OK;
}

/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

/** Initialize I2C0 (no-op - the bus is installed via installBus()).
 */
void I2Cdev::initialize() {

}

/** Enable or disable I2C (no-op on ESP-IDF).
 */
void I2Cdev::enable(bool isEnabled) {

}

/** Default timeout value for read operations.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout (kept for API compat; the new
 *        driver uses I2CDEV_XFER_TIMEOUT_MS per transaction)
 * @return Status of read operation (non-zero = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {

    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    if (count != 0) *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (non-zero = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (non-zero = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/** Read multiple bytes from an 8-bit device register.
 *
 * Single combined transaction: START, addr+W, regAddr, REPEATED-START,
 * addr+R, data..., STOP. (The old implementation split this into two
 * separate transactions via SelectRegister(), which worked but doubled
 * the bus traffic and could interleave with another task's transaction
 * between the register-set and the read.)
 *
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Unused (kept for API compatibility)
 * @return number of bytes read (0 on failure - callers like
 *         testConnection() rely on this to detect a dead device)
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    i2c_master_dev_handle_t dev = deviceHandle(devAddr);
    if (dev == nullptr) return 0;

    esp_err_t err = i2c_master_transmit_receive(dev, &regAddr, 1,
                                                data, length,
                                                I2CDEV_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "read 0x%02x/0x%02x len=%u: %s",
                 devAddr, regAddr, length, esp_err_to_name(err));
        return 0;
    }
    return length;
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data){

    uint8_t data1[] = {(uint8_t)(data>>8), (uint8_t)(data & 0xff)};
    return writeBytes(devAddr, regAddr, 2, data1);
}

/** Point the device's register pointer at `reg` (single write
 * transaction). Kept for API compatibility - readBytes/writeBytes now
 * set the register pointer inside their own combined transactions. */
void I2Cdev::SelectRegister(uint8_t dev, uint8_t reg){
    i2c_master_dev_handle_t handle = deviceHandle(dev);
    if (handle == nullptr) return;

    esp_err_t err = i2c_master_transmit(handle, &reg, 1,
                                        I2CDEV_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "select reg 0x%02x/0x%02x: %s",
                 dev, reg, esp_err_to_name(err));
    }
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) == 0) return false;
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10101011 masked | value
    uint8_t b = 0;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 *
 * Single transaction: START, addr+W, regAddr, data..., STOP.
 *
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
    i2c_master_dev_handle_t dev = deviceHandle(devAddr);
    if (dev == nullptr) return false;

    uint8_t buf[I2CDEV_MAX_WRITE_LEN];
    if (length >= sizeof(buf)) return false;

    buf[0] = regAddr;
    memcpy(buf + 1, data, length);

    esp_err_t err = i2c_master_transmit(dev, buf, length + 1,
                                        I2CDEV_XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "write 0x%02x/0x%02x len=%u: %s",
                 devAddr, regAddr, length, esp_err_to_name(err));
        return false;
    }
    return true;
}


/**
 * read word
 * @param devAddr
 * @param regAddr
 * @param data
 * @param timeout
 * @return
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout){
    uint8_t msb[2] = {0,0};
    if (readBytes(devAddr, regAddr, 2, msb, timeout) == 0) return 0;
    *data = (int16_t)((msb[0] << 8) | msb[1]);
    return 1;
}
