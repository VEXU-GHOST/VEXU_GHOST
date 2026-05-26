#include <stdio.h>
#include <cstring>
#include <optional>
#include <array>
#include "polling_firmware.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Device.h"
#include "comms.h"
#include "SoftwareI2CBus.h"
#include "HardwareI2CBus.h"
#include "i2c.pio.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define MAX_DEVICES         20
#define MAX_UNIQUES_DEVICES 5
#define MAX_POLLS           100

// Task stack sizes in words
#define STACK_COMMS         1024
#define STACK_POLLING       2048

// ---------------------------------------------------------------------------
// I2C bus table
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  sda;
    uint8_t  scl;
    I2CBus  *i2c;
} i2c_bus_t;

static i2c_bus_t i2c_buses[9] = {};

// ---------------------------------------------------------------------------
// Device array
// ---------------------------------------------------------------------------

static std::array<std::unique_ptr<Device>, MAX_DEVICES> devices;
static uint32_t total_devices = 0;

// ---------------------------------------------------------------------------
// Data arrays (file scope — avoids stack overflow in polling_task)
// ---------------------------------------------------------------------------

static uint16_t g_r[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint16_t g_g[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint16_t g_b[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint32_t g_lux[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint32_t g_cct[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_acc_x[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_acc_y[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_acc_z[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_gyro_x[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_gyro_y[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_gyro_z[MAX_POLLS][MAX_UNIQUES_DEVICES];
static float    g_temperature[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint16_t g_distance_mm[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint8_t  g_range_status[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint16_t g_sigma_mm[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint32_t g_signal_rate_kcps[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint32_t g_ambient_rate_kcps[MAX_POLLS][MAX_UNIQUES_DEVICES];
static uint8_t  g_gpio_state[MAX_POLLS][MAX_UNIQUES_DEVICES];

// ---------------------------------------------------------------------------
// Shared configuration (protected by g_config_mutex)
// ---------------------------------------------------------------------------

static SemaphoreHandle_t g_polling_sem;   // counting: given by comms_task, taken by polling_task
static SemaphoreHandle_t g_config_mutex;

// initialize with values for testing as needed

static SensorConfig g_config = {
    .polling_interval_ms      = 500,
    .polling_times             = 10,
    .color_sensor_cnt          = 1,
    .color_sensor_bus_sel      = {2, 2, 3, 4, 5},
    .imu_cnt                   = 0,
    .imu_bus_sel               = {2, 2, 3, 4, 5},
    .distance_sensor_cnt       = 0,
    .distance_sensor_bus_sel   = {2, 2, 3, 4, 5},
    .io_expander_cnt           = 0,
    .io_expander_bus_sel       = {2, 2, 3, 4, 5},
    .io_expander_pin_mode      = {
        {1, 0, 0, 1},
        {255, 255, 255, 255},
        {255, 255, 255, 255},
        {255, 255, 255, 255},
        {255, 255, 255, 255},
    },
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void reset_device_array() {
    for (auto &d : devices) d.reset();
}

static void init_i2c_buses() {
    uint offset_pio0 = pio_add_program(pio0, &i2c_program);
    uint offset_pio1 = pio_add_program(pio1, &i2c_program);

    i2c_buses[1] = {SDA_INPUT_1, SCL_INPUT_1, new HardwareI2CBus(i2c1)};
    i2c_buses[2] = {SDA_INPUT_2, SCL_INPUT_2, new HardwareI2CBus(i2c0)};
    i2c_buses[3] = {SDA_INPUT_3, SCL_INPUT_3, new SoftwareI2CBus(3, pio0, 0, offset_pio0, SDA_INPUT_3, SCL_INPUT_3, I2C_FREQ_HZ)};
    i2c_buses[4] = {SDA_INPUT_4, SCL_INPUT_4, new SoftwareI2CBus(4, pio0, 1, offset_pio0, SDA_INPUT_4, SCL_INPUT_4, I2C_FREQ_HZ)};
    i2c_buses[5] = {SDA_INPUT_5, SCL_INPUT_5, new SoftwareI2CBus(5, pio0, 2, offset_pio0, SDA_INPUT_5, SCL_INPUT_5, I2C_FREQ_HZ)};
    i2c_buses[6] = {SDA_INPUT_6, SCL_INPUT_6, new SoftwareI2CBus(6, pio1, 0, offset_pio1, SDA_INPUT_6, SCL_INPUT_6, I2C_FREQ_HZ)};
    i2c_buses[7] = {SDA_INPUT_7, SCL_INPUT_7, new SoftwareI2CBus(7, pio1, 1, offset_pio1, SDA_INPUT_7, SCL_INPUT_7, I2C_FREQ_HZ)};
    i2c_buses[8] = {SDA_INPUT_8, SCL_INPUT_8, new SoftwareI2CBus(8, pio1, 2, offset_pio1, SDA_INPUT_8, SCL_INPUT_8, I2C_FREQ_HZ)};

    i2c_init(i2c_buses[1].i2c->get_i2c_instance(), I2C_FREQ_HZ);
    gpio_set_function(i2c_buses[1].sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c_buses[1].scl, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_buses[1].sda);
    gpio_pull_up(i2c_buses[1].scl);

    i2c_init(i2c_buses[2].i2c->get_i2c_instance(), I2C_FREQ_HZ);
    gpio_set_function(i2c_buses[2].sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c_buses[2].scl, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_buses[2].sda);
    gpio_pull_up(i2c_buses[2].scl);
}

// ---------------------------------------------------------------------------
// comms_task — reads USB CDC bytes, parses frames, updates config, signals polling
// ---------------------------------------------------------------------------

static void comms_task(void *) {
    while (true) {
        // getchar_timeout_us lets the task yield when no data is available,
        // so polling_task gets CPU time between incoming bytes.
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) {
            taskYIELD();
            continue;
        }

        if (!comms_feed_byte((uint8_t)c)) continue;

        const uint8_t  cmd     = comms_get_cmd();
        const uint8_t *payload = comms_get_payload();
        const uint16_t plen    = comms_get_payload_len();

        if (cmd == CMD_CONFIGURE) {
            SensorConfig new_cfg;
            // Copy current config as default so unspecified keys keep their values
            xSemaphoreTake(g_config_mutex, portMAX_DELAY);
            new_cfg = g_config;
            xSemaphoreGive(g_config_mutex);

            if (comms_parse_yaml_config(payload, plen, new_cfg)) {
                xSemaphoreTake(g_config_mutex, portMAX_DELAY);
                g_config = new_cfg;
                xSemaphoreGive(g_config_mutex);
                comms_send_ack(0);
                xSemaphoreGive(g_polling_sem);  // schedule one polling run
            } else {
                comms_send_ack(1);  // parse error
            }

        } else if (cmd == CMD_POLL_NOW) {
            comms_send_ack(0);
            xSemaphoreGive(g_polling_sem);  // trigger with current config
        }
    }
}

// ---------------------------------------------------------------------------
// polling_task — waits for trigger, polls all sensors, transmits results
// ---------------------------------------------------------------------------

static void polling_task(void *) {
    while (true) {
        // Block until comms_task signals a polling run
        xSemaphoreTake(g_polling_sem, portMAX_DELAY);

        // Snapshot config atomically so comms_task can update g_config mid-run
        SensorConfig cfg;
        xSemaphoreTake(g_config_mutex, portMAX_DELAY);
        cfg = g_config;
        xSemaphoreGive(g_config_mutex);

        // ---- Device initialisation ----------------------------------------

        uint8_t device_index = 0;

        for (uint8_t i = 0; i < cfg.color_sensor_cnt; i++) {
            devices[device_index] = std::make_unique<ISL29124Device>(
                color_sensor_address[i], i2c_buses[cfg.color_sensor_bus_sel[i]].i2c);
            if (!devices[device_index]->init()) {
                printf("Color sensor #%d init failed!\n", i + 1);
                // while (1) tight_loop_contents();
            }
            device_index++;
        }

        for (uint8_t i = 0; i < cfg.imu_cnt; i++) {
            devices[device_index] = ICM20602Device::create(
                imu_address[i],
                i2c_buses[cfg.imu_bus_sel[i]].i2c,
                i2c_buses[cfg.imu_bus_sel[i]].sda,
                i2c_buses[cfg.imu_bus_sel[i]].scl);
            if (!devices[device_index]->init()) {
                printf("IMU #%d init failed!\n", i + 1);
                // while (1) tight_loop_contents();
            }
            device_index++;
        }

        for (uint8_t i = 0; i < cfg.distance_sensor_cnt; i++) {
            devices[device_index] = std::make_unique<VL53L4CDDevice>(
                distance_sensor_address[i], i2c_buses[cfg.distance_sensor_bus_sel[i]].i2c);
            if (!devices[device_index]->init()) {
                printf("Distance sensor #%d init failed!\n", i + 1);
                // while (1) tight_loop_contents();
            }
            device_index++;
        }

        for (uint8_t i = 0; i < cfg.io_expander_cnt; i++) {
            devices[device_index] = std::make_unique<TCA9536Device>(
                io_expander_address[i], i2c_buses[cfg.io_expander_bus_sel[i]].i2c);
            if (!devices[device_index]->init()) {
                printf("IO expander #%d init failed!\n", i + 1);
                // while (1) tight_loop_contents();
            }
            for (uint8_t pin = 0; pin < 4; pin++) {
                if (cfg.io_expander_pin_mode[i][pin] != 255) {
                    auto ioexpander = static_cast<TCA9536Device *>(devices[device_index].get());
                    if (!ioexpander->pin_mode(pin, cfg.io_expander_pin_mode[i][pin])) {
                        printf("IO expander #%d pin %d mode failed!\n", i + 1, pin);
                        // while (1) tight_loop_contents();
                        ioexpander->destroy();
                    }
                }
            }
            device_index++;
        }

        total_devices = device_index;

        // ---- Polling loop --------------------------------------------------

        for (uint32_t poll_count = 0; poll_count < cfg.polling_times; poll_count++) {
            // Reset per-type indices each iteration so they always index [0..cnt)
            uint8_t color_idx = 0;
            uint8_t imu_idx   = 0;
            uint8_t dist_idx  = 0;
            uint8_t gpio_idx  = 0;

            for (uint8_t i = 0; i < total_devices; i++) {
                switch (devices[i]->get_type()) {
                    case DeviceType::ISL29124: {
                        auto color_sensor = static_cast<ISL29124Device *>(devices[i].get());
                        auto data = color_sensor->get_data();
                        if (color_idx < cfg.color_sensor_cnt) {
                            // if data is not valid and there are data from previous timestamp, use data from
                            // previous timestampe as buffer, otherwise if the data is not valid and its the
                            // first timestamp or if device is not properly intialized data is defaulted to 0
                            // from get_data()
                            if (!data.valid && poll_count > 0) {
                                g_r[poll_count][color_idx]   = g_r[poll_count - 1][color_idx];
                                g_g[poll_count][color_idx]   = g_g[poll_count - 1][color_idx];
                                g_b[poll_count][color_idx]   = g_b[poll_count - 1][color_idx];
                                g_lux[poll_count][color_idx] = g_lux[poll_count - 1][color_idx];
                                g_cct[poll_count][color_idx] = g_cct[poll_count - 1][color_idx];
                                printf("Color sensor #%d data invalid and buffered at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                            }
                            else {
                                if (!color_sensor->is_initialized()) {
                                    printf("Color sensor #%d not initialized and tried to get data at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                                }
                                g_r[poll_count][color_idx]   = data.r;
                                g_g[poll_count][color_idx]   = data.g;
                                g_b[poll_count][color_idx]   = data.b;
                                g_lux[poll_count][color_idx] = data.lux;
                                g_cct[poll_count][color_idx] = data.cct;
                            }
                            color_idx++;
                        }
                        break;
                    }

                    case DeviceType::ICM20602: {
                        auto imu = static_cast<ICM20602Device *>(devices[i].get());
                        auto data = imu->get_data();
                        if (imu_idx < cfg.imu_cnt) {
                            if (!data.valid && poll_count > 0) {
                                g_acc_x[poll_count][imu_idx]       = g_acc_x[poll_count - 1][imu_idx];
                                g_acc_y[poll_count][imu_idx]       = g_acc_y[poll_count - 1][imu_idx];
                                g_acc_z[poll_count][imu_idx]       = g_acc_z[poll_count - 1][imu_idx];
                                g_gyro_x[poll_count][imu_idx]      = g_gyro_x[poll_count - 1][imu_idx];
                                g_gyro_y[poll_count][imu_idx]      = g_gyro_y[poll_count - 1][imu_idx];
                                g_gyro_z[poll_count][imu_idx]      = g_gyro_z[poll_count - 1][imu_idx];
                                g_temperature[poll_count][imu_idx] = g_temperature[poll_count - 1][imu_idx];
                                printf("IMU #%d data invalid and buffered at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                            }
                            else {
                                if (!imu->is_initialized()) {
                                    printf("IMU #%d not initialized and tried to get data at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                                }
                                g_acc_x[poll_count][imu_idx]       = data.acc.x;
                                g_acc_y[poll_count][imu_idx]       = data.acc.y;
                                g_acc_z[poll_count][imu_idx]       = data.acc.z;
                                g_gyro_x[poll_count][imu_idx]      = data.gyro_rps.x;
                                g_gyro_y[poll_count][imu_idx]      = data.gyro_rps.y;
                                g_gyro_z[poll_count][imu_idx]      = data.gyro_rps.z;
                                g_temperature[poll_count][imu_idx] = data.temperature;
                            }
                            imu_idx++;
                        }
                        break;
                    }

                    case DeviceType::VL53L4CD: {
                        auto distance_sensor = static_cast<VL53L4CDDevice *>(devices[i].get());
                        auto data = distance_sensor->get_data();
                        if (dist_idx < cfg.distance_sensor_cnt) {
                            if (!data.valid && poll_count > 0) {
                                g_distance_mm[poll_count][dist_idx]       = g_distance_mm[poll_count - 1][dist_idx];
                                g_range_status[poll_count][dist_idx]      = g_range_status[poll_count - 1][dist_idx];
                                g_sigma_mm[poll_count][dist_idx]          = g_sigma_mm[poll_count - 1][dist_idx];
                                g_signal_rate_kcps[poll_count][dist_idx]  = g_signal_rate_kcps[poll_count - 1][dist_idx];
                                g_ambient_rate_kcps[poll_count][dist_idx] = g_ambient_rate_kcps[poll_count - 1][dist_idx];
                                printf("Distance sensor #%d data invalid and buffered at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                            }
                            else {
                                if (!distance_sensor->is_initialized()) {
                                    printf("Distance sensor #%d not initialized and tried to get data at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                                }
                                g_distance_mm[poll_count][dist_idx]        = data.distance_mm;
                                g_range_status[poll_count][dist_idx]       = data.range_status;
                                g_sigma_mm[poll_count][dist_idx]           = data.sigma_mm;
                                g_signal_rate_kcps[poll_count][dist_idx]   = data.signal_rate_kcps;
                                g_ambient_rate_kcps[poll_count][dist_idx]  = data.ambient_rate_kcps;
                            }
                            dist_idx++;
                        }
                        break;
                    }

                    case DeviceType::TCA9536: {
                        auto io_expander = static_cast<TCA9536Device *>(devices[i].get());
                        auto data = io_expander->get_data();
                        if (gpio_idx < cfg.io_expander_cnt) {
                            if (!data.valid && poll_count > 0) {
                                g_gpio_state[poll_count][gpio_idx] = g_gpio_state[poll_count - 1][gpio_idx];
                                printf("IO expander #%d data invalid and buffered at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                            }
                            else {
                                if (!io_expander->is_initialized()) {
                                    printf("IO expander #%d not initialized and tried to get data at %lu ms\n", i + 1, (unsigned long)(poll_count * cfg.polling_interval_ms));
                                }
                                g_gpio_state[poll_count][gpio_idx] = data.gpio_state;
                            }
                            gpio_idx++;
                        }
                        break;
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(cfg.polling_interval_ms));
        }

        // ---- Transmit results ----------------------------------------------
        //
        // Build and send a CMD_DATA binary packet. The payload starts with a
        // DataHeader, then the colour/IMU/distance/GPIO frames row-major.

        const uint32_t n_polls = cfg.polling_times;
        const uint8_t  n_col   = cfg.color_sensor_cnt;
        const uint8_t  n_imu   = cfg.imu_cnt;
        const uint8_t  n_dist  = cfg.distance_sensor_cnt;
        const uint8_t  n_gpio  = cfg.io_expander_cnt;

        // Print human-readable debug summary over USB
        for (uint32_t p = 0; p < n_polls; p++) {
            for (uint8_t i = 0; i < n_col;  i++)
                printf("Poll %lu ms - Color Sensor #%d: R=%5u G=%5u B=%5u LUX=%6lu CCT=%6lu\n",
                    (unsigned long)(p * cfg.polling_interval_ms), i + 1,
                    g_r[p][i], g_g[p][i], g_b[p][i],
                    (unsigned long)g_lux[p][i], (unsigned long)g_cct[p][i]);

            for (uint8_t i = 0; i < n_imu;  i++)
                printf("Poll %lu ms - IMU #%d: Acc=(%.2f,%.2f,%.2f)g Gyro=(%.2f,%.2f,%.2f)rad/s Temp=%.2f C\n",
                    (unsigned long)(p * cfg.polling_interval_ms), i + 1,
                    g_acc_x[p][i], g_acc_y[p][i], g_acc_z[p][i],
                    g_gyro_x[p][i], g_gyro_y[p][i], g_gyro_z[p][i],
                    g_temperature[p][i]);

            for (uint8_t i = 0; i < n_dist; i++)
                printf("Poll %lu ms - Distance Sensor #%d: %4d mm  Status=%d  Sigma=%4d mm\n",
                    (unsigned long)(p * cfg.polling_interval_ms), i + 1,
                    g_distance_mm[p][i], g_range_status[p][i], g_sigma_mm[p][i]);

            for (uint8_t i = 0; i < n_gpio; i++)
                printf("Poll %lu ms - IO Expander #%d: GPIO=0b%04b\n",
                    (unsigned long)(p * cfg.polling_interval_ms), i + 1,
                    g_gpio_state[p][i] & 0xF);
        }

        // Send binary CMD_DATA packet to Jetson.
        // Payload: DataHeader + sensor frames packed row-major.
        {
            static uint8_t tx_buf[sizeof(DataHeader)
                + MAX_POLLS * MAX_UNIQUES_DEVICES * (sizeof(ColorFrame) + sizeof(ImuFrame) + sizeof(DistanceFrame) + sizeof(GpioFrame))];

            DataHeader hdr = {n_polls, n_col, n_imu, n_dist, n_gpio};
            uint16_t offset = 0;
            memcpy(tx_buf + offset, &hdr, sizeof(hdr)); offset += sizeof(hdr);

            for (uint32_t p = 0; p < n_polls; p++) {
                for (uint8_t i = 0; i < n_col; i++) {
                    ColorFrame f = {g_r[p][i], g_g[p][i], g_b[p][i], g_lux[p][i], g_cct[p][i]};
                    memcpy(tx_buf + offset, &f, sizeof(f)); offset += sizeof(f);
                }
                for (uint8_t i = 0; i < n_imu; i++) {
                    ImuFrame f = {g_acc_x[p][i], g_acc_y[p][i], g_acc_z[p][i],
                                  g_gyro_x[p][i], g_gyro_y[p][i], g_gyro_z[p][i],
                                  g_temperature[p][i]};
                    memcpy(tx_buf + offset, &f, sizeof(f)); offset += sizeof(f);
                }
                for (uint8_t i = 0; i < n_dist; i++) {
                    DistanceFrame f = {g_distance_mm[p][i], g_range_status[p][i],
                                       g_sigma_mm[p][i], g_signal_rate_kcps[p][i],
                                       g_ambient_rate_kcps[p][i]};
                    memcpy(tx_buf + offset, &f, sizeof(f)); offset += sizeof(f);
                }
                for (uint8_t i = 0; i < n_gpio; i++) {
                    GpioFrame f = {g_gpio_state[p][i]};
                    memcpy(tx_buf + offset, &f, sizeof(f)); offset += sizeof(f);
                }
            }

            comms_send(CMD_DATA, tx_buf, offset);
        }

        reset_device_array();
    }
}

// ---------------------------------------------------------------------------
// main — hardware init then hand off to FreeRTOS
// ---------------------------------------------------------------------------

int main() {
    stdio_init_all();
    sleep_ms(500);
    init_i2c_buses();

    printf("polling_firmware: FreeRTOS starting\n");

    g_polling_sem  = xSemaphoreCreateCounting(8, 0);
    g_config_mutex = xSemaphoreCreateMutex();

    xTaskCreate(comms_task,   "comms",   STACK_COMMS,   nullptr, 3, nullptr);
    xTaskCreate(polling_task, "polling", STACK_POLLING, nullptr, 2, nullptr);

    vTaskStartScheduler();

    for (;;);  // unreachable
}
