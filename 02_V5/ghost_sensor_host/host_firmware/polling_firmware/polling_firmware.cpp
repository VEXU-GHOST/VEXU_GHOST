#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "inc/isl29124.h"
#include "src/open_source_drivers/icm20602/inc/icm20602.h"
#include "inc/VL53L4CD/platform.h"
#include "inc/VL53L4CD/VL53L4CD_api.h"
#include "inc/VL53L4CD/VL53L4CD_calibration.h"
#include "src/open_source_drivers/Library-Sensors/src/imu_null.h"
#include "src/open_source_drivers/Library-Sensors/src/imu_icm20602.h"
#include "src/open_source_drivers/Library-Sensors/src/Targets.h"
#include "src/open_source_drivers/Library-Sensors/src/bus_base.h"

#define I2C_PORT_0 i2c0
#define I2C_PORT_1 i2c1
#define SDA_INPUT_1 2
#define SCL_INPUT_1 3
#define SDA_INPUT_2 4
#define SCL_INPUT_2 5
#define SDA_INPUT_3 6
#define SCL_INPUT_3 7
#define SDA_INPUT_4 8
#define SCL_INPUT_4 9
#define SDA_INPUT_5 0
#define SCL_INPUT_5 1
#define SDA_INPUT_6 28
#define SCL_INPUT_6 29
#define SDA_INPUT_7 26
#define SCL_INPUT_7 27
#define SDA_INPUT_8 18
#define SCL_INPUT_8 19
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 1
#define UART_RX_PIN 0
#define I2C_FREQ_HZ 400000
#define TESTING 0
#define VL53L4CD_I2C_ADDR 0x29
#define ICM_20602_I2C_ADDR 0x68

int main()
{
    stdio_init_all();
    sleep_ms(1000);

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT_0, I2C_FREQ_HZ);
    i2c_init(I2C_PORT_1, I2C_FREQ_HZ);
    
    gpio_set_function(SDA_INPUT_1, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_1, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_1);
    gpio_pull_up(SCL_INPUT_1);
    gpio_set_function(SDA_INPUT_2, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_2, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_2);
    gpio_pull_up(SCL_INPUT_2);
    gpio_set_function(SDA_INPUT_3, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_3, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_3);
    gpio_pull_up(SCL_INPUT_3);
    gpio_set_function(SDA_INPUT_4, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_4, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_4);
    gpio_pull_up(SCL_INPUT_4);
    gpio_set_function(SDA_INPUT_6, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_6, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_6);
    gpio_pull_up(SCL_INPUT_6);
    gpio_set_function(SDA_INPUT_7, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_7, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_7);
    gpio_pull_up(SCL_INPUT_7);
    gpio_set_function(SDA_INPUT_8, GPIO_FUNC_I2C);
    gpio_set_function(SCL_INPUT_8, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_INPUT_8);
    gpio_pull_up(SCL_INPUT_8);

    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // // Init UART if in testing mode (TESTING == 1), otherwise init I2C on input 5
    // // Set the TX and RX pins by using the function select on the GPIO
    // // Set datasheet for more information on function select
    if (TESTING) {
        uart_init(UART_ID, BAUD_RATE);
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
        uart_puts(UART_ID, " Hello, UART!\n");
    }
    else {
        gpio_set_function(SDA_INPUT_5, GPIO_FUNC_I2C);
        gpio_set_function(SCL_INPUT_5, GPIO_FUNC_I2C);
        gpio_pull_up(SDA_INPUT_5);
        gpio_pull_up(SCL_INPUT_5);
    }
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    // Send out a string, with CR/LF conversions
    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart

    // Color sensor initialisation
    isl29124_t color_sensor;
    if (!isl29124_init(&color_sensor, I2C_PORT_0, ISL29124_I2C_ADDR)) {
        printf("ISL29124 not found — check wiring!\n");
        while (1) tight_loop_contents();
    }
    else {
        printf("ISL29124 found and initialised!\n");
    }

    // IMU initialisation
    // icm20602_dev_t imu = ICM20602_DEFAULT_INIT();
    // if (!icm20602_init(&imu)) {
    //     printf("ICM20602 not found — check wiring!\n");
    //     while (1) tight_loop_contents();
    // }
    // else {
    //     printf("ICM20602 found and initialised!\n");
    // }

    static ImuBase* imu = &ImuIcm20602(ImuBase::XPOS_YPOS_ZPOS, BusBase::BUS_INDEX_0, BusI2c::IMU_I2C_PINS, ICM_20602_I2C_ADDR);
    imu->init();
    printf("IMU found and initialised!\n");

    // Distance sensor initialisation
    vl53l4cd_dev_t distance_sensor;
    if (vl53l4cd_init(distance_sensor, I2C_PORT_0, VL53L4CD_I2C_ADDR)) {
        printf("VL53L4CD not found — check wiring!\n");
        while (1) tight_loop_contents();
    }
    else {
        printf("VL53L4CD found and initialised!\n");
    }
    /* ---- Configure timing: 50 ms budget, continuous mode ---- */
    printf("SetRangeTiming status: %u\n", VL53L4CD_SetRangeTiming(distance_sensor, 50, 0));
     /* ---- Start continuous ranging ---- */
    printf("StartRanging status: %u\n", VL53L4CD_StartRanging(distance_sensor));

    // main program loop

    while (true) {
        printf("Hello, world!\n");

        // Read RGB values from the color sensor
        uint16_t r, g, b;
        if (isl29124_read_rgb(&color_sensor, &r, &g, &b) < 0) {
            printf("Read error\n");
        } else {
            // Update range based on green saturation
            isl29124_autorange(&color_sensor, g);
 
            // Calculate lux and CCT
            int cct = 0;
            uint32_t lux = isl29124_cal_lux(&color_sensor, &cct);
 
            printf("R=%5u  G=%5u  B=%5u  |  LUX=%6lu  CCT=%5dK\n",
                   r, g, b, (unsigned long)lux, cct);
        }

        // Read accelerometer, gyroscope, and temperature values from the IMU
        // float ax, ay, az, gx, gy, gz, t;
        // if (icm20602_read_data(&imu, &ax, &ay, &az, &gx, &gy, &gz, &t) < 0) {
        //     printf("Read error\n");
        // } else {
        //     printf("Accel: %6d %6d %6d | Gyro: %6d %6d %6d | Temp: %6d\n",
        //            ax, ay, az, gx, gy, gz, t);
        // }
        
        const acc_gyro_rps_t acc_gyro_rps =  imu->get_acc_gyro_rps();

        // convert the gyro data from radians per second to degrees per second
        const xyz_t gyro_dps =  acc_gyro_rps.gyro_rps * ImuBase::RADIANS_TO_DEGREES;
        const xyz_t acc =  acc_gyro_rps.acc;
        printf("Gyro (dps): %6.1f %6.1f %6.1f | Acc (g): %6.3f %6.3f %6.3f\n",
               gyro_dps.x, gyro_dps.y, gyro_dps.z, acc.x, acc.y, acc.z);
        static const ImuNull XPOS_ZPOS_YNEG(ImuBase::XPOS_ZPOS_YNEG);
        const xyz_t output = XPOS_ZPOS_YNEG.map_axes(acc);
        printf("Output (g): %6.3f %6.3f %6.3f\n", output.x, output.y, output.z);

        // Read distance value from the distance sensor
        uint8_t data_ready = 0;
        VL53L4CD_ResultsData_t results;
        VL53L4CD_CheckForDataReady(distance_sensor, &data_ready);
        if (data_ready) {
            VL53L4CD_GetResult(distance_sensor, &results);
            VL53L4CD_ClearInterrupt(distance_sensor);
             printf("Status: %3u  Distance: %5u mm  Signal: %5lu kcps  "
               "Ambient: %5lu kcps  Sigma: %3u mm  SPADs: %3u\n",
               results.range_status,
               results.distance_mm,
               (unsigned long)results.signal_rate_kcps,
               (unsigned long)results.ambient_rate_kcps,
               results.sigma_mm,
               results.number_of_spad);
        }

        // Sleep for a bit before the next reading
        sleep_ms(200);
    }
}
