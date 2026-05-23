/*
Original IMU driver code by Martin Budden: https://github.com/martinbudden/Library-Sensors
*/

#include "imu_icm20602.h"
#include <stdio.h>
#define LIBRARY_SENSORS_SERIAL_DEBUG
// #if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
// #include <HardwareSerial.h>
// #endif

#include <cassert>


namespace { // use anonymous namespace to make items local to this translation unit

constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };

constexpr uint8_t REG_XG_OFFS_TC_H          = 0x04;
constexpr uint8_t REG_XG_OFFS_TC_L          = 0x05;
constexpr uint8_t REG_YG_OFFS_TC_H          = 0x07;
constexpr uint8_t REG_YG_OFFS_TC_L          = 0x08;
constexpr uint8_t REG_ZG_OFFS_TC_H          = 0x0A;
constexpr uint8_t REG_ZG_OFFS_TC_L          = 0x0B;

constexpr uint8_t REG_SELF_TEST_X_ACCEL     = 0x0D;
constexpr uint8_t REG_SELF_TEST_Y_ACCEL     = 0x0E;
constexpr uint8_t REG_SELF_TEST_Z_ACCEL     = 0x0F;

//  GYRO OFFSET ADJUSTMENT REGISTERS
constexpr uint8_t REG_XG_OFFS_USRH          = 0x13;
constexpr uint8_t REG_XG_OFFS_USRL          = 0x14;
constexpr uint8_t REG_YG_OFFS_USRH          = 0x15;
constexpr uint8_t REG_YG_OFFS_USRL          = 0x16;
constexpr uint8_t REG_ZG_OFFS_USRH          = 0x17;
constexpr uint8_t REG_ZG_OFFS_USRL          = 0x18;

constexpr uint8_t REG_SAMPLE_RATE_DIVIDER   = 0x19;
    constexpr uint8_t DIVIDE_BY_1 = 0x00;
    constexpr uint8_t DIVIDE_BY_2 = 0x01;
constexpr uint8_t REG_CONFIG                = 0x1A;
    // Update rate: 1kHz, Filter:176 3-DB BW (Hz), default value used by M5Stack, the least filtered 1kHz update variant
    constexpr uint8_t DLPF_CFG_1 = 0x01;
    // Update rate: 8kHz, Filter:3281 3-DB BW (Hz) - only non-32kHz variant less filtered than DLPF_CFG_1
    constexpr uint8_t DLPF_CFG_7 = 0x07;
constexpr uint8_t REG_GYRO_CONFIG           = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG          = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2         = 0x1D;

constexpr uint8_t REG_FIFO_ENABLE           = 0x23;
    constexpr uint8_t GYRO_FIFO_EN = 0b00001000;
    constexpr uint8_t ACC_FIFO_EN  = 0b00000100;

constexpr uint8_t REG_INT_PIN_CFG           = 0x37;
constexpr uint8_t REG_INT_ENABLE            = 0x38;
constexpr uint8_t FIFO_WM_INT_STATUS        = 0x39; // FIFO watermark interrupt status

constexpr uint8_t REG_ACCEL_XOUT_H          = 0x3B;
constexpr uint8_t REG_ACCEL_XOUT_L          = 0x3C;
constexpr uint8_t REG_ACCEL_YOUT_H          = 0x3D;
constexpr uint8_t REG_ACCEL_YOUT_L          = 0x3E;
constexpr uint8_t REG_ACCEL_ZOUT_H          = 0x3F;
constexpr uint8_t REG_ACCEL_ZOUT_L          = 0x40;

constexpr uint8_t REG_TEMP_OUT_H            = 0x41;
constexpr uint8_t REG_TEMP_OUT_L            = 0x42;

constexpr uint8_t REG_GYRO_XOUT_H           = 0x43;
constexpr uint8_t REG_GYRO_XOUT_L           = 0x44;
constexpr uint8_t REG_GYRO_YOUT_H           = 0x45;
constexpr uint8_t REG_GYRO_YOUT_L           = 0x46;
constexpr uint8_t REG_GYRO_ZOUT_H           = 0x47;
constexpr uint8_t REG_GYRO_ZOUT_L           = 0x48;

constexpr uint8_t REG_FIFO_WM_TH1           = 0x60; // FIFO watermark threshold in number of bytes
constexpr uint8_t REG_FIFO_WM_TH2           = 0x61;

constexpr uint8_t REG_SIGNAL_PATH_RESET     = 0x68;
constexpr uint8_t REG_ACCEL_INTEL_CTRL      = 0x69;
constexpr uint8_t REG_USER_CTRL             = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1            = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2            = 0x6C;

constexpr uint8_t REG_FIFO_COUNT_H          = 0x72;
constexpr uint8_t REG_FIFO_COUNT_L          = 0x73;
constexpr uint8_t REG_FIFO_R_W              = 0x74;

constexpr uint8_t REG_WHO_AM_I              = 0x75;

// ACCELEROMETER OFFSET REGISTERS
constexpr uint8_t REG_XA_OFFSET_H           = 0x77;
constexpr uint8_t REG_XA_OFFSET_L           = 0x78;
constexpr uint8_t REG_YA_OFFSET_H           = 0x7A;
constexpr uint8_t REG_YA_OFFSET_L           = 0x7B;
constexpr uint8_t REG_ZA_OFFSET_H           = 0x7D;
constexpr uint8_t REG_ZA_OFFSET_L           = 0x7E;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)

#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuIcm20602_USE_SPI_BUS)
ImuIcm20602::ImuIcm20602(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
ImuIcm20602::ImuIcm20602(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
#else
ImuIcm20602::ImuIcm20602(uint8_t axis_order, I2CBus* i2c, const BusI2c::i2c_pins_t& pins, uint8_t i2c_address) :
    ImuBase(axis_order, _bus),
    _bus(i2c_address, i2c, pins)
{
}
#endif

int ImuIcm20602::init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex)
{
    (void)target_output_data_rate_hz;
    (void)gyro_sensitivity;
    (void)acc_sensitivity;

    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temperature_gyro_data_t) == acc_temperature_gyro_data_t::DATA_SIZE);

#if defined(FRAMEWORK_USE_FREERTOS)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    _bus_mutex = bus_mutex;
#endif

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyro_id_msp = MSP_GYRO_ID_DEFAULT;
    _acc_id_msp = MSP_ACC_ID_DEFAULT;

    _bus.set_device_data_register(REG_GYRO_XOUT_H, reinterpret_cast<uint8_t*>(&_spi_acc_temperature_gyro_data), sizeof(_spi_acc_temperature_gyro_data));

    bus_semaphore_take(_bus_mutex);

    const uint8_t chip_id = _bus.read_register(REG_WHO_AM_I);
    delay_ms(1);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    // printf("IMU init, chip_id=%02x\r\n", chip_id);
#else
    (void)chip_id;
#endif

    int pwr_mgmt_clr = _bus.write_register(REG_PWR_MGMT_1, 0); // clear the power management register
    // printf("Clear power management register: %d\n", pwr_mgmt_clr);
    if (pwr_mgmt_clr != 2) {
        return -1;
    }
    delay_ms(10);

    constexpr uint8_t DEVICE_RESET = 0x01U << 7U;
    int device_reset = _bus.write_register(REG_PWR_MGMT_1, DEVICE_RESET); // reset the device
    // printf("Device reset: %d\n", device_reset);
    if (device_reset != 2) {
        return -2;
    }
    delay_ms(10);

    while (_bus.read_register(REG_PWR_MGMT_1) != 0) {
        // wait for the reset bit to clear, indicating reset is complete
    }
    // printf("Reset complete\r\n");
    
    int reset_buffer_1 = _bus.write_register(REG_PWR_MGMT_1, 0);
    // printf("Post-reset write buffer 1: %d\n", reset_buffer_1);
    delay_ms(10);

    int reset_buffer_2 = _bus.write_register(REG_PWR_MGMT_1, 0);
    // printf("Post-reset write buffer 2: %d\n", reset_buffer_2);
    delay_ms(10);

    int reset_buffer_3 = _bus.write_register(REG_PWR_MGMT_1, 0);
    // printf("Post-reset write buffer 3: %d\n", reset_buffer_3);
    delay_ms(10);

    constexpr uint8_t ACC_FCHOICE_B = 0x00; // Filter:218.1 3-DB BW (Hz), least filtered 1kHz update variant
    int acc_config_fchoice = _bus.write_register(REG_ACCEL_CONFIG2, ACC_FCHOICE_B);
    // printf("Configure acceleration filter choice: %d\n", acc_config_fchoice);
    if (acc_config_fchoice != 2) {
        return -3;
    }

    delay_ms(1);

    constexpr uint8_t FIFO_MODE_OVERWRITE = 0b01000000;
    int set_fifo_mode = _bus.write_register(REG_CONFIG, DLPF_CFG_1 | FIFO_MODE_OVERWRITE);
    // printf("Set FIFO mode: %d\n", set_fifo_mode);
    if (set_fifo_mode != 2) {
        return -4;
    }
    delay_ms(1);

    // M5Stack default divider is two, giving 500Hz output rate
    int set_sample_rate_div = _bus.write_register(REG_SAMPLE_RATE_DIVIDER, DIVIDE_BY_2);
    // printf("Set sampling rate divider: %d\n", set_sample_rate_div);
    if (set_sample_rate_div != 2) {
        return -5;
    }
    delay_ms(1);
    _gyro_sample_rate_hz = 500;
    _acc_sample_rate_hz = 500;

    int disable_fifo = _bus.write_register(REG_FIFO_ENABLE, 0x00); // FIFO disabled
    // printf("Disable FIFO: %d\n", disable_fifo);
    if (disable_fifo != 2) {
        return -6;
    }
    delay_ms(1);

    // M5 Unified settings
    //_bus.write_register(REG_INT_PIN_CFG, 0b11000000); // Active low, open drain 50us pulse width, clear on read
    int int_config = _bus.write_register(REG_INT_PIN_CFG, 0x22);
    // printf("Configure interrupt: %d\n", int_config);
    if (int_config != 2) {
        return -7;
    }
    delay_ms(1);

    constexpr uint8_t DATA_RDY_INT_EN = 0x01;
    int int_enable = _bus.write_register(REG_INT_ENABLE, DATA_RDY_INT_EN); // data ready interrupt enabled
    // printf("Enable data ready interrupt: %d\n", int_enable);
    if (int_enable != 2) {
        return -8;
    }
    delay_ms(10);

    int user_ctrl = _bus.write_register(REG_USER_CTRL, 0x00);
    // printf("Set user control: %d\n", user_ctrl);
        if (user_ctrl != 2) {
        return -9;
    }

     // datasheet sect 4.24: OUTPUT_LIMIT bit must be set to 1 at every power-up to avoid capping output at 0x7FFF
    constexpr uint8_t OUTPUT_LIMIT = 0x02;
    int set_output_limit = _bus.write_register(REG_ACCEL_INTEL_CTRL, OUTPUT_LIMIT);
    // printf("Set output limit bit to 1: %d\n", set_output_limit);
    if (set_output_limit != 2) {
        return -10;
    }
    delay_ms(1);

    // Gyro scale is fixed at 2000DPS, the maximum supported.
    enum gyro_scale_e { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };
    constexpr uint8_t GYRO_FCHOICE_B = 0x00; // enables gyro update rate and filter configuration using REG_CONFIG
    int config_gyro = _bus.write_register(REG_GYRO_CONFIG, (GFS_2000DPS << 3) | GYRO_FCHOICE_B); // cppcheck-suppress badBitmaskCheck
    // printf("Configure gyroscope: %d\n", config_gyro);
    if (config_gyro != 2) {
        return -11;
    }
    _gyro_resolution_dps = GYRO_2000DPS_RES;
    _gyro_resolution_rps = GYRO_2000DPS_RES * DEGREES_TO_RADIANS;
    delay_ms(1);

    // Accelerometer scale is fixed at 8G, the maximum supported.
    enum acc_scale_e { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };
    int fs_sel = _bus.write_register(REG_ACCEL_CONFIG, AFS_8G << 3U);
    // printf("Set acceleratometer full scale: %d\n", fs_sel);
    if (fs_sel != 2) {
        return -12;
    }
    _acc_resolution = ACC_8G_RES;
    delay_ms(1);

    constexpr uint8_t CLKSEL_1 = 0x01;
    int set_clk = _bus.write_register(REG_PWR_MGMT_1, CLKSEL_1); // CLKSEL must be set to 001 to achieve full gyroscope performance.
    // printf("Set clock 3: %d\n", set_clk);
    if (set_clk != 2) {
        return -13;
    }
    delay_ms(10);

    bus_semaphore_give(_bus_mutex);
    delay_ms(1);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyro_sample_rate_hz);
}

void ImuIcm20602::set_interrupt_driven()
{
    // set interrupt level as configured in init()
    _bus.set_interrupt_driven(BusBase::IRQ_EDGE_RISE);
}

ImuBase::xyz_int32_t ImuIcm20602::read_gyro_raw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give(_bus_mutex);

    xyz_int32_t ret {
        .x = static_cast<int16_t>((gyro.value.x_h << 8U) | gyro.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((gyro.value.y_h << 8U) | gyro.value.y_l),
        .z = static_cast<int16_t>((gyro.value.z_h << 8U) | gyro.value.z_l)
    };
    return ret;
}

ImuBase::xyz_int32_t ImuIcm20602::read_acc_raw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    bus_semaphore_give(_bus_mutex);

     return xyz_int32_t {
        .x = static_cast<int16_t>((acc.value.x_h << 8U) | acc.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((acc.value.y_h << 8U) | acc.value.y_l),
        .z = static_cast<int16_t>((acc.value.z_h << 8U) | acc.value.z_l)
    };
}

xyz_t ImuIcm20602::read_gyro_rps()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give(_bus_mutex);

    return gyro_rps_from_raw(gyro.value);
}

xyz_t ImuIcm20602::read_gyro_dps()
{
    return read_gyro_rps() * RADIANS_TO_DEGREES;
}

xyz_t ImuIcm20602::read_acc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    bus_semaphore_give(_bus_mutex);

    return acc_from_raw(acc.value);
}

FAST_CODE acc_gyro_rps_t ImuIcm20602::read_acc_gyro_rps()
{
    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_XOUT_H, &_spi_acc_temperature_gyro_data.accGyro.data[0], sizeof(_spi_acc_temperature_gyro_data.accGyro));
    // _bus.read_device_data();
    bus_semaphore_give(_bus_mutex);
    
    return acc_gyro_rps_from_raw(_spi_acc_temperature_gyro_data.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE acc_gyro_rps_t ImuIcm20602::get_acc_gyro_rps() const
{
    return acc_gyro_rps_from_raw(_spi_acc_temperature_gyro_data.accGyro.value);
}

int32_t ImuIcm20602::read_temperature_raw() const
{
    std::array<uint8_t, 2> data;

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_TEMP_OUT_H, &data[0], sizeof(data));
    bus_semaphore_give(_bus_mutex);

    const int32_t temperature = static_cast<int16_t>((data[0] << 8U) | data[1]); // NOLINT(hicpp-use-auto,modernize-use-auto)
    return temperature;
}

float ImuIcm20602::read_temperature() const
{
    const int32_t temperature = read_temperature_raw();

    return static_cast<float>(temperature) / 326.8F + 25.0F; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

xyz_t ImuIcm20602::gyro_rps_from_raw(const mems_sensor_data_t::value_t& data) const
{
    // static cast to int16_t to sign extend the 8 bit values
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .y =   static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -(static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y),
        .y =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .y = -(static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x),
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .y =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z,
        .z = -(static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y)
    };
#else
    const xyz_t gyro {
        .x =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .y =   static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
    return map_axes(gyro);
#endif
}

xyz_t ImuIcm20602::acc_from_raw(const mems_sensor_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .y =   static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -(static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y),
        .y =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .y = -(static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x),
        .z =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =   static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .y =   static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z,
        .z = -(static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y)
    };
#else
    const xyz_t acc = {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
    return map_axes(acc);
#endif
}

acc_gyro_rps_t ImuIcm20602::acc_gyro_rps_from_raw(const acc_temperature_gyro_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyro_rps = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyro_rps = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x),
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .y = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x),
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyro_rps = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x),
            .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z)
        },
        .acc = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x),
            .y = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z)
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyro_rps = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XPOS_ZNEG) || defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS_NED)
    return acc_gyro_rps_t {
        .gyro_rps = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z)
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .z = -(static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z)
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return acc_gyro_rps_t {
        .gyro_rps = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z,
            .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y)
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z,
            .z = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y)
        }
    };
#else
    // Axis order mapping done at run-time
    // return values are fully calculated for each `case` to allow eliding copy on return (return value optimization, RVO)
    // speed optimization is more important than size optimization here
    switch (_axis_order) {
    case XPOS_YPOS_ZPOS: // NOLINT(bugprone-branch-clone)
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
                .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case YPOS_XNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x),
                .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
                .y = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x),
                .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case XNEG_YNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x),
                .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
                .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x),
                .y = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
                .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case YNEG_XPOS_ZPOS:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
                .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
                .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case XPOS_YNEG_ZNEG:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
                .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
                .z = -(static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z)
            }
        };
    case YPOS_XPOS_ZNEG:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.x,
                .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.y,
                .z = -(static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z)
            }
        };
    case XNEG_YPOS_ZNEG:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.x),
                .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.x),
                .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.y,
                .z = -(static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z)
            }
        };
    case YNEG_XNEG_ZNEG:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.x),
                .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.y),
                .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.x),
                .y = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.y),
                .z = -(static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z)
            }
        };
    case XPOS_ZPOS_YNEG:
        return acc_gyro_rps_t {
            .gyro_rps = {
                .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.y,
                .z = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.z)
            }
        };
    default:
        // default uses map_axes() function
        return acc_gyro_rps_t {
            .gyro_rps = map_axes({
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            }),
            .acc = map_axes({
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            })
        };
        break;
    } // end switch
#endif
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)
