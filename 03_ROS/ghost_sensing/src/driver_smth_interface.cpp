#include "driver_smth_interface.h"
#include <cstdarg>
#include <string.h>

// todo, use C++ functions better here instead of being in C
namespace ghost_sensing
{

  uint8_t[][4] sensors = {
      {0x29, 0x29, 0x29, 0x29},
      {0x39, 0x39, 0x39, 0x39},
  };
  uint8_t smth_i2c_interface::check_sensors()
  {
    uint8_t id;
    for (int i = 0; i < sizeof(sensors); i++)
    {
      smth_i_i2c_interface::read(sensors[i][0], sensors[i][1], sensors[i][2], sensors[i][3]);
    }
    return 0;
  }

  /**
   * @brief  interface iic bus init
   * @return status codes
   *         - 0 success
   *         - 1 iic init failed
   * @note   none
   */
  uint8_t smth_i2c_interface::init()
  {
    // std::cout << "opening " << filename << std::endl;
    gs_fd = open(filename.c_str(), O_RDWR);
    if (gs_fd < 0)
    {
      perror("Failed to open the i2c bus");
      return 1;
    }
    // std::cout << "gs_fd " << gs_fd << std::endl;
    return 0;
  }

  /**
   * @brief  interface iic bus deinit
   * @return status code
   *         - 0 success
   *         - 1 iic deinit failed
   * @note   none
   */
  uint8_t tcs_i2c_interface::deinit(void)
  {

    if (close(gs_fd) < 0)
    {
      perror("Failed to close the i2c bus");
      return 1;
    }
    gs_fd = -1;
    return 0;
  }

  /**
   * @brief      interface iic bus read
   * @param[in]  addr iic device write address
   * @param[in]  reg iic register address
   * @param[out] *buf pointer to a data buffer
   * @param[in]  len length of the data buffer
   * @return     status code
   *             - 0 success
   *             - 1 read failed
   * @note       none
   */
  uint8_t tcs_i2c_interface::read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
  {
    if (ioctl(gs_fd, I2C_SLAVE, addr) < 0)
    {
      perror("Failed to set I2C address");
      return 1;
    }
    int writeout = ::write(gs_fd, &reg, 1);
    if (writeout != 1)
    {
      perror("Failed to write register address");
      // std::cout << gs_fd << " " << writeout << std::endl;
      return 1;
    }

    if (::read(gs_fd, buf, len) != len)
    {
      perror("Failed to read data");
      return 1;
    }

    return 0;
  }

  /**
   * @brief     interface iic bus write
   * @param[in] addr iic device write address
   * @param[in] reg iic register address
   * @param[in] *buf pointer to a data buffer
   * @param[in] len length of the data buffer
   * @return    status code
   *            - 0 success
   *            - 1 write failed
   * @note      none
   */
  uint8_t tcs_i2c_interface::write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
  {
    if (ioctl(gs_fd, I2C_SLAVE, addr) < 0)
    {
      perror("Failed to set I2C address");
      return 1;
    }

    uint8_t data[len + 1];
    data[0] = reg;
    for (uint16_t i = 0; i < len; i++)
    {
      data[i + 1] = buf[i];
    }

    if (::write(gs_fd, data, len + 1) != (len + 1))
    {
      perror("Failed to write data");
      return 1;
    }

    return 0;
  }

  /**
   * @brief     interface delay ms
   * @param[in] ms time
   * @note      none
   */
  void tcs_i2c_interface::delay_ms(uint32_t ms)
  {
    usleep(1000 * ms);
  }

  /**
   * @brief     interface print format data
   * @param[in] fmt format data
   * @note      none
   */
  void tcs_i2c_interface::debug_print(const char *const fmt, ...)
  {
    va_list args;
    va_start(args, fmt);

    // Create a new format string with the prefix.
    std::string full_fmt = std::string("tcs34725_driver: ") + fmt;

    // Define a fixed-size stack buffer.
    char buffer[256];

    // Format the message into the stack buffer.
    // If the formatted message is longer than STACK_BUFFER_SIZE, it will be truncated.
    vsnprintf(buffer, 256, full_fmt.c_str(), args);

    // Log the formatted message.
    RCLCPP_DEBUG(logger, "%s", buffer);
    va_end(args);
  }
}
