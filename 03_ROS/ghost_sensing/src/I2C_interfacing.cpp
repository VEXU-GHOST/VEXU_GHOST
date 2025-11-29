#include "I2C_interfacing.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <string.h>


// todo, use C++ functions better here instead of being in C
namespace ghost_sensing
{
/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t I2C_interfacing::init()
{
  //std::cout << "opening " << filename << std::endl;
  gs_fd = open(filename.c_str(), O_RDWR);
  if (gs_fd < 0) {
    perror("Failed to open the i2c bus");
    return 1;
  }
  //std::cout << "gs_fd " << gs_fd << std::endl;
  return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t I2C_interfacing::deinit(void)
{

  if (close(gs_fd) < 0) {
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
uint8_t I2C_interfacing::read(uint8_t reg, uint8_t * buf, uint16_t len)
{
  if (len > 0U && buf == nullptr) {
    RCLCPP_ERROR(logger, "Read buffer is null while requesting %u bytes", len);
    return 1;
  }

  if (ioctl(gs_fd, I2C_SLAVE, addr) < 0) {
    perror("Failed to set I2C address");
    return 1;
  }
  int writeout = ::write(gs_fd, &reg, 1);
  if (writeout != 1) {
    perror("Failed to write register address");
    //std::cout << gs_fd << " " << writeout << std::endl;
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
uint8_t I2C_interfacing::write(uint8_t reg, uint8_t * buf, uint16_t len)
{
  if (len > 0U && buf == nullptr) {
    RCLCPP_ERROR(logger, "Write buffer is null while attempting to send %u bytes", len);
    return 1;
  }

  if (ioctl(gs_fd, I2C_SLAVE, addr) < 0) {
    perror("Failed to set I2C address");
    return 1;
  }

  uint8_t data[len + 1];
  data[0] = reg;
  for (uint16_t i = 0; i < len; i++) {
    data[i + 1] = buf[i];
  }

  if (::write(gs_fd, data, len + 1) != (len + 1)) {
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
void I2C_interfacing::delay_ms(uint32_t ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void I2C_interfacing::debug_print(const char * const fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  // Create a new format string with the prefix.
  std::string full_fmt = std::string("tcs34725_driver: ") + fmt;

  // Define a fixed-size stack buffer.
  std::array<char, 256> buffer{};

  // Format the message into the stack buffer.
  // If the formatted message is longer than STACK_BUFFER_SIZE, it will be truncated.
  vsnprintf(buffer.data(), buffer.size(), full_fmt.c_str(), args);

  // Log the formatted message.
  RCLCPP_DEBUG(logger, "%s", buffer.data());
  va_end(args);
}
}
