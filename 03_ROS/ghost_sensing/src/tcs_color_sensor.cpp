/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_tcs34725_interface_template.c
 * @brief     driver tcs34725 interface template source file
 * @version   2.0.0
 * @author    Shifeng Li
 * @date      2021-02-28
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/02/28  <td>2.0      <td>Shifeng Li  <td>format the code
 * <tr><td>2020/10/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include <ghost_sensing/tcs_color_sensor.hpp>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "driver_tcs34725_interface.h"
#include "driver_tcs34725.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>

#define TCS34725_BASIC_DEFAULT_RGBC_INTERRUPT                        TCS34725_BOOL_FALSE                                       /**< disable interrupt */
#define TCS34725_BASIC_DEFAULT_WAIT                                  TCS34725_BOOL_TRUE                                        /**< enable wait */
#define TCS34725_BASIC_DEFAULT_RGBC                                  TCS34725_BOOL_TRUE                                        /**< enable rgbc */
#define TCS34725_BASIC_DEFAULT_INTEGRATION_TIME                      TCS34725_INTEGRATION_TIME_50MS                            /**< 50 ms integration time */
#define TCS34725_BASIC_DEFAULT_WAIT_TIME                             TCS34725_WAIT_TIME_2P4MS                                  /**< 2.4 ms wait time */
#define TCS34725_BASIC_DEFAULT_CLEAR_LOW_INTERRUPT_THRESHOLD         0x0000U                                                   /**< 0x0000 low interrupt threshold */
#define TCS34725_BASIC_DEFAULT_CLEAR_HIGH_INTERRUPT_THRESHOLD        0xFFFFU                                                   /**< 0xFFFF high interrupt threshold */
#define TCS34725_BASIC_DEFAULT_GAIN                                  TCS34725_GAIN_16X                                         /**< 16x gain */
#define TCS34725_BASIC_DEFAULT_INTERRUPT_MODE                        TCS34725_INTERRUPT_MODE_1_CLEAR_CHANNEL_OUT_OF_THRESHOLD  /**< 1 clear out of threshold */
#define TCS34725_BASIC_DEFAULT_POWER_ON                              TCS34725_BOOL_TRUE                                        /**< enable power on */


#define IIC_DEVICE_NAME "/dev/i2c-7" /**< iic device name */
static int gs_fd = -1;               /**< file descriptor */

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t tcs34725_interface_iic_init(void)
{
  std::cout << "opening " << IIC_DEVICE_NAME << std::endl;
  gs_fd = open(IIC_DEVICE_NAME, O_RDWR);
  if (gs_fd < 0)
  {
    perror("Failed to open the i2c bus");
    return 1;
  }
  std::cout << "gs_fd " << gs_fd << std::endl;
  return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t tcs34725_interface_iic_deinit(void)
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
uint8_t tcs34725_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (ioctl(gs_fd, I2C_SLAVE, addr) < 0)
  {
    perror("Failed to set I2C address");
    return 1;
  }
  int writeout = write(gs_fd, &reg, 1);
  if (writeout != 1)
  {
    perror("Failed to write register address");
    std::cout << gs_fd << " " << writeout << std::endl;
    return 1;
  }

  if (read(gs_fd, buf, len) != len)
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
uint8_t tcs34725_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
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

  if (write(gs_fd, data, len + 1) != (len + 1))
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
void tcs34725_interface_delay_ms(uint32_t ms)
{
  usleep(1000 * ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void tcs34725_interface_debug_print(const char *const fmt, ...)
{

  char str[256];
  uint16_t len;
  va_list args;

  memset((char *)str, 0, sizeof(char) * 256);
  va_start(args, fmt);
  vsnprintf((char *)str, 255, (char const *)fmt, args);
  va_end(args);

  len = strlen((char *)str);
  (void)printf(str, len);
}

static tcs34725_handle_t gs_handle; /**< tcs34725 handle */
namespace ghost_sensing
{
  TCSColorSensorNode::TCSColorSensorNode()
      : rclcpp::Node("tcs_color_sensor_node")
  {
    printf("YO\n");

    uint8_t res;

    /* link interface function */
    DRIVER_TCS34725_LINK_INIT(&gs_handle, tcs34725_handle_t);
    DRIVER_TCS34725_LINK_IIC_INIT(&gs_handle, tcs34725_interface_iic_init);
    DRIVER_TCS34725_LINK_IIC_DEINIT(&gs_handle, tcs34725_interface_iic_deinit);
    DRIVER_TCS34725_LINK_IIC_READ(&gs_handle, tcs34725_interface_iic_read);
    DRIVER_TCS34725_LINK_IIC_WRITE(&gs_handle, tcs34725_interface_iic_write);
    DRIVER_TCS34725_LINK_DELAY_MS(&gs_handle, tcs34725_interface_delay_ms);
    DRIVER_TCS34725_LINK_DEBUG_PRINT(&gs_handle, tcs34725_interface_debug_print);

    /* tcs34725 init */
    res = tcs34725_init(&gs_handle);
    if (res != 0)
    {
      tcs34725_interface_debug_print("tcs34725: init failed.\n");
    }
     /* set power on */
  
    /* set rgbc interrupt */
    res = tcs34725_set_rgbc_interrupt(&gs_handle, TCS34725_BASIC_DEFAULT_RGBC_INTERRUPT);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set rgbc interrupt failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set wait */
    res = tcs34725_set_wait(&gs_handle, TCS34725_BASIC_DEFAULT_WAIT);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set wait failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set rgbc */
    res = tcs34725_set_rgbc(&gs_handle, TCS34725_BASIC_DEFAULT_RGBC);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set rgbc failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set rgbc integration time */
    res = tcs34725_set_rgbc_integration_time(&gs_handle, TCS34725_BASIC_DEFAULT_INTEGRATION_TIME);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set rgbc integration time failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set wait time */
    res = tcs34725_set_wait_time(&gs_handle, TCS34725_BASIC_DEFAULT_WAIT_TIME);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set wait time failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set rgbc clear low interrupt threshold */
    res = tcs34725_set_rgbc_clear_low_interrupt_threshold(&gs_handle, TCS34725_BASIC_DEFAULT_CLEAR_LOW_INTERRUPT_THRESHOLD);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set rgbc clear low interrupt threshold failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set rgbc clear high interrupt threshold */
    res = tcs34725_set_rgbc_clear_high_interrupt_threshold(&gs_handle, TCS34725_BASIC_DEFAULT_CLEAR_HIGH_INTERRUPT_THRESHOLD);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set rgbc clear high interrupt threshold failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set gain */
    res = tcs34725_set_gain(&gs_handle, TCS34725_BASIC_DEFAULT_GAIN);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set gain failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set interrupt mode */
    res = tcs34725_set_interrupt_mode(&gs_handle, TCS34725_BASIC_DEFAULT_INTERRUPT_MODE);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set interrupt mode failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }
    
    /* set power on */
    res = tcs34725_set_power_on(&gs_handle, TCS34725_BASIC_DEFAULT_POWER_ON);
    if (res != 0)
    {
        tcs34725_interface_debug_print("tcs34725: set power on failed.\n");
        (void)tcs34725_deinit(&gs_handle);
        
    }  
    printf("INIT FINISHED\n");



for (int i = 0;i <6 * 1;i++) {
  uint16_t red = 0,  green=0,  blue=0,  clear=0;
    int rgbc = tcs34725_read_rgbc(&gs_handle, &red, &green,& blue,& clear) ;
      printf("==%d r: 0x%x g: 0x%x b: 0x%x c: 0x%x\n", rgbc , red , green , blue , clear );
    usleep(1000 * 500);
}
  
  
  
  }
  void TCSColorSensorNode::start()
  {
  }

}
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
std::make_shared<ghost_sensing::TCSColorSensorNode>();
  //rclcpp::spin(std::make_shared<ghost_sensing::TCSColorSensorNode>());
  //rclcpp::shutdown();
  return 0;
}
