#pragma once
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
 * @file      driver_tcs34725.h
 * @brief     driver tcs34725 header file
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

#include <driver_tcs34725_interface.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <memory>

namespace ghost_sensing
{
class color_sensor_tcs34725
{
public:
/**
 * @brief chip register definition
 */
  class REG
  {
public:
    static const uint8_t ENABLE = 0x80,          /**< enable register */
      ATIME = 0x81,          /**< atime register */
      WTIME = 0x83,          /**< wtime register */
      AILTL = 0xA4,          /**< ailtl register */
      AILTH = 0xA5,          /**< ailth register */
      AIHTL = 0xA6,          /**< aihtl register */
      AIHTH = 0xA7,          /**< aihtl register */
      PERS = 0x8C,           /**< pers register */
      CONFIG = 0x8D,         /**< config register */
      CONTROL = 0x8F,        /**< control register */
      ID = 0x92,             /**< id register */
      STATUS = 0x93,         /**< status register */
      CDATAL = 0xB4,         /**< cdatal register */
      CDATAH = 0xB5,         /**< cdatah register */
      RDATAL = 0xB6,         /**< rdatal register */
      RDATAH = 0xB7,         /**< rdatah register */
      GDATAL = 0xB8,         /**< gdatal register */
      GDATAH = 0xB9,         /**< gdatah register */
      BDATAL = 0xBA,         /**< bdatal register */
      BDATAH = 0xBB,         /**< bdatah register */
      CLEAR = 0xE6;          /**< clear register */
  };

  const uint8_t ADDRESS = 0x29;

  color_sensor_tcs34725(std::shared_ptr<tcs_i2c_interface> iface)
  : m_i2c_communication(iface) {}
/**
 * @brief chip information definition
 */
  static constexpr const char * CHIP_NAME = "AMS TCS34725";
  static constexpr const char * MANUFACTURER_NAME = "AMS";                      /**< manufacturer name */
  static constexpr float SUPPLY_VOLTAGE_MIN = 2.7f;                      /**< chip min supply voltage */
  static constexpr float SUPPLY_VOLTAGE_MAX = 3.6f;                      /**< chip max supply voltage */
  static constexpr float MAX_CURRENT = 20.0f;                            /**< chip max current */
  static constexpr float TEMPERATURE_MIN = -40.0f;                       /**< chip min operating temperature */
  static constexpr float TEMPERATURE_MAX = 85.0f;                        /**< chip max operating temperature */
  static constexpr int DRIVER_VERSION = 2000; /**< driver version */                          // ????????????

/**
 * @brief tcs34725 integration time enumeration definition
 */
  typedef enum
  {
    INTEGRATION_TIME_2P4MS = 0xFF,        /**< integration time 2.4 ms */
    INTEGRATION_TIME_24MS  = 0xF6,        /**< integration time 24 ms */
    INTEGRATION_TIME_50MS  = 0xEB,        /**< integration time 50 ms */
    INTEGRATION_TIME_101MS = 0xD5,        /**< integration time 101 ms */
    INTEGRATION_TIME_154MS = 0xC0,        /**< integration time 154 ms */
    INTEGRATION_TIME_700MS = 0x00,        /**< integration time 700 ms */
  } integration_time_t;

/**
 * @brief tcs34725 gain enumeration definition
 */
  typedef enum
  {
    GAIN_1X  = 0x00,        /**< 1x gain */
    GAIN_4X  = 0x01,        /**< 4x gain */
    GAIN_16X = 0x02,        /**< 16x gain */
    GAIN_60X = 0x03,        /**< 60x gain */
  } gain_t;

/**
 * @brief tcs34725 wait time enumeration definition
 */
  typedef enum
  {
    WAIT_TIME_2P4MS  = 0x0FF,        /**< 2.4 ms wait time */
    WAIT_TIME_204MS  = 0x0AB,        /**< 204 ms wait time */
    WAIT_TIME_614MS  = 0x000,        /**< 614 ms wait time */
    WAIT_TIME_29MS   = 0x1FF,        /**< 29 ms wait time */
    WAIT_TIME_2450MS = 0x1AB,        /**< 2450 ms wait time */
    WAIT_TIME_7400MS = 0x100,        /**< 7400 ms wait time */
  } wait_time_t;

/**
 * @}
 */

/**
 * @addtogroup tcs34725_interrupt_driver
 * @{
 */

/**
 * @brief tcs34725 interrupt mode enumeration definition
 */
  typedef enum
  {
    INTERRUPT_MODE_EVERY_RGBC_CYCLE                  = 0x00,        /**< every rgbc cycle interrupt */
    INTERRUPT_MODE_1_CLEAR_CHANNEL_OUT_OF_THRESHOLD  = 0x01,        /**< 1 cycle out of threshold interrupt */
    INTERRUPT_MODE_2_CLEAR_CHANNEL_OUT_OF_THRESHOLD  = 0x02,        /**< 2 cycle out of threshold interrupt */
    INTERRUPT_MODE_3_CLEAR_CHANNEL_OUT_OF_THRESHOLD  = 0x03,        /**< 3 cycle out of threshold interrupt */
    INTERRUPT_MODE_5_CLEAR_CHANNEL_OUT_OF_THRESHOLD  = 0x04,        /**< 5 cycle out of threshold interrupt */
    INTERRUPT_MODE_10_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x05,        /**< 10 cycle out of threshold interrupt */
    INTERRUPT_MODE_15_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x06,        /**< 15 cycle out of threshold interrupt */
    INTERRUPT_MODE_20_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x07,        /**< 20 cycle out of threshold interrupt */
    INTERRUPT_MODE_25_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x08,        /**< 25 cycle out of threshold interrupt */
    INTERRUPT_MODE_30_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x09,        /**< 30 cycle out of threshold interrupt */
    INTERRUPT_MODE_35_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x0A,        /**< 35 cycle out of threshold interrupt */
    INTERRUPT_MODE_40_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x0B,        /**< 40 cycle out of threshold interrupt */
    INTERRUPT_MODE_45_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x0C,        /**< 45 cycle out of threshold interrupt */
    INTERRUPT_MODE_50_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x0D,        /**< 50 cycle out of threshold interrupt */
    INTERRUPT_MODE_55_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x0E,        /**< 55 cycle out of threshold interrupt */
    INTERRUPT_MODE_60_CLEAR_CHANNEL_OUT_OF_THRESHOLD = 0x0F,        /**< 60 cycle out of threshold interrupt */
  } interrupt_mode_t;


/**
 * @brief     initialize the chip
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 * @note      none
 */
  uint8_t init(
    bool enable_interrupt = false, //
    bool enable_wait = false,  //
    bool power_on = true,  //
    bool enable_rgbc_measurement = true,   //
    integration_time_t integration_time = INTEGRATION_TIME_50MS,//
    wait_time_t wait_time = WAIT_TIME_2P4MS, //

    uint16_t low_interrupt_threshold = 0x0000U,//
    uint16_t high_interrupt_threshold = 0xFFFFU,//
    gain_t gain = GAIN_16X,                       //
    interrupt_mode_t interrupt_mode = INTERRUPT_MODE_1_CLEAR_CHANNEL_OUT_OF_THRESHOLD//
  );

/**
 * @brief     close the chip
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t deinit();

/**
 * @brief      read the rgbc data
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *red pointer to a red color buffer
 * @param[out] *green pointer to a green color buffer
 * @param[out] *blue pointer to a blue color buffer
 * @param[out] *clear pointer to a clear color buffer
 * @return     status code
 *             - 0 success
 *             - 1 read rgbc failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t read_rgbc(
    uint16_t * red, uint16_t * green,
    uint16_t * blue, uint16_t * clear);

/**
 * @brief      read the rgb data
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *red pointer to a red color buffer
 * @param[out] *green pointer to a green color buffer
 * @param[out] *blue pointer to a blue color buffer
 * @return     status code
 *             - 0 success
 *             - 1 read rgb failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t read_rgb(
    uint16_t * red, uint16_t * green,
    uint16_t * blue);

/**
 * @brief      read the clear data
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *clear pointer to a clear color buffer
 * @return     status code
 *             - 0 success
 *             - 1 read clear failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t read_c(uint16_t * clear);

private: // maybe remove
/**
 * @brief     enable or disable the wait time
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] enable bool value
 * @return    status code
 *            - 0 success
 *            - 1 set wait failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_wait(bool enable);

/**
 * @brief      get the wait time
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *enable pointer to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get wait failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_wait(bool * enable);

/**
 * @brief     enable or disable the rgbc adc
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] enable bool value
 * @return    status code
 *            - 0 success
 *            - 1 set rgbc failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_rgbc(bool enable);

/**
 * @brief      get the rgbc status
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *enable pointer to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rgbc failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_rgbc(bool * enable);

/**
 * @brief     enable or disable the power
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] enable bool value
 * @return    status code
 *            - 0 success
 *            - 1 set power on failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_power_on(bool enable);

/**
 * @brief      get the power status
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *enable pointer to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get power on failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_power_on(bool * enable);

/**
 * @brief     set the rgbc adc integration time
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] t adc integration time
 * @return    status code
 *            - 0 success
 *            - 1 set rgbc integration time failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_rgbc_integration_time(
    integration_time_t t);

/**
 * @brief      get the rgbc adc integration time
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *t pointer to an integration time buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rgbc integration time failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_rgbc_integration_time(
    integration_time_t * t);

/**
 * @brief     set the wait time
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] t wait time
 * @return    status code
 *            - 0 success
 *            - 1 set wait time failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_wait_time(wait_time_t t);

/**
 * @brief      get the wait time
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *t pointer to a wait time buffer
 * @return     status code
 *             - 0 success
 *             - 1 get wait time failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_wait_time(wait_time_t * t);

/**
 * @brief     set the adc gain
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] gain adc gain
 * @return    status code
 *            - 0 success
 *            - 1 set gain failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_gain(gain_t gain);

/**
 * @brief      get the adc gain
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *gain pointer to an adc gain buffer
 * @return     status code
 *             - 0 success
 *             - 1 get gain failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_gain(gain_t * gain);

/**
 * @}
 */

/**
 * @defgroup interrupt_driver tcs34725 interrupt driver function
 * @brief    tcs34725 interrupt driver modules
 * @ingroup  driver
 * @{
 */

/**
 * @brief     enable or disable the rgbc interrupt
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] enable bool value
 * @return    status code
 *            - 0 success
 *            - 1 set rgbc interrupt failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_rgbc_interrupt(bool enable);

/**
 * @brief      get the rgbc interrupt
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *enable pointer to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rgbc interrupt failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_rgbc_interrupt(bool * enable);

/**
 * @brief     set the interrupt mode
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] mode interrupt mode
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_interrupt_mode(interrupt_mode_t mode);

/**
 * @brief      get the interrupt mode
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *mode pointer to an interrupt mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt mode failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_interrupt_mode(interrupt_mode_t * mode);

/**
 * @brief     set the rgbc clear low interrupt threshold
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] threshold low interrupt threshold
 * @return    status code
 *            - 0 success
 *            - 1 set rgbc clear low interrupt threshold failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_rgbc_clear_low_interrupt_threshold(

    uint16_t threshold);

/**
 * @brief      get the rgbc clear low interrupt threshold
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *threshold pointer to a low interrupt threshold buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rgbc clear low interrupt threshold failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_rgbc_clear_low_interrupt_threshold(

    uint16_t * threshold);

/**
 * @brief     set the rgbc clear high interrupt threshold
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] threshold high interrupt threshold
 * @return    status code
 *            - 0 success
 *            - 1 set rgbc clear high interrupt threshold failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_rgbc_clear_high_interrupt_threshold(

    uint16_t threshold);

/**
 * @brief      get the rgbc clear high interrupt threshold
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[out] *threshold pointer to a high interrupt threshold buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rgbc clear high interrupt threshold failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_rgbc_clear_high_interrupt_threshold(

    uint16_t * threshold);

/**
 * @}
 */

/**
 * @defgroup extend_driver tcs34725 extend driver function
 * @brief    tcs34725 extend driver modules
 * @ingroup  driver
 * @{
 */

/**
 * @brief     set the chip register
 * @param[in] *handle pointer to a tcs34725 handle structure
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len data buffer length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
  uint8_t set_reg(uint8_t reg, uint8_t * buf, uint16_t len);

/**
 * @brief      get the chip register
 * @param[in]  *handle pointer to a tcs34725 handle structure
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len data buffer length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
  uint8_t get_reg(uint8_t reg, uint8_t * buf, uint16_t len);

protected:
  std::shared_ptr<ghost_sensing::tcs_i2c_interface> m_i2c_communication;
};
}
