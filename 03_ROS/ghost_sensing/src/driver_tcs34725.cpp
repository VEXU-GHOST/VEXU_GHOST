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
 * @file      driver_tcs34725.c
 * @brief     driver tcs34725 source file
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

#include "driver_tcs34725.h"


namespace ghost_sensing
{

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
uint8_t color_sensor_tcs34725::set_rgbc_interrupt(bool enable)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);            /* read enable config */
  if (res != 0) {                                                                                /* check the result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  prev &= ~(1 << 4);                                                                             /* clear interrupt */
  prev |= enable << 4;                                                                           /* set enable */
  res = m_i2c_communication->write(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                                /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                   /* write register failed */

    return 1;                                                                                    /* return error */
  }

  return 0;                                                                                      /* success return 0 */
}


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
uint8_t color_sensor_tcs34725::get_rgbc_interrupt(bool * enable)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);          /* read enable config */
  if (res != 0) {                                                                              /* check the result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read register failed */

    return 1;                                                                                  /* return error */
  }
  prev &= 1 << 4;                                                                              /* get interrupt */
  *enable = (bool)((prev >> 4) & 0x01);                                             /* set interrupt */

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_wait(bool enable)
{
  uint8_t res, prev;

  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);            /* read enable config */
  if (res != 0) {                                                                                /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  prev &= ~(1 << 3);                                                                             /* clear enable bit */
  prev |= enable << 3;                                                                           /* set enable */
  res = m_i2c_communication->write(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                                /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                   /* write register failed */

    return 1;                                                                                    /* return error */
  }

  return 0;                                                                                      /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_wait(bool * enable)
{
  uint8_t res, prev;

  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);          /* read config */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read register failed */

    return 1;                                                                                  /* return error */
  }
  prev &= 1 << 3;                                                                              /* get wait bit */
  *enable = (bool)((prev >> 3) & 0x01);                                             /* get wait */

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_rgbc(bool enable)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);            /* read config */
  if (res != 0) {                                                                                /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  prev &= ~(1 << 1);                                                                             /* clear enable bit */
  prev |= enable << 1;                                                                           /* set enable */
  res = m_i2c_communication->write(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                                /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                   /* write register failed */

    return 1;                                                                                    /* return error */
  }

  return 0;                                                                                      /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_rgbc(bool * enable)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);          /* read enable config */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read register failed */

    return 1;                                                                                  /* return error */
  }
  prev &= 1 << 1;                                                                              /* get rgbc bit */
  *enable = (bool)((prev >> 1) & 0x01);                                             /* get enable */

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_power_on(bool enable)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);            /* read config */
  if (res != 0) {                                                                                /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  prev &= ~(1 << 0);                                                                             /* clear enable bit */
  prev |= enable << 0;                                                                           /* set enable */
  res = m_i2c_communication->write(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                                /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                   /* write register failed */

    return 1;                                                                                    /* return error */
  }

  return 0;                                                                                      /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_power_on(bool * enable)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);          /* read enable config */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read register failed */

    return 1;                                                                                  /* return error */
  }
  prev &= 1 << 0;                                                                              /* get enable bit */
  *enable = (bool)((prev >> 0) & 0x01);                                             /* get enable */

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_rgbc_integration_time(

  integration_time_t t)
{
  uint8_t res;


  res = m_i2c_communication->write(ADDRESS, REG::ATIME, (uint8_t *)&t, 1);             /* write config */
  if (res != 0) {                                                                              /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                 /* write register failed */

    return 1;                                                                                  /* return error */
  }

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_rgbc_integration_time(

  integration_time_t * t)
{
  uint8_t res;


  res = m_i2c_communication->read(ADDRESS, REG::ATIME, (uint8_t *)t, 1);             /* read config */
  if (res != 0) {                                                                            /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                               /* write register failed */

    return 1;                                                                                /* return error */
  }

  return 0;                                                                                  /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_wait_time(wait_time_t t)
{
  uint8_t res, prev, bit;


  res = m_i2c_communication->read(ADDRESS, REG::CONFIG, (uint8_t *)&prev, 1);           /* read config */
  if (res != 0) {                                                                               /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                   /* read register failed */

    return 1;                                                                                   /* return error */
  }
  bit = (t & 0x100) >> 8;                                                                       /* get bit */
  prev &= ~(1 << 1);                                                                            /* clear wait time bit */
  prev |= bit << 1;                                                                             /* set bit */
  res = m_i2c_communication->write(ADDRESS, REG::CONFIG, (uint8_t *)&prev, 1);          /* write config */
  if (res != 0) {                                                                               /* check result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                  /* write register failed */

    return 1;                                                                                   /* return error */
  }
  prev = t & 0xFF;                                                                              /* get time */
  res = m_i2c_communication->write(ADDRESS, REG::WTIME, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                               /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                  /* write register failed */

    return 1;                                                                                   /* return error */
  }

  return 0;                                                                                     /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_wait_time(wait_time_t * t)
{
  uint8_t res, prev, bit;


  res = m_i2c_communication->read(ADDRESS, REG::CONFIG, (uint8_t *)&prev, 1);          /* read config */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read failed */

    return 1;                                                                                  /* return error */
  }
  prev &= 1 << 1;                                                                              /* get wait time bit */
  bit = (prev >> 1) & 0x01;                                                                    /* get wait time */
  res = m_i2c_communication->read(ADDRESS, REG::WTIME, (uint8_t *)&prev, 1);           /* read config */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read register failed */

    return 1;                                                                                  /* return error */
  }
  *t = (wait_time_t)((bit << 8) | prev);                                              /* get time */

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_rgbc_clear_low_interrupt_threshold(

  uint16_t threshold)
{
  uint8_t res;
  uint8_t buf[2];


  buf[0] = threshold & 0xFF;                                                                 /* get threshold LSB */
  buf[1] = (threshold >> 8) & 0xFF;                                                          /* get threshold MSB */
  res = m_i2c_communication->write(ADDRESS, REG::AILTL, (uint8_t *)buf, 2);          /* write config */
  if (res != 0) {                                                                            /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                               /* write register failed */

    return 1;                                                                                /* return error */
  }

  return 0;                                                                                  /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_rgbc_clear_low_interrupt_threshold(

  uint16_t * threshold)
{
  uint8_t res, buf[2];


  memset(buf, 0, sizeof(uint8_t) * 2);                                                      /* clear the buffer */
  res = m_i2c_communication->read(ADDRESS, REG::AILTL, (uint8_t *)buf, 2);          /* read ailtl */
  if (res != 0) {                                                                           /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                               /* read register failed */

    return 1;                                                                               /* return error */
  }
  *threshold = ((uint16_t)buf[1] << 8) | buf[0];                                            /* get threshold */

  return 0;                                                                                 /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_rgbc_clear_high_interrupt_threshold(

  uint16_t threshold)
{
  uint8_t res;
  uint8_t buf[2];


  buf[0] = threshold & 0xFF;                                                                 /* get threshold LSB */
  buf[1] = (threshold >> 8) & 0xFF;                                                          /* get threshold MSB */
  res = m_i2c_communication->write(ADDRESS, REG::AIHTL, (uint8_t *)buf, 2);          /* write config */
  if (res != 0) {                                                                            /* check the result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                               /* write register failed */

    return 1;                                                                                /* return error */
  }

  return 0;                                                                                  /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_rgbc_clear_high_interrupt_threshold(

  uint16_t * threshold)
{
  uint8_t res, buf[2];


  memset(buf, 0, sizeof(uint8_t) * 2);                                                      /* clear the buffer */
  res = m_i2c_communication->read(ADDRESS, REG::AIHTL, (uint8_t *)buf, 2);          /* read aihtl */
  if (res != 0) {                                                                           /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                               /* read register failed */

    return 1;                                                                               /* return error */
  }
  *threshold = ((uint16_t)buf[1] << 8) | buf[0];                                            /* get threshold */

  return 0;                                                                                 /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_interrupt_mode(interrupt_mode_t mode)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::PERS, (uint8_t *)&prev, 1);            /* read pers */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                  /* read register failed */

    return 1;                                                                                  /* return error */
  }
  prev &= ~0x0F;                                                                               /* clear mode bit */
  prev |= mode;                                                                                /* set mode */
  res = m_i2c_communication->write(ADDRESS, REG::PERS, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                              /* check result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                 /* write register failed */

    return 1;                                                                                  /* return error */
  }

  return 0;                                                                                    /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_interrupt_mode(interrupt_mode_t * mode)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::PERS, (uint8_t *)&prev, 1);          /* read pers */
  if (res != 0) {                                                                            /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                /* read register failed */

    return 1;                                                                                /* return error */
  }
  prev &= 0x0F;                                                                              /* get interrupt mode bits */
  *mode = (interrupt_mode_t)(prev & 0x0F);                                          /* get interrupt mode */

  return 0;                                                                                  /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_gain(gain_t gain)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::CONTROL, (uint8_t *)&prev, 1);            /* read control */
  if (res != 0) {                                                                                 /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                     /* read register failed */

    return 1;                                                                                     /* return error */
  }
  prev &= ~0x03;                                                                                  /* get gain bits */
  prev |= gain;                                                                                   /* set gain */
  res = m_i2c_communication->write(ADDRESS, REG::CONTROL, (uint8_t *)&prev, 1);           /* write config */
  if (res != 0) {                                                                                 /* check result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                    /* write register failed */

    return 1;                                                                                     /* return error */
  }

  return 0;                                                                                       /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_gain(gain_t * gain)
{
  uint8_t res, prev;


  res = m_i2c_communication->read(ADDRESS, REG::CONTROL, (uint8_t *)&prev, 1);          /* read config */
  if (res != 0) {                                                                               /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                   /* read register failed */

    return 1;                                                                                   /* return error */
  }
  prev &= 0x03;                                                                                 /* get gain bits */
  *gain = (gain_t)(prev & 0x03);                                                       /* get gain */

  return 0;                                                                                     /* success return 0 */
}

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
 *             - 4 data not ready
 * @note       none
 */
uint8_t color_sensor_tcs34725::read_rgbc(
  uint16_t * red, uint16_t * green,
  uint16_t * blue, uint16_t * clear)
{
  uint8_t res, prev;
  uint8_t buf[8];


  res = m_i2c_communication->read(ADDRESS, REG::STATUS, (uint8_t *)&prev, 1);            /* read status */
  if (res != 0) {                                                                                /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  if ((prev & (1 << 4)) != 0) {                                                                  /* find interrupt */
    res = m_i2c_communication->write(ADDRESS, REG::CLEAR, NULL, 0);                      /* clear interrupt */
    if (res != 0) {                                                                              /* check result */
      m_i2c_communication->debug_print("tcs34725: clear interrupt failed.\n");                                /* clear interrupt failed */

      return 1;                                                                                  /* return error */
    }
  }
  if ((prev & 0x01) != 0) {                                                                      /* if data ready */
    res = m_i2c_communication->read(ADDRESS, REG::CDATAL, (uint8_t *)buf, 8);            /* read data */
    if (res != 0) {                                                                              /* check result */
      m_i2c_communication->debug_print("tcs34725: read failed.\n");                                           /* read failed */

      return 1;                                                                                  /* return error */
    }
    *clear = ((uint16_t)buf[1] << 8) | buf[0];                                                   /* get clear */
    *red = ((uint16_t)buf[3] << 8) | buf[2];                                                     /* get red */
    *green = ((uint16_t)buf[5] << 8) | buf[4];                                                   /* get green */
    *blue = ((uint16_t)buf[7] << 8) | buf[6];                                                    /* get blue */

    return 0;                                                                                    /* success return 0 */
  } else {
    m_i2c_communication->debug_print("tcs34725: data not ready.\n");                                          /* data not ready */

    return 4;                                                                                    /* return error */
  }
}

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
uint8_t color_sensor_tcs34725::read_rgb(
  uint16_t * red, uint16_t * green,
  uint16_t * blue)
{
  uint8_t res, prev;
  uint8_t buf[8];


  res = m_i2c_communication->read(ADDRESS, REG::STATUS, (uint8_t *)&prev, 1);            /* read config */
  if (res != 0) {                                                                                /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  if ((prev & (1 << 4)) != 0) {                                                                  /* find interrupt */
    res = m_i2c_communication->write(ADDRESS, REG::CLEAR, NULL, 0);                      /* clear interrupt */
    if (res != 0) {                                                                              /* check result */
      m_i2c_communication->debug_print("tcs34725: clear interrupt failed.\n");                                /* clear interrupt failed */

      return 1;                                                                                  /* return error */
    }
  }
  if ((prev & 0x01) != 0) {                                                                      /* if data ready */
    res = m_i2c_communication->read(ADDRESS, REG::CDATAL, (uint8_t *)buf, 8);            /* read data */
    if (res != 0) {                                                                              /* check result */
      m_i2c_communication->debug_print("tcs34725: read failed.\n");                                           /* read data failed */

      return 1;                                                                                  /* return error */
    }
    *red = ((uint16_t)buf[3] << 8) | buf[2];                                                     /* get red */
    *green = ((uint16_t)buf[5] << 8) | buf[4];                                                   /* get green */
    *blue = ((uint16_t)buf[7] << 8) | buf[6];                                                    /* get blue */

    return 0;                                                                                    /* success return 0 */
  } else {
    m_i2c_communication->debug_print("tcs34725: data not ready.\n");                                          /* data not ready */

    return 1;                                                                                    /* return error */
  }
}

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
uint8_t color_sensor_tcs34725::read_c(uint16_t * clear)
{
  uint8_t res, prev;
  uint8_t buf[8];


  res = m_i2c_communication->read(ADDRESS, REG::STATUS, (uint8_t *)&prev, 1);            /* read status */
  if (res != 0) {                                                                                /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                    /* read register failed */

    return 1;                                                                                    /* return error */
  }
  if ((prev & (1 << 4)) != 0) {                                                                  /* find interrupt */
    res = m_i2c_communication->write(ADDRESS, REG::CLEAR, NULL, 0);                      /* clear interrupt */
    if (res != 0) {                                                                              /* check result */
      m_i2c_communication->debug_print("tcs34725: clear interrupt failed.\n");                                /* clear interrupt failed */

      return 1;                                                                                  /* return error */
    }
  }
  if ((prev & 0x01) != 0) {                                                                      /* if data ready */
    res = m_i2c_communication->read(ADDRESS, REG::CDATAL, (uint8_t *)buf, 8);            /* read data */
    if (res != 0) {                                                                              /* check result */
      m_i2c_communication->debug_print("tcs34725: read failed.\n");                                           /* read failed */

      return 1;                                                                                  /* return error */
    }
    *clear = ((uint16_t)buf[1] << 8) | buf[0];                                                   /* get clear */

    return 0;                                                                                    /* success return 0 */
  } else {
    m_i2c_communication->debug_print("tcs34725: data not ready.\n");                                          /* data not ready */

    return 1;                                                                                    /* return error */
  }
}

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
uint8_t color_sensor_tcs34725::init(
  bool enable_interrupt,
  bool enable_wait,
  bool power_on,
  bool enable_rgbc_measurement,
  integration_time_t integration_time,
  wait_time_t wait_time,
  uint16_t low_interrupt_threshold,
  uint16_t high_interrupt_threshold,
  gain_t gain,
  interrupt_mode_t interrupt_mode
)
{
  uint8_t res, id;

  res = m_i2c_communication->read(ADDRESS, REG::ID, (uint8_t *)&id, 1);          /* read id */
  if (res != 0) {                                                                        /* check result */
    m_i2c_communication->debug_print("tcs34725: read id failed.\n");                                  /* read id failed */
    //deinit();

    return 1;                                                                            /* return error */
  }
  m_i2c_communication->debug_print("tcs34725: found id is 0x%x\n", id);
  if ((id != 0x44) && (id != 0x4D)) {                                                    /* check id */
    m_i2c_communication->debug_print("tcs34725: id is error.\n");                                     /* id is error */
    deinit();

    return 1;                                                                            /* return error */
  }

  if (set_rgbc_interrupt(enable_interrupt) != 0) {
    m_i2c_communication->debug_print("set rgbc interrupt failed");
    deinit();
    return 1;
  }

  if (set_wait(enable_wait) != 0) {
    m_i2c_communication->debug_print("set wait failed");
    deinit();
    return 1;
  }

  if (set_rgbc(enable_rgbc_measurement) != 0) {
    m_i2c_communication->debug_print("set rgbc failed");
    deinit();
    return 1;
  }


  if (set_rgbc_integration_time(integration_time) != 0) {
    m_i2c_communication->debug_print("set rgcb integration time failed");
    deinit();
    return 1;
  }

  if (set_wait_time(wait_time) != 0) {
    m_i2c_communication->debug_print("set wait time failed");
    deinit();
    return 1;
  }
  if (set_rgbc_clear_low_interrupt_threshold(low_interrupt_threshold) != 0) {
    m_i2c_communication->debug_print("set rgbc clear low interrupt threshold failed");
    deinit();
    return 1;
  }

  if (set_rgbc_clear_high_interrupt_threshold(high_interrupt_threshold) != 0) {
    m_i2c_communication->debug_print("set rgbc clear high interrupt threshold failed");
    deinit();
    return 1;
  }

  if (set_gain(gain) != 0) {
    m_i2c_communication->debug_print("set gain failed");
    deinit();
    return 1;
  }


  if (set_interrupt_mode(interrupt_mode) != 0) {
    m_i2c_communication->debug_print("set interrupt mode failed");
    deinit();
    return 1;
  }
  if (set_power_on(power_on) != 0) {
    m_i2c_communication->debug_print("set power on failed");
    deinit();
    return 1;
  }
  return 0;                                                                              /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::deinit()
{
  uint8_t res, prev;

  res = m_i2c_communication->read(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1);         /* read enable */
  if (res != 0) {                                                                             /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");                                 /* read register failed */
    //m_i2c_communication->deinit();
    return 1;                                                                                 /* return error */
  }
  prev &= ~(1 << 0);                                                                          /* disable */
  if (m_i2c_communication->write(ADDRESS, REG::ENABLE, (uint8_t *)&prev, 1) != 0) {   /* write enable */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");                                /* write register failed */

    return 1;                                                                                 /* return error */
  }
  //if (m_i2c_communication->deinit() != 0) {                                                            /* iic deinit */
  //  m_i2c_communication->debug_print("tcs34725: iic deinit failed.\n");                                    /* iic deinit failed */

  //  return 1;                                                                                 /* return error */
  //}
  return 0;                                                                                   /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::set_reg(uint8_t reg, uint8_t * buf, uint16_t len)
{
  uint8_t res;

  res = m_i2c_communication->write(ADDRESS, reg, buf, len);               /* write data */
  if (res != 0) {                                                         /* check result */
    m_i2c_communication->debug_print("tcs34725: write register failed.\n");            /* write register failed */

    return 1;                                                             /* return error */
  }

  return 0;                                                               /* success return 0 */
}

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
uint8_t color_sensor_tcs34725::get_reg(uint8_t reg, uint8_t * buf, uint16_t len)
{
  uint8_t res;

  res = m_i2c_communication->read(ADDRESS, reg, buf, len);               /* read data */
  if (res != 0) {                                                        /* check result */
    m_i2c_communication->debug_print("tcs34725: read register failed.\n");            /* read register failed */

    return 1;                                                            /* return error */
  }

  return 0;                                                              /* success return 0 */
}

}
