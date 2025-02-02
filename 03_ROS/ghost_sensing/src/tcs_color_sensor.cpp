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
#include <rclcpp/rclcpp.hpp>

void rgbc2hsv(
  float r, float g, float b, float clear, float & h, float & s,
  float & v)
{
  // ignoring clear value rn


  if (r < 0 || r > 1 || g < 0 || g > 1 || b < 0 || b > 1) {
    throw std::invalid_argument("RGB values must be in range [0,1]");
  }

  float vmax = std::max({r, g, b});
  float vmin = std::min({r, g, b});
  float c = vmax - vmin;
  float l = (vmax + vmin) / 2;

  h = 0;
  if (c != 0) {
    if (vmax == r) {
      h = 60 * (fmod((g - b) / c, 6));
    } else if (vmax == g) {
      h = 60 * ((b - r) / c + 2);
    } else {     // vmax == b
      h = 60 * ((r - g) / c + 4);
    }
  }
  if (h < 0) {h += 360;}

  float sv = (vmax == 0) ? 0 : (c / vmax);
  float sl = (l == 0 || l == 1) ? 0 : (c / (1 - std::abs(2 * l - 1)));

  //ColorHSV hsv = {h, sv, vmax};
  s = sv;
  v = vmax;
  //ColorHSL hsl = {h, sl, l};


  //return {hsv, hsl};
}


using namespace std::chrono_literals;
namespace ghost_sensing
{
TCSColorSensorNode::TCSColorSensorNode()
: rclcpp::Node("tcs_color_sensor_node")
{
  std::string filename = "/dev/i2c-7";
  uint8_t res;


  auto iface = std::make_shared<tcs_i2c_interface>(filename);
  res = iface->init();
  if (res == 1) {
    RCLCPP_FATAL(
      this->get_logger(), "tcs34725 color sensor: %s bus does not exist\n", filename.c_str());
    rclcpp::shutdown();
  }

  m_sensor = std::make_shared<color_sensor_tcs34725>(iface);

  m_publish_timer =
    this->create_wall_timer(20ms, std::bind(&TCSColorSensorNode::timer_poll_color_sensor, this));
  m_color_pub = this->create_publisher<ghost_msgs::msg::ColorSensor>("/sensors/color_sensor", 10);// whats 10


  printf("INIT FINISHED\n");


}

void TCSColorSensorNode::timer_poll_color_sensor()
{
  if (m_delay_loops-- > 0) {return;}
  auto msg = ghost_msgs::msg::ColorSensor();
  int rgbc = m_sensor->read_rgbc(&msg.raw_r, &msg.raw_g, &msg.raw_b, &msg.raw_c);
  rgbc = 0, msg.raw_r = 0, msg.raw_g = 1<<16 - 1, msg.raw_b = 0;
  if (rgbc == 1) {
    int res = m_sensor->init();
    if (res != 0) {
      RCLCPP_WARN(this->get_logger(), "tcs34725 color sensor: init failed.\n");
    }
    m_delay_loops = 100;
    return;
  } else if (rgbc == 4) {
    // data not ready yet
    return;
  }


  float max_sensor_val = (1 << 16) - 1;
  rgbc2hsv(
    msg.raw_r / max_sensor_val, msg.raw_g / max_sensor_val, msg.raw_b / max_sensor_val,
    msg.raw_c / max_sensor_val, msg.h, msg.s, msg.v);

  m_color_pub->publish(msg);

  printf(
    "r: %d g: %d b: %d c: %d h: %f s: %f v: %f\n", msg.raw_r, msg.raw_g, msg.raw_b, msg.raw_c,
    msg.h, msg.s,
    msg.v);
}

void TCSColorSensorNode::start()
{
}

}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ghost_sensing::TCSColorSensorNode>());
  rclcpp::shutdown();
  return 0;
}
