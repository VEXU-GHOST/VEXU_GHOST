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

// https://en.wikipedia.org/wiki/HSL_and_HSV#From_RGB
// https://chatgpt.com/share/679f0740-6d9c-800c-98c3-25346862aadc
std_msgs::msg::ColorRGBA rgbc2hsv(std_msgs::msg::ColorRGBA rgb)
{
  auto hsv = std_msgs::msg::ColorRGBA();

  if (rgb.r < 0 || rgb.r > 1 || rgb.g < 0 || rgb.g > 1 || rgb.b < 0 || rgb.b > 1)
  {
    throw std::invalid_argument("RGB values must be in range [0,1]");
  }

  float vmax = std::max({rgb.r, rgb.g, rgb.b});
  float vmin = std::min({rgb.r, rgb.g, rgb.b});
  float c = vmax - vmin;
  float l = (vmax + vmin) / 2;

  float h = 0;
  if (c != 0)
  {
    if (vmax == rgb.r)
    {
      h = 60 * (fmod((rgb.g - rgb.b) / c, 6));
    }
    else if (vmax == rgb.g)
    {
      h = 60 * ((rgb.b - rgb.r) / c + 2);
    }
    else
    { // vmax == rgb.b
      h = 60 * ((rgb.r - rgb.g) / c + 4);
    }
  }
  if (h < 0)
  {
    h += 360;
  }

  float sv = (vmax == 0) ? 0 : (c / vmax);

  hsv.r = h;
  hsv.g = sv;
  hsv.b = vmax;
  hsv.a = -1; // Preserve alpha value

  // To switch to HSL conversion, replace sv with sl and vmax with l
  // float sl = (l == 0 || l == 1) ? 0 : (c / (1 - std::abs(2 * l - 1)));
  // hsv->g = sl;
  // hsv->b = l;
  return hsv;
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
    if (res == 1)
    {
      RCLCPP_FATAL(
          this->get_logger(), "tcs34725 color sensor: %s bus does not exist\n", filename.c_str());
      rclcpp::shutdown();
    }

    m_sensor = std::make_shared<color_sensor_tcs34725>(iface);

    m_publish_timer =
        this->create_wall_timer(20ms, std::bind(&TCSColorSensorNode::timer_poll_color_sensor, this));
    m_rgb_pub = this->create_publisher<std_msgs::msg::ColorRGBA>("/sensors/color_sensor_0/rgb", 10); // whats 10
    m_hsv_pub = this->create_publisher<std_msgs::msg::ColorRGBA>("/sensors/color_sensor_0/hsv", 10); // whats 10

    printf("INIT FINISHED\n");
  }

  void TCSColorSensorNode::timer_poll_color_sensor()
  {
    const float max_sensor_val = (1 << 16) - 1;
    if (m_delay_loops-- > 0)
    {
      return;
    }
    auto msg_rgb = std_msgs::msg::ColorRGBA();
    uint16_t r = 0, g = 0, b = 0, c = 0;
    int rgbc = m_sensor->read_rgbc(&r, &g, &b, &c);

    rgbc = 0, r = 1 << 12, g =0 , b = 1<<16 - 1; // for testing only
    if (rgbc == 1 || (r == 0 && g == 0 && b == 0 && c == 0))
    {
      // could not communicate or got all zeros which should realistically never happen since we dont clear the registers
      // there might be a better way to check uninitalized sensor, but simple solution rn is that values are all 0 which will never happen unless its perfectly dark which it will never be
      // TODO
      int res = m_sensor->init();
      if (res != 0)
      {
        RCLCPP_WARN(this->get_logger(), "tcs34725 color sensor: init failed.\n");
      }
      m_delay_loops = 100;
      return;
    }
    else if (rgbc == 4)
    {
      // data not ready yet
      return;
    }

    msg_rgb.r = r / max_sensor_val;
    msg_rgb.g = g / max_sensor_val;
    msg_rgb.b = b / max_sensor_val;
    msg_rgb.a = c / max_sensor_val;

    auto msg_hsv = rgbc2hsv(msg_rgb);

    m_rgb_pub->publish(msg_rgb);
    m_hsv_pub->publish(msg_hsv);

    printf(
        "r: %f g: %f b: %f a: %f | h: %f s: %f v: %f a: %f\n",
        msg_rgb.r, msg_rgb.g, msg_rgb.b, msg_rgb.a,
        msg_hsv.r, msg_hsv.g, msg_hsv.b, msg_hsv.a);
  }

}
  int main(int argc, char *argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ghost_sensing::TCSColorSensorNode>());
    rclcpp::shutdown();
    return 0;
  }

/*

ros2 topic pub /visualization_marker visualization_msgs/msg/Marker '{
  header: {frame_id: "map"},
  ns: "test",
  id: 0,
  type: 2,  # Sphere
  action: 0,
  pose: { position: { x: 0.0, y: 0.0, z: 0.0 }},
  scale: { x: 1.0, y: 1.0, z: 1.0 },
  color: { r: 1.0, g: 0.0, b: 0.0, a: 1.0 }  # Red color
}'

*/