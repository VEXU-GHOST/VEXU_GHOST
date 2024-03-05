#include "annie-onboarding/subscriber_member_function.hpp"


namespace annie_onboarding{

    MinimalSubscriber::MinimalSubscriber(): Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }


    void MinimalSubscriber::topic_callback(std_msgs::msg::String::SharedPtr msg){

      

      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

        int MinimalSubscriber::add_ints(int a, int b){
        return a + b; 
        }

        float MinimalSubscriber::add_floats(float a, float b){
        float sum = a + b;
        return sum;
        }

        // float MinimalSubscriber::function_that_throws_error(){
        // return 1/0;
        // }

//         bool MinimalSubscriber::comparing_strings(std::string a, std::string b){
//         bool verify = false;
//         if(a == b){
//         verify = true;
//         }
//           return verify;
// }

  // private:
  //   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

}