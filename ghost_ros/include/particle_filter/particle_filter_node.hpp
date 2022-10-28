class ParticleFilterNode : public rclcpp::Node {
  public:

  ParticleFilterNode(): Node("particle_filter_node"){
    // Subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "latency_test_topic",
      10,
      std::bind(&ParticleFilterNode::LaserCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "latency_test_topic",
      10,
      std::bind(&ParticleFilterNode::OdometryCallback, this, _1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "latency_test_topic",
      10,
      std::bind(&ParticleFilterNode::InitialPoseCallback, this, _1));

    // Publishers
    // publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("latency_test_topic", 10);

  }

  void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

  }

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

  }

  void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

  }
  
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

};