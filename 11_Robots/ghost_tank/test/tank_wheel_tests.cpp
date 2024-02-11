#include "ghost_tank/tank_wheel_odom.hpp"
#include "gtest/gtest.h"

class TankTest : public ::testing::Test {
protected:
std::shared_ptr<ghost_tank::TankOdometry> tank;


	void SetUp() override {
		tank = std::make_shared<ghost_tank::TankOdometry>();
	}

	void TearDown() override {
	}
	
};

// TEST_F(TankTest, InitialPositionTest) {
//     std::vector<double> initialPosition = tank->getRobotPositionWorld();
//     ASSERT_EQ(initialPosition[0], 0.0);  
//     ASSERT_EQ(initialPosition[1], 0.0);  
//     ASSERT_EQ(initialPosition[2], 0.0);  
// }

TEST_F(TankTest, StraightLineTest){
    tank->updateEncoders(2.0, 2.0);
    tank->setAngle(0.349066);
    tank->setXPosition(5.0);
    tank->setYPosition(2.0);
   // m_Angle = 0.349066;
   // m_Xposition = 5.0;
   // m_Yposition = 2.0;
	
std::vector<double> StraightLinePosition = tank->getRobotPositionWorld();
    ASSERT_NEAR(StraightLinePosition[0], 5.063962844, 0.1);  
    ASSERT_NEAR(StraightLinePosition[1], 2.023280582, 0.1);  
    ASSERT_NEAR(StraightLinePosition[2], 0.349066, 0.1);   
}

TEST_F(TankTest, HalfCircleTest){
tank->updateEncoders(-507.6923076, 507.6923076);  
   //tank->setAngle(0.0);  
    tank->setXPosition(5.0);
    tank->setYPosition(2.0);

    std::vector<double> HalfCirclePosition = tank->getRobotPositionWorld();
    EXPECT_NEAR(HalfCirclePosition[0], 5.0, 0.1);  
    EXPECT_NEAR(HalfCirclePosition[1], 2.0, 0.1);  
    EXPECT_NEAR(HalfCirclePosition[2], 3.1415926535, 0.1); 
}

/*TEST_F(TankTest, TurnandGoStraightTest){
    tank -> updateEncoders(126.923, -126.923);
   // tank -> setAngle(0.0);
    tank -> setXPosition(0.0);
    tank -> setYPosition(0.0);

    tank->updateEncoders(2.0, 2.0);
std::vector<double> TurnandStraightLinePosition = tank->getRobotPositionWorld();
    ASSERT_NEAR(TurnandStraightLinePosition[0], 0.0481312318, 0.1);  
    ASSERT_NEAR(TurnandStraightLinePosition[1], 0.0481312318, 0.1);  
    ASSERT_NEAR(TurnandStraightLinePosition[2], 0.7853981633, 0.1);   

}*/

int main(int argc, char **argv) {
	//rclcpp::init(argc, argv);


	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}