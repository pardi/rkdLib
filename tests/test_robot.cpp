#include <gtest/gtest.h>
#include <RKD/Robot.h>

// Demonstrate some basic assertions.
TEST(RobotTest, BasicAssertions) {
  
  RKD::Robot myRobot;

  EXPECT_DEATH(myRobot.getNrOfJoints(), "mY ERROR");

}
