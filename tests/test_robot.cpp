#include <gtest/gtest.h>
#include <RKD/Robot.h>

// Demonstrate some basic assertions.
TEST(RobotTest, ClassInitialisation) {
  
  rkd::Robot panda_robot;

  EXPECT_FALSE(panda_robot.good());

}

TEST(RobotTest, ReturnNumberofJoints) {
  
 	rkd::Robot panda_robot("../urdf/panda.urdf", "panda_link0", "panda_hand");

  EXPECT_EQ(panda_robot.getNrOfJoints(), 7);

}


TEST(RobotTest, ReturnFK) {
  
 	rkd::Robot panda_robot("../urdf/panda.urdf", "panda_link0", "panda_hand");

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};
  Eigen::Matrix<double, 6, 1> p_res;
  
  p_res << 0.66419e-310, 4.66419e-310, 1.033, 3.14159, 0, 0;

	Eigen::Map<Eigen::Matrix<double, 6, 1>> p(panda_robot.getFK(q).data());

  const double TOLLERANCE = 1e-5;

  ASSERT_LE((p_res - p).norm(), TOLLERANCE);

}


TEST(RobotTest, ReturnJntPose) {
  
 	rkd::Robot panda_robot("../urdf/panda.urdf", "panda_link0", "panda_hand");

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};
  Eigen::Matrix<double, 6, 1> p_res;
  
  p_res << 0.66419e-310, 4.66419e-310, 1.033, 1.5708, 0, 0;

  const int LAST_JOINT = 6;

	Eigen::Map<Eigen::Matrix<double, 6, 1>> p(panda_robot.getJntPose(q, LAST_JOINT).data());

  const double TOLLERANCE = 1e-5;

  ASSERT_LE((p_res - p).norm(), TOLLERANCE);

}


TEST(RobotTest, ReturnCoriolis) {
  
 	rkd::Robot panda_robot("../urdf/panda.urdf", "panda_link0", "panda_hand");

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> dq{0, 0, 0, 0, 0, 0, 0};
  Eigen::Matrix<double, 7, 1> C_res;
  
  C_res << 0, 0, 0, 0, 0, 0,  0;

	Eigen::Map<Eigen::Matrix<double, 7, 1>> C(panda_robot.getCoriolis(q, dq).data());

  const double TOLLERANCE = 1e-5;

  ASSERT_LE((C_res - C).norm(), TOLLERANCE);

}
