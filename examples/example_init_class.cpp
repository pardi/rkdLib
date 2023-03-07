#include <RKD/Robot.h>

int main(){

	RKD::Robot panda_robot("../urdf/panda.urdf", "panda_link0", "panda_hand");
	
	Eigen::VectorXd q(7);
	q << 0, 0, 0, 0, 0, 0, 0;

	Eigen::VectorXd p = panda_robot.getFK(q);

	std::cout << p << std::endl;

	return 0;
}