#include <RKD/Robot.h>

void test_getfk(RKD::Robot& panda_robot){
	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};

	// Result
	// 0.088
	// -3.24185e-17
	// 0.926
	// 0.785398
	// 3.14018e-16
	// 3.14159

	Eigen::Map<Eigen::Matrix<double, 6, 1>> p(panda_robot.getFK(q).data());

	std::cout << p << std::endl;
	
}

void test_getik(RKD::Robot& panda_robot){
	std::vector<double> p_vec{0, 0, 0.9, 0, 0, 0, 0};

	// Result

	std::vector<double> q_vec = panda_robot.getIK(p_vec);
	
	Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_vec.data());

	std::cout << q << std::endl;

	Eigen::Map<Eigen::Matrix<double, 6, 1>> p(panda_robot.getFK(q_vec).data());
	
	std::cout << p << std::endl;
	
}

void test_getJntPose(RKD::Robot& panda_robot){
	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};

// Result
// 	          0
// 4.66026e-310
//        1.033
//       1.5708
//           -0
//            0

	Eigen::Map<Eigen::Matrix<double, 6, 1>> p(panda_robot.getJntPose(q, 6).data());

	std::cout << p << std::endl;
	
}


void test_getJacobian(RKD::Robot& panda_robot){

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};

//            0        0.593 -3.77476e-17       -0.277  4.75175e-17        0.107 -6.16298e-33
//  4.6526e-310  1.95399e-17        0.088  1.22125e-18        0.088  1.95399e-17            0
//            0       -0.088            0       0.0055            0        0.088            0
//            0            0            0            0            0            0            0
//            0            1            0           -1            0           -1 -4.44089e-16
//            1  2.22045e-16            1  2.22045e-16            1  2.22045e-16           -1

	Eigen::Map<Eigen::Matrix<double, 6, 7>> J(panda_robot.getJacobian(q).data());

	std::cout << J << std::endl;

}
void test_getInertia(RKD::Robot& panda_robot){

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};

	Eigen::Map<Eigen::Matrix<double, 6, 7>> M(panda_robot.getInertia(q).data());

	std::cout << M << std::endl;

}


void test_getCoriolis(RKD::Robot& panda_robot){

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};
	std::vector<double> dq{0, 0, 0, 0, 0, 0, 0};

	Eigen::Map<Eigen::Matrix<double, 7, 1>> C(panda_robot.getCoriolis(q, dq).data());

	std::cout << C << std::endl;

}

void test_getGravit(RKD::Robot& panda_robot){

	std::vector<double> q{0, 0, 0, 0, 0, 0, 0};

	Eigen::Map<Eigen::Matrix<double, 7, 1>> g(panda_robot.getGravity(q).data());

	std::cout << g << std::endl;

}


int main(){

	RKD::Robot panda_robot("../urdf/panda.urdf", "panda_link0", "panda_hand");
	
	std::cout << "--FK--" << std::endl;
	test_getfk(panda_robot);

	std::cout << "--JntPose--" << std::endl;
	test_getJntPose(panda_robot);

	std::cout << "--Jacobian--" << std::endl;
	test_getJacobian(panda_robot);	

	std::cout << "--IK--" << std::endl;
	test_getik(panda_robot);	

	std::cout << "--Inertia--" << std::endl;
	test_getInertia(panda_robot);	

	std::cout << "--Coriolis--" << std::endl;
	test_getCoriolis(panda_robot);	

	std::cout << "--Gravity--" << std::endl;
	test_getGravit(panda_robot);	
	

	return 0;
}