#include <RKD/Robot.h>

using namespace RKD;

Robot::Robot(){}

Robot::~Robot(){}

Robot::Robot(const Robot& robot){

	this->chain_ = robot.chain_;
	this->chainPL_ = robot.chainPL_;
	this->chainPtr_ = robot.chainPtr_;
	this->chain_len_ = robot.chain_len_;
	this->last_q_ = robot.last_q_;
	this->q_min_ = robot.q_min_;
	this->q_max_ = robot.q_max_;
	this->q_nominal_ = robot.q_nominal_;
	this->f_init_ = robot.f_init_;
	this->f_chainPL_ = robot.f_chainPL_;
	this->joint_handles_ = robot.joint_handles_;
}

Robot::Robot(const std::string& urdf_path, const std::string& chain_root, const std::string& chain_tip){

	// Load the model
	urdf::ModelInterfaceSharedPtr model = getURDFModel(urdf_path);

	if (model){
		f_init_ = loadRobot(model, chain_root, chain_tip);

		if (f_init_){

			//------------------------------------------------------
			// Get number of joints in the chain
			//------------------------------------------------------
			chain_len_ = chain_.getNrOfJoints();

			// Get Pointer of the chain
			chainPtr_ = &chain_;

			//------------------------------------------------------
			// Resize variables
			//------------------------------------------------------
			q_min_.resize(chain_len_);
			q_max_.resize(chain_len_);
			q_nominal_.resize(chain_len_);

			//------------------------------------------------------
			// Get params from the model
			//------------------------------------------------------

			for (uint idx = 0; idx < chain_len_; idx++) {
				// Get Joint Handles by name
				std::string joint_handle = chainPtr_->getSegment(idx).getJoint().getName();
				joint_handles_.push_back(joint_handle);
				// Get Joint parameters
				urdf::JointConstSharedPtr joint =  model->getJoint(joint_handle);
				q_min_(idx) = joint->limits->lower;
				q_max_(idx) = joint->limits->upper;
				q_nominal_(idx) = (q_min_(idx) + q_max_(idx)) / 2.0;
			}	

			// Initialise last configuration
			last_q_ = q_nominal_;

			//------------------------------------------------------
			// Initialise TRAC-IK
			//------------------------------------------------------
	 		tracik_solver_.reset(new TRAC_IK::TRAC_IK(chain_, q_min_, q_max_));
			if (!tracik_solver_->getKDLChain(chain_)){
				std::cout << "TRAC_IK ERROR" << std::endl;
				f_init_ = false;
				return;
			}

			// Dummy iteration for initialisisation purposes
			KDL::Frame end_effector_pose;
			KDL::JntArray result;

			end_effector_pose.M = KDL::Rotation::Quaternion(1.0, 0, 0, 0);
			end_effector_pose.p = KDL::Vector(0.2*sin(2 * M_PI), 0, .1);
			tracik_solver_->CartToJnt(q_nominal_, end_effector_pose, result);
		}
	}else{
		std::cout << "Model not loaded!" << std::endl;
		f_init_ = false;	
	}	
}

urdf::ModelInterfaceSharedPtr Robot::getURDFModel(const std::string& urdf_path){
  
	std::ifstream urdf_file(urdf_path); //taking file as inputstream
	std::string urdf_str;

	urdf::ModelInterfaceSharedPtr model; 

	if(urdf_file) {
		std::ostringstream ss;

		ss << urdf_file.rdbuf(); 

		urdf_str = ss.str();

		model = urdf::parseURDF(urdf_str);
	}
	else{
		std::cout << "[ERROR] The URDF file does not exist!!!" << std::endl;
	}

	return model;
}

bool Robot::loadRobot(urdf::ModelInterfaceSharedPtr model, const std::string& chain_root, const std::string& chain_tip){

	// Load the tree
	KDL::Tree tree;

	// Generate Tree from URDF
	if (!kdl_parser::treeFromUrdfModel(*model, tree)){
		std::cout << "Failed to construct KDL tree" << std::endl;
		return false;
	}
	
	//Extract the chain from the tree ("chain_root" and "chain_tip" )
	if (!tree.getChain(chain_root.c_str(), chain_tip.c_str(), chain_)){
		std::cout << "Error retrieving the " << chain_root << " and/or " << chain_tip << " from the chain." << std::endl;
	}
	else{
		std::cout << "Extracted " << chain_.getNrOfJoints() << " joints from the chain of the KDL tree" << std::endl;
	}

	return true;
}

std::vector<double> Robot::getIK(std::vector<double> const& end_effector_pose, bool const near, Parameterisation const param){

	std::vector<double> q_res(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){

		KDL::JntArray q(chain_len_);
		
		//------------------------------------------------------
		// Initialise the structure of IK algorithm
		//------------------------------------------------------
		KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
		KDL::ChainIkSolverVel_pinv iksolver(*chainPtr_);
		KDL::ChainIkSolverPos_NR_JL idsolver(*chainPtr_, q_min_, q_max_, fksolver, iksolver);

		//------------------------------------------------------
		// Define the initial value for the algorithm
		//------------------------------------------------------
		KDL::JntArray q_init(chain_len_);

		//------------------------------------------------------
		// Check if we want to look for a new configuration near the previous one
		//------------------------------------------------------
		if (near && last_q_.rows() != 0){
			q_init = last_q_;
		}
		else{
			q_init = q_nominal_;
		}

		//------------------------------------------------------
		// Call Cartesian to Joint function
		//------------------------------------------------------
		KDL::Frame ee_frame = PoseToFrame(end_effector_pose, param);

		idsolver.CartToJnt(q_init, ee_frame, q);

		//------------------------------------------------------
		// Store current configuration
		//------------------------------------------------------
		last_q_ = q;

		// Move to result
		for (uint idx = chain_len_; idx--;){
			q_res[idx] = q.data[idx];
		}
	}
	else{
		std::cerr<< "Robot class has not initialized yet." << std::endl;
	}

	return q_res;
}


KDL::Frame Robot::PoseToFrame(std::vector<double> const& pose, Parameterisation const param){

	KDL::Frame res;
	
	// Position
	res.p[0] = pose[0];
	res.p[1] = pose[1];
	res.p[2] = pose[2];

	switch(param){
		case QUATERNION: {KDL::Rotation::Quaternion(pose[3], pose[4], pose[5], pose[6]); } break;
		case RPY: {KDL::Rotation::RPY(pose[3], pose[4], pose[5]); } break;
		default:
			std::cerr << "Parameterisation: " << param << " not recognised"<< std::endl;
	}

	return res;
}

std::vector<double> Robot::getTRAC_IK(std::vector<double> const& end_effector_pose, int& rc, bool near, Parameterisation const param){
		
	std::vector<double> q_init(chain_len_);

	if (near && last_q_.rows() != 0){
			// Move to result
			for (uint idx = chain_len_; idx--;){
				q_init[idx] = last_q_.data[idx];
			}
		}
		else{
			// Move to result
			for (uint idx = chain_len_; idx--;){
				q_init[idx] = q_nominal_.data[idx];
			}
		}

	return getTRAC_IK(end_effector_pose, rc, q_init, param);

}

std::vector<double> Robot::getTRAC_IK(std::vector<double> const& end_effector_pose, int& rc, std::vector<double> const& q_init, Parameterisation const param){
  
	std::vector<double> q_res(chain_len_);

    double elapsed = 0.0;
    const double timeout = IK_TIMEOUT;
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){

		KDL::JntArray q(chain_len_);
		KDL::JntArray q_initKDL(chain_len_);
		KDL::Frame ee_frame = PoseToFrame(end_effector_pose, param);

		// Copy q_init
		for (uint idx = chain_len_; idx--;){
			q_initKDL.data[idx] = q_init[idx];
		}

		start_time = boost::posix_time::microsec_clock::local_time();

		do{
			q = q_initKDL;
			rc = tracik_solver_->CartToJnt(q_initKDL, ee_frame, q);

			diff = boost::posix_time::microsec_clock::local_time() - start_time;
	      	elapsed = diff.total_nanoseconds() / 1e9;

	    }while (rc < 0 && elapsed < timeout);
		
		//------------------------------------------------------
		// Store next configuration
		//------------------------------------------------------
	    last_q_ = q;

		//------------------------------------------------------
		// Copy to result the final configuration
		//------------------------------------------------------
		for (uint idx = chain_len_; idx--;){
			q_res[idx] = q.data[idx];
		}
	}else{
		std::cout << "Robot not initialised!" << std::endl;
	}
		
    return q_res;

}

std::vector<double> Robot::getJacobian(std::vector<double> const& q){

	std::vector<double> J_res;

	KDL::JntArray q_kdl(q.size());

	for (uint i = 0; i < q.size(); ++i)
		q_kdl.data(i) = q[i];

	KDL::Jacobian J(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainJntToJacSolver jsolver(*chainPtr_);

		//------------------------------------------------------
		// Call Joint to Jacobian function
		//------------------------------------------------------
		jsolver.JntToJac(q_kdl, J);
	}
	else{
		std::cerr << "Robot class has not initialized yet." << std::endl;
		// TODO: does this raise an error?	
	}

	for (uint i = 0; i < chain_len_; ++i)
		for (uint j = 0; j < SIZE_POSE; ++j)
			J_res.push_back(J.data(j, i));

	return J_res;	
}

KDL::JntSpaceInertiaMatrix Robot::getInertia(const KDL::JntArray q){

	KDL::JntSpaceInertiaMatrix H(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainDynParam DChain(*chainPtr_, KDL::Vector(0, 0, -9.81));

		//------------------------------------------------------
		// Call Joint to Inertia matrix function
		//------------------------------------------------------
		DChain.JntToMass(q, H);
	}
	else
		std::cerr << "Robot class has not initialized yet." << std::endl;

	return H;
}

KDL::JntArray Robot::getCoriolis(const KDL::JntArray q, const KDL::JntArray dq){

	KDL::JntArray C(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainDynParam DChain(*chainPtr_, KDL::Vector(0,0,-9.81));
		//------------------------------------------------------
		// Call Joint to Coriolis matrix function
		//------------------------------------------------------
		DChain.JntToCoriolis(q, dq, C);
	}
	else
		std::cerr << "Robot class has not initialized yet." << std::endl;

	return C;
}

KDL::JntArray Robot::getGravity(const KDL::JntArray q){

	KDL::JntArray G(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainDynParam DChain(*chainPtr_, KDL::Vector(0, 0, -9.81));
		//------------------------------------------------------
		// Call Joint to Gravity vector function
		//------------------------------------------------------
		DChain.JntToGravity(q, G);
	}
	else
		std::cerr << "Robot class has not initialized yet." << std::endl;

	return G;
}

bool Robot::addPayload(const KDL::Frame &f_tip, const KDL::RigidBodyInertia &I){

	//------------------------------------------------------
	// Check whether a payload is already in place or not
	//------------------------------------------------------
	if (f_chainPL_){
		std::cerr<< "Payload already in the chain." << std::endl;
		return false;
	}

	//------------------------------------------------------
	// Create a new segment as input
	//------------------------------------------------------
	KDL::Segment pl_seg("payload", chain_.getSegment(chain_len_).getJoint(), f_tip, I);

	//------------------------------------------------------
	// Copy the structure of the current chain
	//------------------------------------------------------
	chainPL_ = chain_;

	//------------------------------------------------------
	// Add the new segment to the chain
	//------------------------------------------------------
	chainPL_.addSegment(pl_seg);

	//------------------------------------------------------
	// Enable chain + payload pointer
	//------------------------------------------------------
	f_chainPL_ = true;

	//------------------------------------------------------
	// Update the chain pointer
	//------------------------------------------------------

	chainPtr_ = &chainPL_;
	
	return true;
}

bool Robot::removePayload(){
	//------------------------------------------------------
	// Disable chain + payload pointer
	//------------------------------------------------------
	f_chainPL_ = false;

	//------------------------------------------------------
	// Update the chain pointer
	//------------------------------------------------------
	chainPtr_ = &chain_;

	return true;
}

bool Robot::getJntPose(const KDL::JntArray q, std::vector< KDL::Frame > &segFrames){

	//------------------------------------------------------
	// Initialise IK structures
	//------------------------------------------------------
	KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
	KDL::Frame T;

	for (uint i = 0; i < chain_len_; ++i){
		//------------------------------------------------------
		// Call Joint To Cartesian functino on the i-th joint
		//------------------------------------------------------
		fksolver.JntToCart(q, T, i);

		//------------------------------------------------------
		// Store the i-th joint's frame
		//------------------------------------------------------
		segFrames.push_back(T);
	}

	return true;
}

bool Robot::getJntPose(const Eigen::VectorXd& q, std::vector< Eigen::VectorXd > &segFrames){


	//------------------------------------------------------
	// Initialise IK structures
	//------------------------------------------------------
	KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
	KDL::Frame T;

	KDL::JntArray q_kdl(chain_len_);
	q_kdl.data = q;

	for (uint i = 0; i < chain_len_; ++i){
		//------------------------------------------------------
		// Call Joint To Cartesian functino on the i-th joint
		//------------------------------------------------------
		fksolver.JntToCart(q_kdl, T, i);

		//------------------------------------------------------
		// Convert Frame to Vector
		//------------------------------------------------------
		Eigen::VectorXd T_eigen(6);

		T_eigen(0) = T.p.data[0];
		T_eigen(1) = T.p.data[1];
		T_eigen(2) = T.p.data[2];

		T.M.GetRPY (T_eigen(3), T_eigen(4), T_eigen(5));

		//------------------------------------------------------
		// Store the i-th joint's frame
		//------------------------------------------------------
		segFrames.push_back(T_eigen);
	}

	return true;
}


std::vector<double> Robot::getJntPose(std::vector<double> const& q, int idx){

	std::vector<double> T_out(SIZE_POSE);

	if(f_init_){

		//------------------------------------------------------
		// Initialise FK structures
		//------------------------------------------------------
		KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
		KDL::Frame T;
		
		KDL::JntArray q_kdl(chain_len_);
		
		for (int i = q.size(); i--;){
			q_kdl.data(i) = q[i];
		}

		//------------------------------------------------------
		// Call Joint To Cartesian functino on the i-th joint
		//------------------------------------------------------
		fksolver.JntToCart(q_kdl, T, idx);

		T_out[0] = T.p.data[0];
		T_out[1] = T.p.data[1];
		T_out[2] = T.p.data[2];
		
		T.M.GetRPY(T_out[3], T_out[4], T_out[5]);
	}
	else{
		std::cerr << "Robot class has not been initialized yet." << std::endl;
	}

	return T_out;
}

bool Robot::good(){
	return f_init_;
}

std::vector<double> Robot::getFK(std::vector<double> const& q){

	return getJntPose(q, chain_len_);
	
}
