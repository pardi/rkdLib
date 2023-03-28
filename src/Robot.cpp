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

std::vector<double> Robot::getIK(const std::vector<double>& end_effector_pose, const bool near, const Parameterisation param){

	KDL::JntArray q(chain_len_);
	
	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		std::cout << "XXX1" << std::endl;
		//------------------------------------------------------
		// Initialise the structure of IK algorithm
		//------------------------------------------------------
		KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
		std::cout << "XXX1.2" << std::endl;
		KDL::ChainIkSolverVel_pinv iksolver(*chainPtr_);
		std::cout << "XXX1.2" << std::endl;
		KDL::ChainIkSolverPos_NR_JL idsolver(*chainPtr_, q_min_, q_max_, fksolver, iksolver);

		std::cout << "XXX1.2" << std::endl;
		//------------------------------------------------------
		// Define the initial value for the algorithm
		//------------------------------------------------------
		KDL::JntArray q_init(chain_len_);
		std::cout << "XXX2" << std::endl;
		
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
		std::cout << "XXX3" << std::endl;
		idsolver.CartToJnt(q_init, ee_frame, q);

		//------------------------------------------------------
		// Store current configuration
		//------------------------------------------------------
		last_q_ = q;

	}
	else{
		std::cerr<< "Robot class has not initialized yet." << std::endl;
	}
	std::cout << "XXX" << std::endl;

	return vectorSerialisation(q);
}


std::vector<double> Robot::getTRAC_IK(const std::vector<double>& end_effector_pose, int rc, bool near, Parameterisation const param){
		
	std::vector<double> q_init(chain_len_);

	if (near && last_q_.rows() != 0){
			q_init = vectorSerialisation(last_q_);
		}
		else{
			q_init = vectorSerialisation(q_nominal_);
		}

	return getTRAC_IK(end_effector_pose, rc, q_init, param);

}

std::vector<double> Robot::getTRAC_IK(const std::vector<double>& end_effector_pose, int rc, const std::vector<double>& q_init, Parameterisation const param){
  
	KDL::JntArray q(chain_len_);

    double elapsed = 0.0;
    const double timeout = IK_TIMEOUT;
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){

		
		KDL::JntArray q_initKDL(chain_len_);
		KDL::Frame ee_frame = PoseToFrame(end_effector_pose, param);

		// Copy q_init
		q_initKDL = vectorDeserialisation(q_init);

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

	}else{
		std::cout << "Robot not initialised!" << std::endl;
	}
		
    return vectorSerialisation(q);

}

std::vector<double> Robot::getJacobian(const std::vector<double>& q){

	KDL::Jacobian J(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		KDL::JntArray q_kdl = vectorDeserialisation(q);

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

	return matrixSerialisation(J);	
}

std::vector<double> Robot::getInertia(const std::vector<double>& q, const std::vector<double>& gravity_vec){

	KDL::JntSpaceInertiaMatrix M(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		KDL::JntArray q_kdl = vectorDeserialisation(q);
		
		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainDynParam DChain(*chainPtr_, KDL::Vector(gravity_vec[0], gravity_vec[1], gravity_vec[2]));

		//------------------------------------------------------
		// Call Joint to Inertia matrix function
		//------------------------------------------------------
		DChain.JntToMass(q_kdl, M);
	}
	else{
		std::cerr << "Robot class has not initialized yet." << std::endl;
	}

	return matrixSerialisation(M);
}

std::vector<double> Robot::getCoriolis(const std::vector<double>& q, const std::vector<double>& dq, const std::vector<double>& gravity_vec){

	KDL::JntArray C(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){

		KDL::JntArray q_kdl = vectorDeserialisation(q);
		KDL::JntArray dq_kdl = vectorDeserialisation(dq);

		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainDynParam DChain(*chainPtr_, KDL::Vector(gravity_vec[0], gravity_vec[1], gravity_vec[2]));
		
		//------------------------------------------------------
		// Call Joint to Coriolis matrix function
		//------------------------------------------------------
		DChain.JntToCoriolis(q_kdl, dq_kdl, C);
	}
	else{
		std::cerr << "Robot class has not initialized yet." << std::endl;
	}

	return vectorSerialisation(C);
}

std::vector<double> Robot::getGravity(const std::vector<double>& q, const std::vector<double>& gravity_vec){

	KDL::JntArray G(chain_len_);

	//------------------------------------------------------
	// Check status of the class
	//------------------------------------------------------
	if (f_init_){
		KDL::JntArray q_kdl = vectorDeserialisation(q);
		//------------------------------------------------------
		// Initialise structure
		//------------------------------------------------------
		KDL::ChainDynParam DChain(*chainPtr_, KDL::Vector(gravity_vec[0], gravity_vec[1], gravity_vec[2]));
		
		//------------------------------------------------------
		// Call Joint to Gravity vector function
		//------------------------------------------------------
		DChain.JntToGravity(q_kdl, G);
	}
	else{
		std::cerr << "Robot class has not initialized yet." << std::endl;
	}

	std::cout << "XXX" << std::endl;

	return vectorSerialisation(G);
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

bool Robot::getJntsPose(const std::vector<double>& q, std::vector<std::vector<double> > &segFrames){

	std::vector<double> T_res(SIZE_POSE);

	if(f_init_){

		//------------------------------------------------------
		// Initialise IK structures
		//------------------------------------------------------
		KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
		KDL::Frame T;

		KDL::JntArray q_kdl = vectorDeserialisation(q);

		for (uint i = 0; i < chain_len_; ++i){
			//------------------------------------------------------
			// Call Joint To Cartesian functino on the i-th joint
			//------------------------------------------------------
			fksolver.JntToCart(q_kdl, T, i);

			//------------------------------------------------------
			// Convert Frame to Vector
			//------------------------------------------------------

			T_res[0] = T.p.data[0];
			T_res[1] = T.p.data[1];
			T_res[2] = T.p.data[2];

			T.M.GetRPY(T_res[3], T_res[4], T_res[5]);

			//------------------------------------------------------
			// Store the i-th joint's frame
			//------------------------------------------------------
			segFrames.push_back(T_res);
		}
	}
	else{
		std::cerr << "Robot class has not initialized yet." << std::endl;
		return false;
	}

	return true;
}

std::vector<double> Robot::getJntPose(const std::vector<double>& q, int idx){

	std::vector<double> T_out(SIZE_POSE);

	if(f_init_){

		//------------------------------------------------------
		// Initialise FK structures
		//------------------------------------------------------
		KDL::ChainFkSolverPos_recursive fksolver(*chainPtr_);
		KDL::Frame T;
		
		KDL::JntArray q_kdl = vectorDeserialisation(q);
		
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

std::vector<double> Robot::getFK(const std::vector<double>& q){

	return getJntPose(q, chain_len_);
	
}

std::vector<double> Robot::vectorSerialisation(const KDL::JntArray& v){

	std::vector<double> v_res(v.data.size());

	for (uint idx = v.data.size(); idx--;){
		v_res[idx] = v.data(idx);
	}

	return v_res;
}

KDL::JntArray Robot::vectorDeserialisation(const std::vector<double>& v){

	KDL::JntArray v_res(v.size());
	
	for (uint idx = v.size(); idx--;){
		v_res.data(idx) = v[idx];
	}

	return v_res;
}

std::vector<double> Robot::matrixSerialisation(const KDL::JntSpaceInertiaMatrix& M){

	std::vector<double> M_res(chain_len_ * SIZE_POSE);

	for (uint i = 0; i < M.data.cols() ; ++i){
		for (uint j = 0; j < M.data.rows(); ++j){
			M_res[i * chain_len_ + j] = M(i, j);
		}
	}

	return M_res;
}

std::vector<double> Robot::matrixSerialisation(const KDL::Jacobian& M){

	std::vector<double> M_res(chain_len_ * SIZE_POSE);

	for (uint i = 0; i < M.data.cols() ; ++i){
		for (uint j = 0; j < M.data.rows(); ++j){
			M_res[i * chain_len_ + j] = M(i, j);
		}
	}

	return M_res;
}

KDL::Frame Robot::PoseToFrame(const std::vector<double>& pose, const Parameterisation param){

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