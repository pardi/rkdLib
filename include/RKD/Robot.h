#ifndef RKDLIB_M_HEADER_COMMON
#define RKDLIB_M_HEADER_COMMON

#define SIZE_POSE 6
#define IK_TIMEOUT 0.005
#define STANDARD_GRAVITY_VEC {0.0, 0.0, -9.81}

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Dense>
// #include <Quaternion.hpp>

// General Libs
#include <iostream>
#include <memory>
#include <fstream>
#include <vector>

// URDF
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
// KDL parser
#include <kdl_parser/kdl_parser.hpp>
// KDL Libs
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
// TRAC-IK Libs
#include <trac_ik/trac_ik.hpp>




namespace RKD{

enum Parameterisation{
	RPY = 0,
	QUATERNION = 1,
	ZYZ = 2};

	//!  Robot Kinematics and Dynamics Library - standalone
	/*!
	  This class provides the user with information about kinematics and dynamics of a robot chain
	*/
class Robot{

public:
	
	/*! \brief Robot Kinematics and Dynamics constructor
	 * 
	 *	Robot dynamics constructor - Empty
	 */
	Robot();

	/*! \brief Robot Kinematics and Dynamics Copy-constructor
	 * 
	 *	Copy constructor
	 */
	Robot(const Robot&);

	/*! \brief Robot Kinematics and Dynamics Constructor
	 * 
	 *	Robot dynamic constructor - urdf model, chain-root, and chain-tip
	 */
	Robot(const std::string&, const std::string&, const std::string&);

	/*! \brief Robot Kinematics and Dynamics distructor
	 * 
	 *	Robot Kinematics and Dynamics discrutuctor
	 */
	~Robot();

	/*! \brief Get IK 
	 * 
	 *	Get Inverse Kinematic from a given cartesian position
	 */
	std::vector<double> getIK(std::vector<double> const&, bool const near=true, Parameterisation const param=RPY);

	/*! \brief Get FK
	 * 
	 *	Get Forward Kinematic from a given configuration
	 */
	std::vector<double> getFK(std::vector<double> const&);

	/*! \brief Get IK fast
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik
	 */
	std::vector<double> getTRAC_IK(std::vector<double> const&, int&, bool near=false, Parameterisation const param=RPY);

	/*! \brief Get IK fast
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik, takes the initial configuration as input
	 */
	std::vector<double> getTRAC_IK(std::vector<double> const&, int&, std::vector<double> const&, Parameterisation const param=RPY);
	
	/*! \brief Get Jacobian of the chain
	 * 
	 *	Get Jacobian from a given configuration
	 */
	std::vector<double> getJacobian(std::vector<double> const&);

	/*! \brief Get Inertia Matrix
	 * 
	 *	Get Inertia matrix from a given configuration
	 */
	std::vector<double> getInertia(std::vector<double> const&, std::vector<double> const& gravity_vec = STANDARD_GRAVITY_VEC);
	
	/*! \brief Get Coriolis Matrix
	 * 
	 *	Get Coriolis Matrix given a joint configuration and joint velocities
	 */		
	std::vector<double> getCoriolis(std::vector<double> const&, std::vector<double> const&, std::vector<double> const& gravity_vec = STANDARD_GRAVITY_VEC);
	
	/*! \brief Get Gravity vector
	 * 
	 *	Get Gravity vector from a given configuration
	 */
	std::vector<double> getGravity(std::vector<double> const& q, std::vector<double> const& gravity_vec = STANDARD_GRAVITY_VEC);

	/*! \brief Load payload
	 * 
	 *	Add a payload to the existing robot chain
	 */
	bool addPayload(const KDL::Frame &f_tip=KDL::Frame::Identity(), const KDL::RigidBodyInertia &I=KDL::RigidBodyInertia::Zero());
	
	/*! \brief UnLoad payload
	 * 
	 *	Remove a payload to the existing robot chain
	 */
	bool removePayload();
	
	/*! \brief Get segment positions
	 * 
	 *	Get pose for all Joints in the chain - Eigen
	 */
	bool getJntsPose(std::vector<double> const&, std::vector<std::vector<double> > &);
	
	/*! \brief Get segment positions
	 * 
	 *	Get pose for a single Joint of the chain 
	 */
	std::vector<double> getJntPose(std::vector<double> const&, int idx = 0);
	
	/*! \brief Check if the robot is correctly loaded
	 * 
	 *	Check if the robot is enabled
	 */	
	bool good();
	
	/*! \brief Get the number of joints of the robot
	 * 
	 *	Get the number of joints of the robot
	 */	
	
	uint inline getNrOfJoints() const {
		return chain_.getNrOfJoints();
	}
	
private:
	
	/*! \brief Get urdf model from file
	 * 
	 *	Read and create a URDF::Model from file
	 */	
	urdf::ModelInterfaceSharedPtr getURDFModel(const std::string&);
	
	/*! \brief Load the Robot from URDF
	 * 
	 *	Generate a KDL chain and tree from an URDF file
	 */	
	bool loadRobot(urdf::ModelInterfaceSharedPtr, const std::string&, const std::string&);
	
	/*! \brief Convert a pose into a KDL::Frame
	 * 
	 *	Generate a KDL frame given a pose expressed as std::vector<double>
	 */	
	KDL::Frame PoseToFrame(std::vector<double> const& pose, Parameterisation const param = RPY);
	
	/*! \brief Convert a Inertia Matrix into a vector<double>
	 * 
	 *	Serialise the matrix of inertia into a std::vector<double> by column
	 */	
	std::vector<double> matrixSerialisation(KDL::JntSpaceInertiaMatrix const&);
	
	/*! \brief Convert a Jacobian Matrix into a vector<double>
	 * 
	 *	Serialise the jacobian matrix into a std::vector<double> by column
	 */	
	std::vector<double> matrixSerialisation(KDL::Jacobian const&);
	
	/*! \brief Convert a JntArray into a vector<double>
	 * 
	 *	Serialise the vector of jnt array into a std::vector<double>
	 */	
	std::vector<double> vectorSerialisation(KDL::JntArray const&);

	/*! \brief Convert a vector<double> into JntArray
	 * 
	 *	Deserialise the vecot of std::vector<double> into a jnt array
	 */	
	KDL::JntArray vectorDeserialisation(std::vector<double> const&);

	/*! \brief Pseudo Inverse of a matrix
	 * 
	 *	Compute the Pseudo Inverse of a matrix
	 */	
	static Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd M_, const bool& damped)
	{	
		double lambda_ = damped?0.1:0.0;

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
		Eigen::MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
		S_.setZero();

	    for (int i = 0; i < sing_vals_.size(); i++)
	        S_(i, i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

	    return Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
	}

	KDL::Chain chain_;
	KDL::Chain chainPL_;
	KDL::Chain* chainPtr_;
	uint chain_len_;
	KDL::JntArray q_min_;
	KDL::JntArray q_max_;
	KDL::JntArray last_q_;
	KDL::JntArray q_nominal_;
	std::vector<std::string> joint_handles_;
	bool f_init_{false};
	bool f_chainPL_{false};
	std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;

};
}

#endif // RobotLIB_M_HEADER_COMMON