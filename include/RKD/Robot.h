#ifndef RKDLIB_M_HEADER_COMMON
#define RKDLIB_M_HEADER_COMMON

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Dense>
// #include <Quaternion.hpp>
#include <fstream>
// General Libs
#include <iostream>
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
	KDL::JntArray getIK(const KDL::Frame&, const bool near=true);
	/*! \brief Get FK
	 * 
	 *	Get Forward Kinematic from a given configuration
	 */
	Eigen::VectorXd getFK(const Eigen::VectorXd&);
	/*! \brief Get IK fast
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik
	 */
	KDL::JntArray getTRAC_IK(const KDL::Frame&, int&, const bool near=false);
	/*! \brief Get IK fast - EIGEN
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik
	 */
	Eigen::VectorXd getTRAC_IK(const Eigen::Vector3d&, const Eigen::Quaterniond&, int&, const bool near = false);
	/*! \brief Get IK fast
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik, takes the initial configuration as input
	 */
	KDL::JntArray getTRAC_IK(const KDL::Frame&, int&, const KDL::JntArray&);
	/*! \brief Get IK fast - EIGEN
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik, takes the initial configuration as input
	 */
	Eigen::VectorXd getTRAC_IK(const Eigen::Vector3d&, const Eigen::Quaterniond&, int&, const Eigen::VectorXd&);

	/*! \brief Get Jacobian of the chain
	 * 
	 *	Get Jacobian from a given configuration
	 */
	KDL::Jacobian getJacobian(const KDL::JntArray);
	/*! \brief Get Jacobian of the chain
	 * 
	 *	Get Jacobian from a given configuration - Eigen
	 */
	Eigen::MatrixXd getJacobian(const Eigen::VectorXd&);
	/*! \brief Get Inertia Matrix
	 * 
	 *	Get Inertia matrix from a given configuration
	 */
	KDL::JntSpaceInertiaMatrix getInertia(const KDL::JntArray);
	/*! \brief Get Coriolis Matrix
	 * 
	 *	Get Coriolis Matrix given a joint configuration and joint velocities
	 */		
	KDL::JntArray getCoriolis(const KDL::JntArray, const KDL::JntArray);
	/*! \brief Get Gravity vector
	 * 
	 *	Get Gravity vector from a given configuration
	 */
	KDL::JntArray getGravity(const KDL::JntArray);
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
	bool getJntPose(const Eigen::VectorXd&, std::vector< Eigen::VectorXd > &);
	/*! \brief Get segment positions
	 * 
	 *	Get pose for a single Joint of the chain 
	 */
	Eigen::VectorXd getJntPose(const Eigen::VectorXd&, const int&);

	/*! \brief Get segment positions
	 * 
	 *	Get pose for all segments in the chain - KDL
	 */
	bool getJntPose(const KDL::JntArray, std::vector< KDL::Frame > &);
	/*! \brief Check if the robot is correctly loaded
	 * 
	 *	Check if the robot is enabled
	 */	
	bool good();
	/*! \brief Get the number of joints of the robot
	 * 
	 *	Get the number of joints of the robot
	 */	
	int inline getNrOfJoints() const {
		return chain_.getNrOfJoints();
	}
	/*! \brief Pseudo Inverse of a matrix
	 * 
	 *	Compute the Pseudo Inverse of a matrix
	 */	
	static Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &M_, const bool& damped)
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
	TRAC_IK::TRAC_IK* tracik_solver_;

};
}

#endif // RobotLIB_M_HEADER_COMMON