/********************************************************************************
Copyright (c) 2020, Tommaso Pardi All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef RKDLIB_M_HEADER_COMMON
#define RKDLIB_M_HEADER_COMMON

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


constexpr size_t kPoseSize = 6;
constexpr double kIkTimeout = 0.005;
const std::vector<double> kStandardGravityVec = {0.0, 0.0, -9.81};

namespace RKD{

enum class Parameterisation{
	RPY,
	QUATERNION,
	ZYZ,
    };

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
	 *	\param Robot object to be copied
	 *	\return Robot object
	 */
	Robot(const Robot&);

	/*! \brief Robot Kinematics and Dynamics Constructor
	 * 
	 *	Robot dynamic constructor - urdf model, chain-root, and chain-tip
	 *	\param urdf model
	 *	\param chain-root
     *  \param chain-tip
	 */
	Robot(const std::string&, const std::string&, const std::string&);

	/*! \brief Robot Kinematics and Dynamics destructor
	 * 
	 *	Robot Kinematics and Dynamics destructor
	 */
	~Robot();

	/*! \brief Get IK
	 * 
	 *	Get Inverse Kinematic from a given cartesian position
	 *	\param cartesian position
	 *	\param near - if true, the initial configuration is used as a starting point for the IK solver
	 *	\param param - parameterisation of the orientation
	 *	\return joint configuration
	 */
	std::vector<double> getIK(const std::vector<double>&, bool const near=true, const Parameterisation param=Parameterisation::RPY);

	/*! \brief Get FK
	 * 
	 *	Get Forward Kinematic from a given configuration
	 *	\param joint configuration
	 *	\return cartesian position
	 *
	 */
	std::vector<double> getFK(const std::vector<double>&);

	/*! \brief Get IK fast
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik
	 *	\param cartesian position
	 *	\param near - if true, the initial configuration is used as a starting point for the IK solver
	 *	\param param - parameterisation of the orientation
	 *	\return joint configuration
	 */
	std::vector<double> getTRAC_IK(const std::vector<double>&, int, bool near=false, Parameterisation const param=Parameterisation::RPY);

	/*! \brief Get IK fast
	 * 
	 *	Get Inverse Kinematic from a given cartesian position using trac-ik, takes the initial configuration as input
	 *	\param cartesian position
	 *  \param near - if true, the initial configuration is used as a starting point for the IK solver
	 *  \param initial configuration
	 *  \param param - parameterisation of the orientation
	 *  \return joint configuration
	 */
	std::vector<double> getTRAC_IK(const std::vector<double>&, int, const std::vector<double>&, Parameterisation const param=Parameterisation::RPY);
	
	/*! \brief Get Jacobian of the chain
	 * 
	 *	Get Jacobian from a given configuration
	 *	\param joint configuration
	 *	\return Jacobian
	 */
	std::vector<double> getJacobian(const std::vector<double>&);

	/*! \brief Get Inertia Matrix
	 * 
	 *	Get Inertia matrix from a given configuration
	 *	\param joint configuration
	 *	\param gravity vector
	 *	\return Inertia Matrix
	 */
	std::vector<double> getInertia(const std::vector<double>&, const std::vector<double>& gravity_vec = kStandardGravityVec);
	
	/*! \brief Get Coriolis Matrix
	 * 
	 *	Get Coriolis Matrix given a joint configuration and joint velocities
	 *	\param joint configuration
	 *	\param joint velocities
	 *	\param gravity vector
	 *	\return Coriolis Matrix
	 */		
	std::vector<double> getCoriolis(const std::vector<double>&, const std::vector<double>&, const std::vector<double>& gravity_vec = kStandardGravityVec);
	
	/*! \brief Get Gravity vector
	 * 
	 *	Get Gravity vector from a given configuration
	 *	\param joint configuration
	 *	\param gravity vector
	 *	\return Gravity vector
	 */
	std::vector<double> getGravity(const std::vector<double>& q, const std::vector<double>& gravity_vec = kStandardGravityVec);

	/*! \brief Load payload
	 * 
	 *	Add a payload to the existing robot chain
	 *	\param tip pose
	 *	\param payload inertia
	 *	\return true if successful
	 */
	bool addPayload(const KDL::Frame& f_tip=KDL::Frame::Identity(), const KDL::RigidBodyInertia &I=KDL::RigidBodyInertia::Zero());
	
	/*! \brief UnLoad payload
	 * 
	 *	Remove a payload to the existing robot chain
	 *	\return true if successful
	 */
	bool removePayload();
	
	/*! \brief Get segment positions
	 * 
	 *	Get pose for all Joints in the chain
	 *	\param joint configuration
	 *	\param vector of segment positions
	 *	\return true if successful
	 */
	bool getJntsPose(const std::vector<double>&, std::vector<std::vector<double> > &);
	
	/*! \brief Get segment positions
	 * 
	 *	Get pose for a single Joint of the chain
	 *	\param joint configuration
	 *	\param index of the joint
	 *	\return pose of the joint
	 */
	std::vector<double> getJntPose(const std::vector<double>&, int idx = 0);
	
	/*! \brief Check if the robot is correctly loaded
	 * 
	 *	Check if the robot is enabled
	 *	\return true if the robot is correctly loaded
	 */	
	bool good();
	
	/*! \brief Get the number of joints of the robot
	 * 
	 *	Get the number of joints of the robot
	 *	\return number of joints
	 */	
	
	std::uint8_t inline getNrOfJoints() const {
		return chain_.getNrOfJoints();
	}
	
private:
	
	/*! \brief Get urdf model from file
	 * 
	 *	Read and create a URDF::Model from file
	 *	\param path to the URDF file
	 *	\return URDF::Model
	 */	
	urdf::ModelInterfaceSharedPtr getURDFModel(const std::string&);
	
	/*! \brief Load the Robot from URDF
	 * 
	 *	Generate a KDL chain and tree from an URDF file
	 *	\param URDF::Model
	 *	\param name of the root link
	 *	\param name of the tip link
	 *	\return true if successful
	 */	
	bool loadRobot(urdf::ModelInterfaceSharedPtr, const std::string&, const std::string&);
	
	/*! \brief Convert a pose into a KDL::Frame
	 * 
	 *	Generate a KDL frame given a pose expressed as std::vector<double>
	 *	\param pose
	 *	\param param - parameterisation of the orientation
	 *	\return KDL::Frame
	 */	
	KDL::Frame PoseToFrame(const std::vector<double>& pose, const Parameterisation param = Parameterisation::RPY);
	
	/*! \brief Convert a Inertia Matrix into a vector<double>
	 * 
	 *	Serialise the matrix of inertia into a std::vector<double> by column
	 *	\param KDL::JntSpaceInertiaMatrix
	 *	\return std::vector<double>
	 */	
	std::vector<double> matrixSerialisation(const KDL::JntSpaceInertiaMatrix&);
	
	/*! \brief Convert a Jacobian Matrix into a vector<double>
	 * 
	 *	Serialise the jacobian matrix into a std::vector<double> by column
	 *	\param KDL::Jacobian
	 *	\return std::vector<double>
	 */	
	std::vector<double> matrixSerialisation(const KDL::Jacobian&);
	
	/*! \brief Convert a JntArray into a vector<double>
	 * 
	 *	Serialise the vector of jnt array into a std::vector<double>
	 *	\param KDL::JntArray
	 *	\return std::vector<double>
	 */	
	std::vector<double> vectorSerialisation(const KDL::JntArray&);

	/*! \brief Convert a vector<double> into JntArray
	 * 
	 *	Deserialise the vecot of std::vector<double> into a jnt array
	 *	\param std::vector<double>
	 *	\return KDL::JntArray
	 */	
	KDL::JntArray vectorDeserialisation(const std::vector<double>&);

	/*! \brief Pseudo Inverse of a matrix
	 * 
	 *	Compute the Pseudo Inverse of a matrix
	 *	\param Eigen::MatrixXd
	 *	\param bool - damped or not
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
	uint8_t chain_len_;
	KDL::JntArray q_min_;
	KDL::JntArray q_max_;
	KDL::JntArray last_q_;
	KDL::JntArray q_nominal_;
	std::vector<std::string> joint_handles_;
	bool f_init_{false};
	bool f_chainPL_{false};
	std::unique_ptr<trac_ik::TRAC_IK> tracik_solver_;

};
}

#endif // RobotLIB_M_HEADER_COMMON
