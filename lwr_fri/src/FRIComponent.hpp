// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 
// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>

// Author: Ruben Smits, Wilm Decre
// Maintainer: Ruben Smits, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef _FRI_COMPONENT_HPP_
#define _FRI_COMPONENT_HPP_

#include <string>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>


//#include <kuka_lwr_fri/typekit/Types.h>

typedef Eigen::Matrix<double, 7, 7> Matrix77d;

namespace lwr_fri {

using namespace RTT;

class FRIComponent: public RTT::TaskContext {
public:
	FRIComponent(const std::string& name);
	virtual ~FRIComponent();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

private:

	int fri_create_socket();
	int fri_recv();
	int fri_send();

	bool isPowerOn() { return m_msr_data.robot.power!=0; }

	tFriMsrData m_msr_data;
	tFriCmdData m_cmd_data;

	sensor_msgs::JointState m_joint_states;
	lwr_fri::FriJointState m_fri_joint_state;

	geometry_msgs::Pose m_cartPos;
	geometry_msgs::Twist m_cartTwist;
	geometry_msgs::Wrench m_cartWrench;

	lwr_fri::CartesianImpedance m_cartImp;

	//OutputPort<tFriKrlData> port_from_krl;
	//OutputPort<tFriKrlData> port_to_krl;
	Matrix77d m_massTmp;

	InputPort<std_msgs::Int32> port_krl_cmd;

	/**
	 * events
	 */
	OutputPort<std::string> port_events;

	/**fri_create_socket
	 * statistics
	 */
	OutputPort<tFriRobotState> port_robot_state;
	OutputPort<tFriIntfState> port_fri_state;

	/**
	 * Current robot data
	 */
	OutputPort<sensor_msgs::JointState> port_joint_state;
	OutputPort<lwr_fri::FriJointState> port_fri_joint_state;

	OutputPort<std::vector<double> > port_joint_pos_msr;
	OutputPort<std::vector<double> > port_joint_trq_msr;

  OutputPort<std::vector<double> > port_joint_pos_des;

	OutputPort<geometry_msgs::Pose>  port_cart_pos_msr;
	OutputPort<geometry_msgs::Wrench> port_cart_wrench_msr;
	RTT::OutputPort<KDL::Jacobian> port_jacobian;
	RTT::OutputPort<Matrix77d > massMatrixPort;
	
	OutputPort<double> port_command_period;

	/**
	 * Robot commands
	**/
	InputPort<std::vector<double> > port_joint_pos_command;
	InputPort<std::vector<double> > port_joint_vel_command;
	InputPort<std::vector<double> > port_joint_effort_command;
	InputPort<lwr_fri::FriJointImpedance> port_fri_joint_impedance;

	InputPort<geometry_msgs::Pose> port_cart_pos_command;
	InputPort<geometry_msgs::Twist> port_cart_vel_command;
	InputPort<geometry_msgs::Wrench> port_cart_wrench_command;
	InputPort<lwr_fri::CartesianImpedance> port_cart_impedance_command;

	KDL::Jacobian jac;

	std::vector<double> m_joint_pos;
	std::vector<double> m_joint_trq;

  std::vector<double> m_joint_pos_des;

	std::vector<double> m_joint_pos_command;
	std::vector<double> m_joint_vel_command;
	std::vector<double> m_joint_effort_command;
	lwr_fri::FriJointImpedance m_fri_joint_impedance;

	int prop_local_port, m_socket, m_remote_port, m_control_mode;
	std::string joint_names_prefix;
	uint16_t counter, fri_state_last;
	struct sockaddr_in m_remote_addr;
	socklen_t m_sock_addr_len;
};

}//Namespace LWR

#endif//_FRI_COMPONENT_HPP_

