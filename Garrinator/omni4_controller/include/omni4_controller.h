#ifndef OMNI4_CONTROLLER_H__
#define OMNI4_CONTROLLER_H__

// std c++
#include <iostream>
#include <cmath>
#include <iomanip>

// Eigen
#include <eigen3/Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace omni4_controller
{

class Omni4Controller : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
	public:
		// constructor and destructor. Nothing to do
		Omni4Controller(){};
		~Omni4Controller(){};

		/**
		* \brief Initialize controller
		* \param hw			Velocity joint interface for all joints
		* \param root_nh		Node handle at root namespace
		* \param controller_nh		Node handle inside the controller namespace
		*/
		bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

		/**
		* \brief Updates controller, i.e. sets the new velocity commands for all joints
		* \param time		Current time
		* \param period		Time since the last called to update
		*/
		void update(const ros::Time& time, const ros::Duration& period);

		/**
		* \brief Starts controller
		* \param time		Current time
		*/
		void starting(const ros::Time& time);

		/**
		* \brief Stops controller
		* \param time		Current time
		*/
		void stopping(const ros::Time& time);

    private:
		// input twist comes here
		ros::Subscriber twist_subscriber_;

		// wheel drive joints
		hardware_interface::JointHandle joint_front_left_;
		hardware_interface::JointHandle joint_front_right_;
		hardware_interface::JointHandle joint_back_left_;
		hardware_interface::JointHandle joint_back_right_;

		// kinematics
		float_t L_;// L = distance between wheels in a square shaped base
		float_t w_rad_;// wheel radius
		Eigen::MatrixXd ik_;

		//twist command RT buffer
		realtime_tools::RealtimeBuffer<Eigen::Vector3d> command_buffer_;

		//command twist callback
		void commandTwistCallback(const geometry_msgs::Twist& _twist);

};

} //end of namespace

PLUGINLIB_EXPORT_CLASS(omni4_controller::Omni4Controller, controller_interface::ControllerBase);

#endif
