#include "omni4_controller.h"

namespace omni4_controller
{

bool Omni4Controller::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
	// for each joint, get name and sets joint handle
	std::string joint_name;
    if (!controller_nh.getParam("rim_front_left_joint", joint_name))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    joint_front_left_ = hw->getHandle(joint_name);  // throws on failure

		//-- next joint
		if (!controller_nh.getParam("rim_front_right_joint", joint_name))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    joint_front_right_ = hw->getHandle(joint_name);  // throws on failure

		//-- next joint
		if (!controller_nh.getParam("rim_back_left_joint", joint_name))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    joint_back_left_ = hw->getHandle(joint_name);  // throws on failure

		//-- next joint
		if (!controller_nh.getParam("rim_back_right_joint", joint_name))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    joint_back_right_ = hw->getHandle(joint_name);  // throws on failure

	//initializes kinematics parameters: constants required to solve the kinematics equations

	/*
	IK=[1/sqrt(2) 1/sqrt(2) L/sqrt(2);
	-1/sqrt(2) 1/sqrt(2) L/sqrt(2);
	-1/sqrt(2) -1/sqrt(2) L/sqrt(2);
	1/sqrt(2) -1/sqrt(2) L/sqrt(2);];

	*/
	L_ = 0.3;//300x300mm
	w_rad_=0.03;// 60 mm diameter wheel
	float_t sqrt_2=sqrt(2);
	ik_.resize(4,3);
	ik_(0,0)=1/sqrt_2;  ik_(0,1)=1/sqrt_2 ; ik_(0,2)= L_/sqrt_2;
	ik_(1,0)=-1/sqrt_2; ik_(1,1)= 1/sqrt_2; ik_(1,2)= L_/sqrt_2;
	ik_(2,0)=-1/sqrt_2; ik_(2,1)=-1/sqrt_2; ik_(2,2)= L_/sqrt_2;
	ik_(3,0)=1/sqrt_2;  ik_(3,1)=-1/sqrt_2; ik_(3,2)= L_/sqrt_2;


    // Init twist subscriber
	twist_subscriber_ = controller_nh.subscribe("robot/cmd_vel", 1, &Omni4Controller::commandTwistCallback, this);

	// returns
	ROS_INFO("omni4_controller loaded successfully");//DEBUG MESSAGE
    return true;
}

void Omni4Controller::update(const ros::Time& time, const ros::Duration& period)
{
	Eigen::Vector3d twist;
	Eigen::VectorXd velocities(4);
	double w_fl, w_fr, w_bl, w_br; //joint (wheel) speeds [rad/s]

	// get twist from the buffer
	twist = *(command_buffer_.readFromRT());

	//Compute the inverse kinematics to obtain joint speeds
	velocities=ik_*twist;// velocities in m/s


	velocities=velocities/w_rad_;// m/s to rad/s
	//
	w_fl=velocities(0);
	w_fr=velocities(1);
	w_br=velocities(2);
	w_bl=velocities(3);


	// Set joint speeds
	joint_front_left_.setCommand(w_fl);
	joint_front_right_.setCommand(w_fr);
	joint_back_left_.setCommand(w_bl);
	joint_back_right_.setCommand(w_br);


}

void Omni4Controller::starting(const ros::Time& time)
{
	// set zero to all joints
	joint_front_left_.setCommand(0);
	joint_front_right_.setCommand(0);
	joint_back_left_.setCommand(0);
	joint_back_right_.setCommand(0);

	// init RT command buffer
	command_buffer_.initRT( Eigen::Vector3d(0,0,0) );
}

void Omni4Controller::stopping(const ros::Time& time)
{
	ROS_INFO("Shutting down omni4_controller");//DEBUG MESSAGE
	// set zero to all joints
	joint_front_left_.setCommand(0);
	joint_front_right_.setCommand(0);
	joint_back_left_.setCommand(0);
	joint_back_right_.setCommand(0);


}

void Omni4Controller::commandTwistCallback(const geometry_msgs::Twist& _twist)
{
	// sets the incoming twist to command buffer
	command_buffer_.writeFromNonRT( Eigen::Vector3d(_twist.linear.x,_twist.linear.y,_twist.angular.z) );
}

}  // end of namespace
