// includes
#ifndef odrive_hw_if_h
#define odrive_hw_if_h

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <math.h>

namespace garrinator_hardware_interface
{

class OdriveHwIf : public hardware_interface::RobotHW
{

  public:
    std::vector<double> velocities_cmmd_;

    OdriveHwIf();// vector.resize(4);
    ~OdriveHwIf();// close serial + stop(velocitats)
    bool init(ros::NodeHandle& _root_nh,ros::NodeHandle& _robot_hw_nh);//open serial
    void read(const ros::Time& _time,const ros::Duration& _period );// tots i pujar a velocities i positions
    void write(const ros::Time& _time,const ros::Duration& _period );//
    void print() const;

  protected:
    // ==================HOMEWORK===================
    // Fixed port names to odrives
    //omni4 controller

    hardware_interface::JointStateInterface  state_joint_interface_;
    hardware_interface::VelocityJointInterface  velocity_joint_interface_;


    // serial and comm related
    int serial_id_od0_, serial_id_od1_;
    int ret_value_;//----
    termios ttySettings_1_, ttySettings_2_; //termios variable to configure serial port
    //termios stdInOldSettings_, stdInNewSettings_;

    std::string read_msg_;
    std::string token_;//--
    std::string delimiter_;//--
    std::string mensg_;// variable that will provide flexibility to the messages
    unsigned char byte_;

    std::vector<double> positions_fb_;
    std::vector<double> velocities_fb_;
    std::vector<double> efforts_fb_; // cannot be used

    float to_rad_;

    unsigned char readByte(const int & _serial_id);

};

}// end of namespace


#endif
