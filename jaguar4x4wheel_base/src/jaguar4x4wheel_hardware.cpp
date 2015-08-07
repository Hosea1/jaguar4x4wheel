/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "jaguar4x4wheel_base/jaguar4x4wheel_hardware.h"
#include <boost/assign/list_of.hpp>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace jaguar4x4wheel_base
{

  /**
  * Initialize Jaguar4x4Wheel hardware
  */
  Jaguar4x4WheelHardware::Jaguar4x4WheelHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
      : nh_(nh),
        private_nh_(private_nh)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3555);
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/prolific");

//    horizon_legacy::connect(port);
//    horizon_legacy::configureLimits(max_speed_, max_accel_);
    resetTravelOffset();
    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void Jaguar4x4WheelHardware::resetTravelOffset()
  {
//    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(polling_timeout_);
//    if (enc)
//    {
//      for (int i = 0; i < 4; i++)
//      {
//        joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
//      }
//    }
//    else
    {
      ROS_ERROR("Could not get encoder data to calibrate travel offset");
    }
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void Jaguar4x4WheelHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
        ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
          joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void Jaguar4x4WheelHardware::updateJointsFromHardware()
  {

//    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(polling_timeout_);
//    if (enc)
//    {
//      for (int i = 0; i < 4; i++)
//      {
//        double new_position = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position_offset;
//        double delta = new_position - joints_[i].position;

//        // detect encoder rollover
//        if (std::abs(delta) < 1.0)
//        {
//          joints_[i].position = new_position;
//        }
//        else
//        {
//          //  rollover has occured, swallow the measurement and update the offset
//          joints_[i].position_offset = delta;
//        }
//      }
//    }

//    horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed = horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(polling_timeout_);
//    if (speed)
//    {
//      for (int i = 0; i < 4; i++)
//      {
//        if (i % 2 == LEFT)
//        {
//          joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
//        }
//        else
//        { // assume RIGHT
//          joints_[i].velocity = linearToAngular(speed->getRightSpeed());
//        }
//      }
//    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void Jaguar4x4WheelHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    //horizon_legacy::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void Jaguar4x4WheelHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Jaguar4x4Wheel reports travel in metres, need radians for ros_control RobotHW
  */
  double Jaguar4x4WheelHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Jaguar4x4Wheel needs m/s,
  */
  double Jaguar4x4WheelHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace jaguar4x4wheel_base
