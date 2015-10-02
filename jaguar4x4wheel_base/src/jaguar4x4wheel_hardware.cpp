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

using namespace DrRobot_MotionSensorDriver;

namespace {
  const float TICKS_PER_METER = 345; //345 based upon the diameter of the wheel including the track, 452 based upon the diamater of the wheel excluding the track. 345 works best inside the lab, 452 works best on the carpet outside
  const uint ENCODER_MIN = 0;
  const uint ENCODER_MAX = 32767;
  const uint PULSES_PER_REVOLUTION = 186;//190; // for speed encoder
}

namespace jaguar4x4wheel_base {

  /**
  * Initialize Jaguar4x4Wheel hardware
  */
  Jaguar4x4WheelHardware::Jaguar4x4WheelHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
      : nh_(nh),
        private_nh_(private_nh)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.27); // or 0.3555?
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/prolific");

    robot_config_.commMethod = DrRobot_MotionSensorDriver::Network;
    robot_config_.boardType = DrRobot_MotionSensorDriver::Jaguar;
    robot_config_.portNum = 10001;
    strncpy(robot_config_.robotIP, "172.16.51.52", sizeof(robot_config_.robotIP) - 1);
    drrobot_motion_driver_.setDrRobotMotionDriverConfig(&robot_config_);

    resetTravelOffset();
    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void Jaguar4x4WheelHardware::resetTravelOffset()
  {
    int res = drrobot_motion_driver_.openNetwork(robot_config_.robotIP,robot_config_.portNum);
    if (res == 0)
    {
      ROS_INFO("open port number at: [%d]", robot_config_.portNum);
    }
    else
    {
      ROS_ERROR("could not open network connection to [%s,%d]",  robot_config_.robotIP,robot_config_.portNum);
      ROS_ERROR("error code [%d]",  res);
    }
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void Jaguar4x4WheelHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("rear_left_wheel")("rear_right_wheel")("front_left_wheel")
                                ("front_right_wheel");
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
    if(drrobot_motion_driver_.portOpen())
    {
      drrobot_motion_driver_.readMotorSensorData(&motor_sensor_data_);

      // Translate from driver data to ROS data
      //motors used are 3 & 4
      //3 is left motor
      //4 is right motor
      for (uint j = 0 ; j < 4; ++j)
      {
        uint i = j;
        if(j > 1)
          j++;
//        if( j == 3)
//          ROS_DEBUG_STREAM(" motorSensorEncoderPos[i] "<<motor_sensor_data_.motorSensorEncoderPos[j]
//                           <<" motorSensorEncoderVel[i] "<<motor_sensor_data_.motorSensorEncoderVel[j]
//                           <<" motorSensorEncoderDir[i] "<<motor_sensor_data_.motorSensorEncoderDir[j]);
        double delta = double(motor_sensor_data_.motorSensorEncoderPos[j])/double(PULSES_PER_REVOLUTION)*2.0*M_PI;
        if(j == 1 || j == 4)
          delta = -delta;
        delta += - joints_[i].position_offset - joints_[i].position;

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0)
        {
          joints_[i].position += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
        }

        double velocity = double(motor_sensor_data_.motorSensorEncoderVel[j])/double(PULSES_PER_REVOLUTION)*2.0*M_PI;
        if(motor_sensor_data_.motorSensorEncoderDir[j] == 0)
          velocity = -velocity;

        joints_[i].velocity = velocity ;
        if(i == 0)
          ROS_DEBUG_STREAM(i<<" delta "<<angularToLinear(delta)<<" m velocity "<<angularToLinear(velocity)<<" m/s");
        if(i == 1)
          ROS_DEBUG_STREAM(i<<" delta "<<angularToLinear(delta)<<" m velocity "<<angularToLinear(velocity)<<" m/s");
      }
    }

  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void Jaguar4x4WheelHardware::writeCommandsToHardware()
  {
    double diff_speed_left_rear = joints_[0].velocity_command * PULSES_PER_REVOLUTION / (2*M_PI);
    double diff_speed_right_rear = -joints_[1].velocity_command * PULSES_PER_REVOLUTION / (2*M_PI);
    double diff_speed_left_front = joints_[2].velocity_command * PULSES_PER_REVOLUTION / (2*M_PI);
    double diff_speed_right_front = -joints_[3].velocity_command * PULSES_PER_REVOLUTION / (2*M_PI);

    // for Jaguar 4x4 independent drive, channel 0 for left rear motor, channel 1 for right rear motor, channel 3 for left front motor, channel 4 for right front motor, here we will use velocity control
    drrobot_motion_driver_.sendMotorCtrlAllCmd(Velocity,diff_speed_left_rear, diff_speed_right_rear,NOCONTROL,
                                                        diff_speed_left_front, diff_speed_right_front,NOCONTROL);
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
