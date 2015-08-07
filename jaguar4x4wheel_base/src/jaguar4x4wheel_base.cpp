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
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

/**
* Control loop for Jaguar4x4Wheel, not realtime safe
*/
void controlLoop(jaguar4x4wheel_base::Jaguar4x4WheelHardware &jaguar4x4wheel,
                 controller_manager::ControllerManager &cm,
                 ros::Time &last_time)
{

  // Calculate monotonic time difference
  ros::Time this_time = ros::Time::now();
  ros::Duration elapsed = this_time - last_time;
  last_time = this_time;

  // Process control loop
  jaguar4x4wheel.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  jaguar4x4wheel.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jaguar4x4wheel_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // Initialize robot hardware and link to controller manager
  jaguar4x4wheel_base::Jaguar4x4WheelHardware jaguar4x4wheel(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&jaguar4x4wheel, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Jaguar4x4Wheel hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue jaguar4x4wheel_queue;
  ros::AsyncSpinner jaguar4x4wheel_spinner(1, &jaguar4x4wheel_queue);

  ros::Time last_time = ros::Time::now();
  ros::TimerOptions control_timer(
      ros::Duration(1 / control_frequency),
      boost::bind(controlLoop, boost::ref(jaguar4x4wheel), boost::ref(cm), boost::ref(last_time)),
      &jaguar4x4wheel_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  jaguar4x4wheel_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
