/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef TEST_METHODS_HPP_
#define TEST_METHODS_HPP_

#include <sstream>
#include <string>

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <gtest/gtest.h>

#include <controller_interface/controller_base.h>
#include <ros/ros.h>

/**
 * @brief Test of the controller that sends messages from the Publisher to the handle
 *
 * @param[in] inputs Input values sent by the Publisher
 * @param[in] outputs Expected output values of the Handle
 * @param[in] publishers Publisher
 * @param[in] values Values to overwrite in the Handle
 * @param[in] controller ros_controller
 */
template <typename InputType, typename OutputType, size_t Size>
void RunOutputInterfaceTest(const boost::array<InputType, Size>& inputs, const boost::array<OutputType, Size>& outputs,
                            const boost::array<ros::Publisher, Size>& publishers,
                            const boost::array<OutputType, Size>& values,
                            controller_interface::ControllerBase* controller) {
  EXPECT_TRUE(controller != NULL) << "controller has an invalid address";
  if (controller == NULL) {
    return;
  }

  // Sending
  for (size_t i = 0; i < Size; ++i) {
    publishers[i].publish(inputs[i]);
  }

  // Update process
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  controller->update(ros::Time::now(), ros::Duration());
  ros::spinOnce();

  // Result verification
  for (size_t i = 0; i < Size; ++i) {
    std::stringstream sst;
    sst << "index : " << i;
    SCOPED_TRACE(sst.str().c_str());
    if (typeid(OutputType) == typeid(double)) {
      EXPECT_DOUBLE_EQ(values[i], outputs[i]);
    } else {
      EXPECT_EQ(values[i], outputs[i]);
    }
  }
  return;
}

/**
 * @brief Subscriber and received value management class
 */
template <typename OutputType>
class StateSubscriber {
  typedef boost::shared_ptr<OutputType const> OutputTypeConstPtr;

 private:
  StateSubscriber(StateSubscriber const&);             // = delete;
  StateSubscriber& operator=(StateSubscriber const&);  // = delete;

 public:
  StateSubscriber(const std::string& name, ros::NodeHandle* const nh) {
    subscriber_ = nh->subscribe<OutputType>(name, 1000, &StateSubscriber::CallBack, this);
  }

  OutputType GetLastMessage() const { return last_message_; }
  bool IsSubscribe() const { return is_subscribe_; }
  void ResetIsSubscribe() { is_subscribe_ = false; }

 private:
  void CallBack(const OutputTypeConstPtr& msg) {
    last_message_ = *msg;
    is_subscribe_ = true;
  }

  ros::Subscriber subscriber_;
  OutputType last_message_;
  bool is_subscribe_;
};

/**
 * @brief Test to receive handle values with the Subscriber
 *
 * @param[in] inputs Input values to set in values
 * @param[in] outputs Expected output values of subscribe_datas
 * @param[in] duration Waiting time
 * @param[in] values values
 * @param[in] subscribe_datas subscribe_datas
 * @param[in] compare Comparison function for subscribe_datas
 * @param[in] controller Controller
 */
template <typename InputType, typename OutputType, size_t Size>
void RunInputInterfaceTest(const boost::array<InputType, Size>& inputs, const boost::array<OutputType, Size>& outputs,
                           const ros::Duration& duration, boost::array<InputType, Size>& values,
                           const boost::array<boost::shared_ptr<StateSubscriber<OutputType> >, Size>& subscribe_datas,
                           boost::function<void(const OutputType&, const OutputType&)> compare,
                           controller_interface::ControllerBase* controller) {
  // Set initial values
  for (size_t i = 0; i < Size; ++i) {
    values[i] = inputs[i];
    subscribe_datas[i]->ResetIsSubscribe();
    EXPECT_FALSE(subscribe_datas[i]->IsSubscribe());
  }

  duration.sleep();
  controller->update(ros::Time::now(), ros::Duration());
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  for (size_t i = 0; i < Size; ++i) {
    std::stringstream sst;
    sst << "index : " << i;
    SCOPED_TRACE(sst.str().c_str());
    compare(subscribe_datas[i]->GetLastMessage(), outputs[i]);
    EXPECT_TRUE(subscribe_datas[i]->IsSubscribe());
  }
}

#endif  // TEST_METHODS_HPP_
