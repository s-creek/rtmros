// -*- C++ -*-

#ifndef CREEKSAMPLE_H
#define CREEKSAMPLE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "creekSampleService_impl.h"

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace RTC;

class creekSample  : public RTC::DataFlowComponentBase
{
public:
  creekSample(RTC::Manager* manager);
  ~creekSample();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void test();
  void chatterCallback(const std_msgs::StringConstPtr& msg);


protected:
  RTC::CorbaPort m_creekSampleServicePort;
  creekSampleService_impl m_service0;

  ros::Publisher chatter_pub;
  ros::Subscriber chatter_sub;

private:
  ros::NodeHandle nh;
};


extern "C"
{
  DLL_EXPORT void creekSampleInit(RTC::Manager* manager);
};

#endif // CREEKSAMPLE_H

