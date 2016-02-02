// -*- C++ -*-

#ifndef RTMROSCNOIDBRIDGE_H
#define RTMROSCNOIDBRIDGE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "rtmrosCnoidBridgeService_impl.h"

// cnoid
#include <cnoid/Body>

// ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

using namespace RTC;

class rtmrosCnoidBridge  : public RTC::DataFlowComponentBase
{
public:
  rtmrosCnoidBridge(RTC::Manager* manager);
  ~rtmrosCnoidBridge();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


protected: 
  RTC::CorbaPort m_rtmrosCnoidBridgeServicePort;
  rtmrosCnoidBridgeService_impl m_service0;


  RTC::TimedDoubleSeq m_q;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn;

  TimedPoint3D         m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  
  TimedOrientation3D         m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;


  ros::Publisher m_jointStatePub;
  sensor_msgs::JointState m_jointStateMsg;

  tf::TransformBroadcaster m_tfbc;
  geometry_msgs::TransformStamped m_tfsMsg;


private:
  cnoid::BodyPtr m_robot;
  std::string m_rootLinkName;

  ros::NodeHandle m_nh;
};


extern "C"
{
  DLL_EXPORT void rtmrosCnoidBridgeInit(RTC::Manager* manager);
};

#endif // RTMROSCNOIDBRIDGE_H

