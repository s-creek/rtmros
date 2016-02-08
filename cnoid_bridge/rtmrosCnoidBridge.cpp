// -*- C++ -*-

#include "rtmrosCnoidBridge.h"

#include <cnoid/BodyLoader>
#include <cnoid/VRMLBodyLoader>
#include <boost/shared_ptr.hpp>

static const char* rtmroscnoidbridge_spec[] =
  {
    "implementation_id", "rtmrosCnoidBridge",
    "type_name",         "rtmrosCnoidBridge",
    "description",       "rtmrosCnoidBridge",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "rtmrosCnoidBridge",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

rtmrosCnoidBridge::rtmrosCnoidBridge(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_rtmrosCnoidBridgeServicePort("rtmrosCnoidBridgeService"),
    m_qIn("qIn", m_q),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy)
{
}
  
rtmrosCnoidBridge::~rtmrosCnoidBridge()
{
}

RTC::ReturnCode_t rtmrosCnoidBridge::onInitialize()
{
  //
  // setup model
  //
  RTC::Properties& prop = getProperties();
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"] );

  cnoid::VRMLBodyLoader* vbl = dynamic_cast< cnoid::VRMLBodyLoader* >(bl.lastActualBodyLoader().get());
  cnoid::VRMLNodePtr node = vbl->getOriginalNode(m_robot->rootLink());
  m_rootLinkName = node->defName;
  std::cout << "root link name = " << m_rootLinkName << std::endl;


  //
  // init rtc
  //
  m_rtmrosCnoidBridgeServicePort.registerProvider("service0", "rtmrosCnoidBridgeService", m_service0);
  addPort(m_rtmrosCnoidBridgeServicePort);

  addInPort("qIn", m_qIn);
  addInPort("basePos", m_basePosIn);
  addInPort("baseRpy", m_baseRpyIn);

  m_basePos.data.x = m_robot->rootLink()->p()(0);
  m_basePos.data.y = m_robot->rootLink()->p()(1);
  m_basePos.data.z = m_robot->rootLink()->p()(2);
 
  m_baseRpy.data.r = 0.0;
  m_baseRpy.data.p = 0.0;
  m_baseRpy.data.y = 0.0;


  //
  // init ros
  //
  m_jointStatePub = m_nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  int n = m_robot->numJoints();
  m_jointStateMsg.name.resize(n);
  m_jointStateMsg.position.resize(n);
  for(int i=0; i<n; i++) {
    m_jointStateMsg.name[i]     = m_robot->joint(i)->name();
    m_jointStateMsg.position[i] = 0.0;
  }

  m_tfsMsg.header.frame_id = "global";
  m_tfsMsg.child_frame_id = m_rootLinkName;

  m_tfsMsg.transform.translation.x = m_basePos.data.x;
  m_tfsMsg.transform.translation.y = m_basePos.data.y;
  m_tfsMsg.transform.translation.z = m_basePos.data.z;
  m_tfsMsg.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);


  image_transport::ImageTransport it(m_nh);
  //m_imagePub = it.advertiseCamera("image_raw", 1);
  m_imagePub = it.advertise("image_raw", 1);


  return RTC::RTC_OK;
}

RTC::ReturnCode_t rtmrosCnoidBridge::onActivated(RTC::UniqueId ec_id)
{
  m_jointStateMsg.header.stamp = ros::Time::now();
  m_tfsMsg.header.stamp = ros::Time::now();

  m_jointStatePub.publish(m_jointStateMsg);
  m_tfbc.sendTransform(m_tfsMsg);


  return RTC::RTC_OK;
}

RTC::ReturnCode_t rtmrosCnoidBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t rtmrosCnoidBridge::onExecute(RTC::UniqueId ec_id)
{
  ros::spinOnce();


  //
  // joint angle
  //
  if( m_qIn.isNew() ) {
    m_qIn.read();

    if( m_q.data.length() == m_robot->numJoints() ) {
      for(int i=0; i<m_robot->numJoints(); i++) {
	m_jointStateMsg.position[i] = m_q.data[i];
      }
    }
    else {
      std::cerr << "error : joint num  ( in : " << m_q.data.length() << ",  model : " << m_robot->numJoints() << " )" << std::endl;
    }
  }
  m_jointStateMsg.header.stamp = ros::Time::now();
  m_jointStatePub.publish(m_jointStateMsg);


  //
  // base pos & rpy
  //
  if( m_basePosIn.isNew() || m_baseRpyIn.isNew() ) {
    m_basePosIn.read();
    m_baseRpyIn.read();

    m_tfsMsg.transform.translation.x = m_basePos.data.x;
    m_tfsMsg.transform.translation.y = m_basePos.data.y;
    m_tfsMsg.transform.translation.z = m_basePos.data.z;
    m_tfsMsg.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
  }
  m_tfsMsg.header.stamp = ros::Time::now();
  m_tfbc.sendTransform(m_tfsMsg);


  return RTC::RTC_OK;
}

extern "C"
{
 
  void rtmrosCnoidBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(rtmroscnoidbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<rtmrosCnoidBridge>,
                             RTC::Delete<rtmrosCnoidBridge>);
  }
  
};

