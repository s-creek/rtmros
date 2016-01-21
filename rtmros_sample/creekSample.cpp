// -*- C++ -*-

#include "creekSample.h"

static const char* creeksample_spec[] =
  {
    "implementation_id", "creekSample",
    "type_name",         "creekSample",
    "description",       "creekSample",
    "version",           "1.0",
    "vendor",            "s-creek",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "creekSample",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

creekSample::creekSample(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_creekSampleServicePort("creekSampleService")
{
  m_service0.setComp(this);
}


creekSample::~creekSample()
{
}


RTC::ReturnCode_t creekSample::onInitialize()
{
  m_creekSampleServicePort.registerProvider("service0", "creekSampleService", m_service0);
  addPort(m_creekSampleServicePort);

  chatter_pub = nh.advertise<std_msgs::String>("chatter", 100);
  chatter_sub = nh.subscribe("chatter/rtm", 100, &creekSample::chatterCallback, this);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSample::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "creekSample::onActivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSample::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "creekSample::onDeactivated" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t creekSample::onExecute(RTC::UniqueId ec_id)
{
  ros::spinOnce();

  coil::TimeValue tv(coil::gettimeofday());
  std::stringstream ss;
  ss << "from rtc (" << tv.sec() << "." << tv.usec() << ")";

  std_msgs::String msg;
  msg.data = ss.str();

  chatter_pub.publish(msg);
  //ROS_INFO("I published [%s]", ss.str().c_str());

  std::cout << "creekSample::onExecute : " << tv.sec() << "." << tv.usec() << std::endl;
  return RTC::RTC_OK;
}


void creekSample::test()
{
  coil::TimeValue tv(coil::gettimeofday());
  std::stringstream ss;
  ss << "from rtc service (" << tv.sec() << "." << tv.usec() << ")";

  std_msgs::String msg;
  msg.data = ss.str();

  chatter_pub.publish(msg);
}


void creekSample::chatterCallback(const std_msgs::StringConstPtr& msg)
{
  //ROS_INFO("Received [%s]", msg->data.c_str());

  std_msgs::String send_msg;
  send_msg.data = "from ros (" + msg->data + ")";
  chatter_pub.publish(send_msg);
}


extern "C"
{
 
  void creekSampleInit(RTC::Manager* manager)
  {
    coil::Properties profile(creeksample_spec);
    manager->registerFactory(profile,
                             RTC::Create<creekSample>,
                             RTC::Delete<creekSample>);
  }
  
};



