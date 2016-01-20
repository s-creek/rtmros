// -*- C++ -*-
/*!
 * @file  creekSample.cpp * @brief creekSample * $Date$ 
 *
 * $Id$ 
 */
#include "creekSample.h"

// Module specification
// <rtc-template block="module_spec">
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
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_creekSampleServicePort("creekSampleService")

    // </rtc-template>
{
}

creekSample::~creekSample()
{
}


RTC::ReturnCode_t creekSample::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports
  m_creekSampleServicePort.registerProvider("service0", "creekSampleService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_creekSampleServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t creekSample::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t creekSample::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


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



