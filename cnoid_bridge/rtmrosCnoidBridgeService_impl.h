// -*-C++-*-

#include "rtmrosCnoidBridgeService.hh"


#ifndef RTMROSCNOIDBRIDGESERVICE_IMPL_H
#define RTMROSCNOIDBRIDGESERVICE_IMPL_H
 
class rtmrosCnoidBridgeService_impl
 : public virtual POA_OpenHRP::rtmrosCnoidBridgeService,
   public virtual PortableServer::RefCountServantBase
{
 private:

 public:
   rtmrosCnoidBridgeService_impl();
   virtual ~rtmrosCnoidBridgeService_impl();

   void test();
};

#endif // RTMROSCNOIDBRIDGESERVICE_IMPL_H


