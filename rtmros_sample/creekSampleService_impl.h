// -*-C++-*-
/*!
 * @file  creekSampleService_impl.h
 * @brief Service implementation header of creekSampleService.idl
 *
 */

#include "creekSampleService.hh"


#ifndef CREEKSAMPLESERVICE_IMPL_H
#define CREEKSAMPLESERVICE_IMPL_H
 
/*
 * Example class implementing IDL interface OpenHRP::creekSampleService
 */
class creekSampleService_impl
 : public virtual POA_OpenHRP::creekSampleService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~creekSampleService_impl();

 public:
   // standard constructor
   creekSampleService_impl();
   virtual ~creekSampleService_impl();

   // attributes and operations
   void test();

};



#endif // CREEKSAMPLESERVICE_IMPL_H


