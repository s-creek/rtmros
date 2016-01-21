// -*-C++-*-

#include "creekSampleService.hh"


#ifndef CREEKSAMPLESERVICE_IMPL_H
#define CREEKSAMPLESERVICE_IMPL_H
 
class creekSample;

class creekSampleService_impl
  : public virtual POA_OpenHRP::creekSampleService,
    public virtual PortableServer::RefCountServantBase
{
public:
  creekSampleService_impl();
  virtual ~creekSampleService_impl();

  void test();
  
  inline void setComp(creekSample *i_comp) { m_comp = i_comp; }

private:
  creekSample *m_comp;
};



#endif // CREEKSAMPLESERVICE_IMPL_H


