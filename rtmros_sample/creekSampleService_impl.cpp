// -*-C++-*-

#include "creekSampleService_impl.h"
#include "creekSample.h"

creekSampleService_impl::creekSampleService_impl()
  : m_comp(NULL)
{
}


creekSampleService_impl::~creekSampleService_impl()
{
}


void creekSampleService_impl::test()
{
  if(m_comp)
    m_comp->test();
}


// End of example implementational code



