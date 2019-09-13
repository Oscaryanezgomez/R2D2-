// Generated by gencpp from file simulator/simulator_base.msg
// DO NOT EDIT!


#ifndef SIMULATOR_MESSAGE_SIMULATOR_BASE_H
#define SIMULATOR_MESSAGE_SIMULATOR_BASE_H

#include <ros/service_traits.h>


#include <simulator/simulator_baseRequest.h>
#include <simulator/simulator_baseResponse.h>


namespace simulator
{

struct simulator_base
{

typedef simulator_baseRequest Request;
typedef simulator_baseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simulator_base
} // namespace simulator


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::simulator::simulator_base > {
  static const char* value()
  {
    return "8e983985b8c042c16203b4cfe29be041";
  }

  static const char* value(const ::simulator::simulator_base&) { return value(); }
};

template<>
struct DataType< ::simulator::simulator_base > {
  static const char* value()
  {
    return "simulator/simulator_base";
  }

  static const char* value(const ::simulator::simulator_base&) { return value(); }
};


// service_traits::MD5Sum< ::simulator::simulator_baseRequest> should match 
// service_traits::MD5Sum< ::simulator::simulator_base > 
template<>
struct MD5Sum< ::simulator::simulator_baseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::simulator::simulator_base >::value();
  }
  static const char* value(const ::simulator::simulator_baseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::simulator::simulator_baseRequest> should match 
// service_traits::DataType< ::simulator::simulator_base > 
template<>
struct DataType< ::simulator::simulator_baseRequest>
{
  static const char* value()
  {
    return DataType< ::simulator::simulator_base >::value();
  }
  static const char* value(const ::simulator::simulator_baseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::simulator::simulator_baseResponse> should match 
// service_traits::MD5Sum< ::simulator::simulator_base > 
template<>
struct MD5Sum< ::simulator::simulator_baseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::simulator::simulator_base >::value();
  }
  static const char* value(const ::simulator::simulator_baseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::simulator::simulator_baseResponse> should match 
// service_traits::DataType< ::simulator::simulator_base > 
template<>
struct DataType< ::simulator::simulator_baseResponse>
{
  static const char* value()
  {
    return DataType< ::simulator::simulator_base >::value();
  }
  static const char* value(const ::simulator::simulator_baseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SIMULATOR_MESSAGE_SIMULATOR_BASE_H